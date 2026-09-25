//! The translator: a syntax-directed walk from `syn`'s tree to WGSL text.
//!
//! Every construct outside the subset is refused where it is met, with its
//! line and column. The walk also tracks each expression's type, far enough
//! to refuse the places where Rust and WGSL would type or round a literal
//! differently. Other type errors are left to naga, which validates the whole
//! module afterwards.

use std::collections::{BTreeMap, BTreeSet};

use proc_macro2::Span;
use syn::spanned::Spanned;
use syn::{
    Attribute, BinOp, Block, Expr, ExprIf, Fields, FnArg, Item, ItemConst, ItemFn, ItemStruct, Lit,
    Local, Member, Pat, ReturnType, Stmt, Type, UnOp,
};

use crate::{Error, Source};

/// The methods the subset accepts, as `(Rust name, WGSL builtin, argument
/// count besides the receiver)`. `x.min(y)` becomes `min(x, y)`.
///
/// Left out on purpose: `round` and `signum`, whose Rust and WGSL
/// definitions differ (Rust rounds half away from zero, WGSL to even;
/// `signum(0.0)` is 1 in Rust and `sign(0.0)` is 0 in WGSL), and `mul_add`,
/// which Rust defines with one rounding and WGSL's `fma` does not.
pub const METHODS: &[(&str, &str, usize)] = &[
    ("abs", "abs", 0),
    ("sqrt", "sqrt", 0),
    ("ln", "log", 0),
    ("exp", "exp", 0),
    ("sin", "sin", 0),
    ("cos", "cos", 0),
    ("acos", "acos", 0),
    ("floor", "floor", 0),
    ("min", "min", 1),
    ("max", "max", 1),
    ("clamp", "clamp", 2),
];

/// Names the emitted WGSL relies on meaning the builtin. A source item or
/// binding with one of these names would silently capture the call.
const BUILTINS_USED: &[&str] = &["select", "array", "f32", "u32", "i32", "bool"];

/// The line length past which a statement's array or struct literal is
/// broken into one element per line.
const MAX_LINE: usize = 100;

/// The characters that end a WGSL line comment in naga's lexer.
const ENDS_A_COMMENT: [char; 7] = [
    '\n', '\u{b}', '\u{c}', '\r', '\u{85}', '\u{2028}', '\u{2029}',
];

/// The prefix of the names the translator gives its own temporaries.
const TEMPORARY: &str = "destructured_";

/// An expression's type, as far as the translator tracks it.
#[derive(Clone, Debug, PartialEq, Eq)]
enum Ty {
    /// The scalar alias `R`.
    Real,
    U32,
    I32,
    Bool,
    /// Float literals, and operations on nothing else: Rust has not fixed
    /// the type yet, and falls back to `f64` if nothing does.
    FloatLiteral,
    /// Integer literals, and operations on nothing else.
    IntLiteral,
    Array(Box<Self>, usize),
    Struct(String),
    /// Anything not tracked; naga checks it.
    Unknown,
}

impl Ty {
    /// The type two operands (or two `if` arms) share.
    fn unify(left: &Self, right: &Self) -> Self {
        match (left, right) {
            (same, other) if same == other => same.clone(),
            (Self::FloatLiteral, Self::Real) | (Self::Real, Self::FloatLiteral) => Self::Real,
            (Self::IntLiteral, other) | (other, Self::IntLiteral)
                if matches!(other, Self::U32 | Self::I32) =>
            {
                other.clone()
            }
            (Self::Array(left, length), Self::Array(right, other_length))
                if length == other_length =>
            {
                Self::Array(Box::new(Self::unify(left, right)), *length)
            }
            _ => Self::Unknown,
        }
    }

    /// Whether Rust could still pick `f64` for part of a value of this type.
    fn holds_float_literal(&self) -> bool {
        match self {
            Self::FloatLiteral => true,
            Self::Array(element, _) => element.holds_float_literal(),
            _ => false,
        }
    }
}

/// A translated expression and its type.
struct Typed {
    text: String,
    ty: Ty,
}

/// What every function can see: the sources' items.
#[derive(Default)]
struct Tables {
    /// Every item name, so the translator's temporaries avoid them.
    items: BTreeSet<String>,
    /// Each struct's fields, in declared order.
    structs: BTreeMap<String, Vec<(String, Ty)>>,
    /// Each function's return type.
    functions: BTreeMap<String, Ty>,
    /// Each constant's type.
    constants: BTreeMap<String, Ty>,
}

/// The WGSL module for `sources`, not yet validated.
pub fn module(sources: &[Source<'_>]) -> Result<String, Error> {
    if sources.is_empty() {
        return Err(Error::NoSources);
    }
    let files = sources
        .iter()
        .map(|source| {
            let file =
                syn::parse_file(source.text).map_err(|e| refusal(source.path, e.span(), &e))?;
            // Inner attributes (`#![cfg(…)]`) would apply to the whole file.
            file.attrs.first().map_or_else(
                || Ok(file.clone()),
                |attr| {
                    Err(refusal(
                        source.path,
                        attr.span(),
                        &"attributes are allowed only on items, struct fields and `let` statements",
                    ))
                },
            )
        })
        .collect::<Result<Vec<_>, _>>()?;
    let tables = tables(sources, &files)?;

    let mut out = String::from(
        "// GENERATED by sim-wgsl-gen from the Rust sources listed below. Do not edit\n\
         // this file: edit the sources and regenerate it (the freshness test that\n\
         // fails names the command). The Rust scalar type `R` is written as `f32`.\n\
         //\n\
         // Sources:\n",
    );
    for source in sources {
        push_line(&mut out, &format!("//   {}", source.path));
    }
    for (source, file) in sources.iter().zip(&files) {
        let translator = Translator {
            path: source.path,
            tables: &tables,
        };
        push_line(&mut out, &format!("\n// ---- {} ----", source.path));
        for item in &file.items {
            out.push('\n');
            translator.item(item, &mut out)?;
        }
    }
    Ok(out)
}

/// The item tables, in two passes: struct names first, since field,
/// parameter and return types may name them.
fn tables(sources: &[Source<'_>], files: &[syn::File]) -> Result<Tables, Error> {
    let mut tables = Tables::default();
    for (source, file) in sources.iter().zip(files) {
        for item in &file.items {
            let ident = match item {
                Item::Struct(s) => &s.ident,
                Item::Fn(f) => &f.sig.ident,
                Item::Const(c) => &c.ident,
                _ => continue,
            };
            let name = ident.to_string();
            if !tables.items.insert(name.clone()) {
                return Err(refusal(
                    source.path,
                    ident.span(),
                    &format!("`{name}` is declared twice"),
                ));
            }
            if let Item::Struct(_) = item {
                tables.structs.insert(name, Vec::new());
            }
        }
    }
    let mut structs = BTreeMap::new();
    let mut functions = BTreeMap::new();
    let mut constants = BTreeMap::new();
    for (source, file) in sources.iter().zip(files) {
        let translator = Translator {
            path: source.path,
            tables: &tables,
        };
        for item in &file.items {
            match item {
                // A generic item is refused when it is translated; its types
                // are not read here.
                Item::Struct(s) if s.generics.params.is_empty() => {
                    let mut fields = Vec::new();
                    for field in &s.fields {
                        if let Some(ident) = &field.ident {
                            fields.push((ident.to_string(), translator.ty(&field.ty)?.1));
                        }
                    }
                    structs.insert(s.ident.to_string(), fields);
                }
                Item::Fn(f) if f.sig.generics.params.is_empty() => {
                    if let ReturnType::Type(_, returns) = &f.sig.output {
                        functions.insert(f.sig.ident.to_string(), translator.ty(returns)?.1);
                    }
                }
                Item::Const(c) => {
                    constants.insert(c.ident.to_string(), translator.ty(&c.ty)?.1);
                }
                _ => {}
            }
        }
    }
    tables.structs = structs;
    tables.functions = functions;
    tables.constants = constants;
    Ok(tables)
}

fn refusal(path: &str, span: Span, message: &dyn std::fmt::Display) -> Error {
    let start = span.start();
    Error::Refused {
        path: path.to_string(),
        line: start.line,
        column: start.column + 1,
        message: message.to_string(),
    }
}

struct Translator<'a> {
    path: &'a str,
    tables: &'a Tables,
}

/// The names bound in one function and their types. WGSL forbids
/// redeclaring a name in one scope.
#[derive(Default)]
struct Scope {
    names: BTreeMap<String, Ty>,
}

/// Where an attribute sits, which decides the attributes allowed on it.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Site {
    Function,
    Struct,
    Field,
    Constant,
    Statement,
}

impl Translator<'_> {
    fn refuse(&self, span: Span, message: impl std::fmt::Display) -> Error {
        refusal(self.path, span, &message)
    }

    // ---- items ----

    fn item(&self, item: &Item, out: &mut String) -> Result<(), Error> {
        match item {
            Item::Fn(f) => self.function(f, out),
            Item::Struct(s) => self.structure(s, out),
            Item::Const(c) => self.constant(c, out),
            other => Err(self.refuse(
                other.span(),
                "only `fn`, `struct` and `const` items are in the subset",
            )),
        }
    }

    /// Checks `attrs` for `site` and returns the doc-comment lines. `anchor`
    /// locates a refusal when there is no attribute to point at.
    fn attributes(
        &self,
        attrs: &[Attribute],
        site: Site,
        anchor: Span,
    ) -> Result<Vec<String>, Error> {
        let mut docs = Vec::new();
        let mut repr_c = false;
        for attr in attrs {
            let name = attr
                .path()
                .get_ident()
                .map(ToString::to_string)
                .unwrap_or_default();
            match name.as_str() {
                "doc" if site != Site::Statement => {
                    if let syn::Meta::NameValue(nv) = &attr.meta
                        && let Expr::Lit(lit) = &nv.value
                        && let Lit::Str(text) = &lit.lit
                    {
                        // Split at every character that ends a WGSL comment
                        // (naga's lexer), so no part of a doc string leaves it.
                        let text = text.value().replace("\r\n", "\n");
                        for line in text.split(ENDS_A_COMMENT) {
                            docs.push(line.strip_prefix(' ').unwrap_or(line).to_string());
                        }
                    }
                }
                "allow" | "expect" => {}
                "must_use" | "inline" if site == Site::Function => {}
                "derive" if site == Site::Struct => {}
                "repr" if site == Site::Struct => {
                    let is_c = attr
                        .parse_args::<syn::Ident>()
                        .is_ok_and(|ident| ident == "C");
                    if !is_c {
                        return Err(self.refuse(attr.span(), "only `#[repr(C)]` is in the subset"));
                    }
                    repr_c = true;
                }
                _ => {
                    return Err(self.refuse(
                        attr.span(),
                        "this attribute is not in the subset: it could make the Rust and the \
                         WGSL differ",
                    ));
                }
            }
        }
        if site == Site::Struct && !repr_c {
            return Err(self.refuse(
                anchor,
                "a shared struct needs `#[repr(C)]`, so its layout is the one the GPU reads",
            ));
        }
        Ok(docs)
    }

    /// Refuses any attribute where the subset takes none: on expressions,
    /// parameters, patterns and the fields of a struct literal. A `#[cfg]`
    /// there would remove code from the Rust and not from the WGSL.
    fn no_attributes(&self, attrs: &[Attribute]) -> Result<(), Error> {
        attrs.first().map_or(Ok(()), |attr| {
            Err(self.refuse(
                attr.span(),
                "attributes are allowed only on items, struct fields and `let` statements",
            ))
        })
    }

    fn declare_item_name(&self, ident: &syn::Ident) -> Result<String, Error> {
        let name = ident.to_string();
        if BUILTINS_USED.contains(&name.as_str()) || METHODS.iter().any(|m| m.1 == name) {
            return Err(self.refuse(
                ident.span(),
                format!("`{name}` is a WGSL builtin the translation calls; rename it"),
            ));
        }
        Ok(name)
    }

    fn function(&self, f: &ItemFn, out: &mut String) -> Result<(), Error> {
        let docs = self.attributes(&f.attrs, Site::Function, f.sig.ident.span())?;
        let sig = &f.sig;
        if sig.asyncness.is_some() || sig.unsafety.is_some() || sig.abi.is_some() {
            return Err(self.refuse(
                sig.span(),
                "`async`, `unsafe` and `extern` functions are not in the subset",
            ));
        }
        if !sig.generics.params.is_empty() || sig.generics.where_clause.is_some() {
            return Err(self.refuse(
                sig.generics.span(),
                "generics are not in the subset: write against `R`",
            ));
        }
        let name = self.declare_item_name(&sig.ident)?;
        let mut scope = Scope::default();
        let mut params = Vec::new();
        for input in &sig.inputs {
            match input {
                FnArg::Receiver(r) => {
                    return Err(self.refuse(r.span(), "`self` is not in the subset: pass values"));
                }
                FnArg::Typed(typed) => {
                    self.no_attributes(&typed.attrs)?;
                    let (text, ty) = self.ty(&typed.ty)?;
                    let param = self.binding(&typed.pat, ty, &mut scope)?;
                    params.push(format!("{param}: {text}"));
                }
            }
        }
        let ReturnType::Type(_, returns) = &sig.output else {
            return Err(self.refuse(
                sig.span(),
                "a shared function must return a value: it has no side effects",
            ));
        };
        let returns = self.ty(returns)?.0;

        comments(out, &docs, "");
        push_line(
            out,
            &format!("fn {name}({}) -> {returns} {{", params.join(", ")),
        );
        self.body(&f.block, &mut scope, out)?;
        out.push_str("}\n");
        Ok(())
    }

    fn structure(&self, s: &ItemStruct, out: &mut String) -> Result<(), Error> {
        let docs = self.attributes(&s.attrs, Site::Struct, s.ident.span())?;
        if !s.generics.params.is_empty() {
            return Err(self.refuse(s.generics.span(), "generic structs are not in the subset"));
        }
        let name = self.declare_item_name(&s.ident)?;
        let Fields::Named(fields) = &s.fields else {
            return Err(self.refuse(s.ident.span(), "a shared struct needs named fields"));
        };
        comments(out, &docs, "");
        push_line(out, &format!("struct {name} {{"));
        for field in &fields.named {
            let field_docs = self.attributes(&field.attrs, Site::Field, field.span())?;
            comments(out, &field_docs, "    ");
            let ident = field
                .ident
                .as_ref()
                .ok_or_else(|| self.refuse(field.span(), "a shared struct needs named fields"))?;
            push_line(out, &format!("    {ident}: {},", self.ty(&field.ty)?.0));
        }
        out.push_str("}\n");
        Ok(())
    }

    fn constant(&self, c: &ItemConst, out: &mut String) -> Result<(), Error> {
        let docs = self.attributes(&c.attrs, Site::Constant, c.ident.span())?;
        let name = self.declare_item_name(&c.ident)?;
        let value = self.expr(&c.expr, &Scope::default())?;
        comments(out, &docs, "");
        push_line(
            out,
            &format!("const {name}: {} = {};", self.ty(&c.ty)?.0, value.text),
        );
        Ok(())
    }

    // ---- statements ----

    fn body(&self, block: &Block, scope: &mut Scope, out: &mut String) -> Result<(), Error> {
        let Some((last, lets)) = block.stmts.split_last() else {
            return Err(self.refuse(block.span(), "an empty body has no value"));
        };
        for stmt in lets {
            match stmt {
                Stmt::Local(local) => self.local(local, scope, out)?,
                Stmt::Expr(e, _) => {
                    return Err(self.refuse(
                        e.span(),
                        "only `let` bindings may come before the final expression",
                    ));
                }
                other => {
                    return Err(self.refuse(
                        other.span(),
                        "nested items and macros are not in the subset",
                    ));
                }
            }
        }
        match last {
            Stmt::Expr(e, None) => {
                let value = self.statement_value(e, scope, "    return ".len())?;
                push_line(out, &format!("    return {value};"));
                Ok(())
            }
            other => Err(self.refuse(
                other.span(),
                "the body must end in an expression without `;`: the function's value",
            )),
        }
    }

    fn local(&self, local: &Local, scope: &mut Scope, out: &mut String) -> Result<(), Error> {
        self.attributes(&local.attrs, Site::Statement, local.span())?;
        let Some(init) = &local.init else {
            return Err(self.refuse(
                local.span(),
                "a `let` needs a value: nothing is assigned later",
            ));
        };
        if let Some((_, diverge)) = &init.diverge {
            return Err(self.refuse(diverge.span(), "`let … else` is not in the subset"));
        }
        let ty = self.expr(&init.expr, scope)?.ty;
        let value = self.statement_value(&init.expr, scope, "    let ".len() + 16)?;
        let unpinned = |span: Span| {
            self.refuse(
                span,
                "this `let` holds only literals, so give it a type (`let x: R = …`): \
                 unconstrained, Rust makes it f64 and WGSL makes it f32",
            )
        };
        match &local.pat {
            Pat::Ident(_) => {
                if ty.holds_float_literal() {
                    return Err(unpinned(local.pat.span()));
                }
                let name = self.binding(&local.pat, ty, scope)?;
                push_line(out, &format!("    let {name} = {value};"));
            }
            Pat::Type(typed) => {
                self.no_attributes(&typed.attrs)?;
                let (text, declared) = self.ty(&typed.ty)?;
                let name = self.binding(&typed.pat, declared, scope)?;
                push_line(out, &format!("    let {name}: {text} = {value};"));
            }
            Pat::Slice(slice) => {
                self.no_attributes(&slice.attrs)?;
                if ty.holds_float_literal() {
                    return Err(unpinned(local.pat.span()));
                }
                let element = match &ty {
                    Ty::Array(element, _) => element.as_ref().clone(),
                    _ => Ty::Unknown,
                };
                // A name, or a field of one, is indexed in place; anything
                // else is bound once, so it is not evaluated once per element.
                let base = if is_place(&init.expr) {
                    value
                } else {
                    let temporary = self.temporary(scope);
                    self.declare(&temporary, local.span(), ty, scope)?;
                    push_line(out, &format!("    let {temporary} = {value};"));
                    temporary
                };
                for (index, element_pattern) in slice.elems.iter().enumerate() {
                    if let Pat::Wild(wild) = element_pattern {
                        self.no_attributes(&wild.attrs)?;
                        continue;
                    }
                    let name = self.binding(element_pattern, element.clone(), scope)?;
                    push_line(out, &format!("    let {name} = {base}[{index}];"));
                }
            }
            other => {
                return Err(self.refuse(
                    other.span(),
                    "only `let name`, `let name: T` and `let [a, b, …]` bindings are in the subset",
                ));
            }
        }
        Ok(())
    }

    /// A name for a translator temporary that no item or binding uses.
    fn temporary(&self, scope: &Scope) -> String {
        // One more candidate than names in use, so one is always free.
        (1..=scope.names.len() + self.tables.items.len() + 1)
            .map(|n| format!("{TEMPORARY}{n}"))
            .find(|name| !scope.names.contains_key(name) && !self.tables.items.contains(name))
            .unwrap_or_default()
    }

    /// A plain name pattern of type `ty`, declared in `scope`.
    fn binding(&self, pat: &Pat, ty: Ty, scope: &mut Scope) -> Result<String, Error> {
        let Pat::Ident(ident) = pat else {
            return Err(self.refuse(pat.span(), "only a plain name can be bound here"));
        };
        self.no_attributes(&ident.attrs)?;
        if let Some(mutability) = &ident.mutability {
            return Err(self.refuse(
                mutability.span(),
                "`mut` is not in the subset: nothing is mutated",
            ));
        }
        if ident.by_ref.is_some() || ident.subpat.is_some() {
            return Err(self.refuse(ident.span(), "only a plain name can be bound here"));
        }
        let name = ident.ident.to_string();
        self.declare(&name, ident.span(), ty, scope)?;
        Ok(name)
    }

    fn declare(&self, name: &str, span: Span, ty: Ty, scope: &mut Scope) -> Result<(), Error> {
        if BUILTINS_USED.contains(&name) || METHODS.iter().any(|m| m.1 == name) {
            return Err(self.refuse(
                span,
                format!("`{name}` is a WGSL builtin the translation calls; rename it"),
            ));
        }
        if scope.names.insert(name.to_string(), ty).is_some() {
            return Err(self.refuse(
                span,
                format!("`{name}` is already bound in this function, and WGSL cannot redeclare it"),
            ));
        }
        Ok(())
    }

    // ---- expressions ----

    fn expr(&self, e: &Expr, scope: &Scope) -> Result<Typed, Error> {
        self.no_attributes(expression_attributes(e))?;
        match e {
            Expr::Lit(lit) => self.literal(&lit.lit),
            Expr::Path(path) => {
                let name = self.name(path)?;
                let ty = scope
                    .names
                    .get(&name)
                    .or_else(|| self.tables.constants.get(&name))
                    .cloned()
                    .unwrap_or(Ty::Unknown);
                Ok(Typed { text: name, ty })
            }
            Expr::Paren(inner) => {
                let inner = self.expr(&inner.expr, scope)?;
                Ok(Typed {
                    text: format!("({})", inner.text),
                    ty: inner.ty,
                })
            }
            Expr::Binary(binary) => self.binary(binary, scope),
            Expr::Unary(unary) => self.unary(unary, scope),
            Expr::Call(call) => self.call(call, scope),
            Expr::MethodCall(call) => self.method(call, scope),
            Expr::Field(field) => self.field(field, scope),
            Expr::Index(index) => self.index(index, scope),
            Expr::Array(array) => {
                if array.elems.is_empty() {
                    return Err(self.refuse(array.span(), "an empty array has no WGSL type"));
                }
                let elements = self.arguments(array.elems.iter(), scope)?;
                let ty = elements
                    .iter()
                    .map(|element| element.ty.clone())
                    .reduce(|a, b| Ty::unify(&a, &b))
                    .unwrap_or(Ty::Unknown);
                Ok(Typed {
                    text: format!("array({})", join(&elements)),
                    ty: Ty::Array(Box::new(ty), elements.len()),
                })
            }
            Expr::Struct(literal) => {
                let (name, arguments) = self.struct_arguments(literal, scope)?;
                Ok(Typed {
                    text: format!("{name}({})", arguments.join(", ")),
                    ty: Ty::Struct(name),
                })
            }
            Expr::If(conditional) => self.conditional(conditional, scope),
            Expr::Cast(cast) => self.cast(cast, scope),
            _ => Err(self.refused(e)),
        }
    }

    fn binary(&self, binary: &syn::ExprBinary, scope: &Scope) -> Result<Typed, Error> {
        let op = binary_operator(&binary.op).ok_or_else(|| {
            self.refuse(
                binary.op.span(),
                "compound assignment is mutation, which is not in the subset",
            )
        })?;
        let left = self.operand(&binary.left, scope)?;
        let right = self.operand(&binary.right, scope)?;
        if left.ty == Ty::FloatLiteral && right.ty == Ty::FloatLiteral {
            return Err(self.refuse(
                binary.span(),
                "an operation on float literals alone: Rust may type it f64, WGSL f32, and \
                 they fold it differently; write its value as one literal, or name its type",
            ));
        }
        let ty = match op {
            "==" | "!=" | "<" | "<=" | ">" | ">=" | "&&" | "||" => Ty::Bool,
            "<<" | ">>" => left.ty,
            _ => Ty::unify(&left.ty, &right.ty),
        };
        Ok(Typed {
            text: format!("{} {op} {}", left.text, right.text),
            ty,
        })
    }

    fn unary(&self, unary: &syn::ExprUnary, scope: &Scope) -> Result<Typed, Error> {
        let op = match unary.op {
            UnOp::Neg(_) => "-",
            UnOp::Not(_) => "!",
            _ => return Err(self.refuse(unary.span(), "dereferencing is not in the subset")),
        };
        let operand = match unary.expr.as_ref() {
            Expr::Unary(_) => {
                let inner = self.expr(&unary.expr, scope)?;
                Typed {
                    text: format!("({})", inner.text),
                    ty: inner.ty,
                }
            }
            other => self.operand(other, scope)?,
        };
        Ok(Typed {
            text: format!("{op}{}", operand.text),
            ty: operand.ty,
        })
    }

    fn call(&self, call: &syn::ExprCall, scope: &Scope) -> Result<Typed, Error> {
        let Expr::Path(path) = call.func.as_ref() else {
            return Err(self.refuse(
                call.func.span(),
                "only a function named directly can be called",
            ));
        };
        self.no_attributes(&path.attrs)?;
        let name = self.name(path)?;
        let arguments = self.arguments(call.args.iter(), scope)?;
        let ty = self
            .tables
            .functions
            .get(&name)
            .cloned()
            .unwrap_or(Ty::Unknown);
        Ok(Typed {
            text: format!("{name}({})", join(&arguments)),
            ty,
        })
    }

    fn method(&self, call: &syn::ExprMethodCall, scope: &Scope) -> Result<Typed, Error> {
        if call.turbofish.is_some() {
            return Err(self.refuse(call.span(), "turbofish is not in the subset"));
        }
        let method = call.method.to_string();
        let Some((_, builtin, arity)) = METHODS.iter().find(|m| m.0 == method) else {
            let known: Vec<&str> = METHODS.iter().map(|m| m.0).collect();
            return Err(self.refuse(
                call.method.span(),
                format!(
                    "`.{method}()` is not in the subset's method table ({})",
                    known.join(", ")
                ),
            ));
        };
        if call.args.len() != *arity {
            return Err(self.refuse(
                call.span(),
                format!("`.{method}()` takes {arity} argument(s) besides its receiver"),
            ));
        }
        let receiver = self.delimited(&call.receiver, scope)?;
        let arguments = self.arguments(call.args.iter(), scope)?;
        let ty = arguments.iter().fold(receiver.ty.clone(), |ty, argument| {
            Ty::unify(&ty, &argument.ty)
        });
        let text = if arguments.is_empty() {
            format!("{builtin}({})", receiver.text)
        } else {
            format!("{builtin}({}, {})", receiver.text, join(&arguments))
        };
        Ok(Typed { text, ty })
    }

    fn field(&self, field: &syn::ExprField, scope: &Scope) -> Result<Typed, Error> {
        let Member::Named(member) = &field.member else {
            return Err(self.refuse(field.span(), "tuple fields are not in the subset"));
        };
        let base = self.operand(&field.base, scope)?;
        let ty = match &base.ty {
            Ty::Struct(name) => self
                .tables
                .structs
                .get(name)
                .and_then(|fields| fields.iter().find(|(f, _)| member == f))
                .map_or(Ty::Unknown, |(_, ty)| ty.clone()),
            _ => Ty::Unknown,
        };
        Ok(Typed {
            text: format!("{}.{member}", base.text),
            ty,
        })
    }

    fn index(&self, index: &syn::ExprIndex, scope: &Scope) -> Result<Typed, Error> {
        let Expr::Lit(syn::ExprLit {
            lit: Lit::Int(position),
            attrs,
        }) = index.index.as_ref()
        else {
            return Err(self.refuse(
                index.index.span(),
                "an index must be an integer literal: a runtime index is storage access, \
                 which belongs to the executor",
            ));
        };
        self.no_attributes(attrs)?;
        let base = self.operand(&index.expr, scope)?;
        let ty = match base.ty {
            Ty::Array(element, _) => *element,
            _ => Ty::Unknown,
        };
        Ok(Typed {
            text: format!("{}[{}]", base.text, position.base10_digits()),
            ty,
        })
    }

    fn cast(&self, cast: &syn::ExprCast, scope: &Scope) -> Result<Typed, Error> {
        let (target, ty) = self.ty(&cast.ty)?;
        let operand = self.delimited(&cast.expr, scope)?;
        let allowed = match ty {
            // To `R` only from an integer: a float literal cast to `R` is
            // `f64` in Rust first.
            Ty::Real => matches!(operand.ty, Ty::U32 | Ty::I32 | Ty::Unknown),
            Ty::U32 | Ty::I32 => matches!(
                operand.ty,
                Ty::Real | Ty::U32 | Ty::I32 | Ty::IntLiteral | Ty::Unknown
            ),
            _ => {
                return Err(self.refuse(cast.ty.span(), "a cast must be to `R`, `u32` or `i32`"));
            }
        };
        if !allowed {
            return Err(self.refuse(
                cast.span(),
                "this cast is not in the subset: cast to `R` only from `u32` or `i32`, and \
                 never cast a float literal (Rust would read it as f64)",
            ));
        }
        Ok(Typed {
            text: format!("{target}({})", operand.text),
            ty,
        })
    }

    /// The refusal for an expression outside the subset.
    fn refused(&self, e: &Expr) -> Error {
        let message = match e {
            Expr::ForLoop(_) | Expr::While(_) | Expr::Loop(_) => {
                "loops are not in the subset: shared math is written loop-free (plan §13d rule 1)"
            }
            Expr::Assign(_) => "assignment is mutation, which is not in the subset",
            Expr::Match(_) => "`match` is not in the subset: use `if`/`else`",
            Expr::Return(_) => "`return` is not in the subset: end the body in its value",
            Expr::Closure(_) => "closures are not in the subset",
            Expr::Macro(_) => "macros are not in the subset",
            Expr::Reference(_) => "references are not in the subset: pass values",
            _ => "this expression is not in the subset",
        };
        self.refuse(e.span(), message)
    }

    /// `e`, parenthesized when it is itself an operation, so the WGSL groups
    /// exactly as the Rust did (WGSL also refuses some unparenthesized mixes,
    /// such as `a && b || c`).
    fn operand(&self, e: &Expr, scope: &Scope) -> Result<Typed, Error> {
        let typed = self.expr(e, scope)?;
        Ok(if matches!(e, Expr::Binary(_)) {
            Typed {
                text: format!("({})", typed.text),
                ty: typed.ty,
            }
        } else {
            typed
        })
    }

    fn arguments<'e>(
        &self,
        args: impl Iterator<Item = &'e Expr>,
        scope: &Scope,
    ) -> Result<Vec<Typed>, Error> {
        args.map(|a| self.delimited(a, scope)).collect()
    }

    fn literal(&self, lit: &Lit) -> Result<Typed, Error> {
        match lit {
            Lit::Float(float) => {
                if !float.suffix().is_empty() {
                    return Err(self.refuse(
                        float.span(),
                        "a float literal takes no suffix: its type is `R`",
                    ));
                }
                let digits = float.base10_digits();
                let text = if digits.contains(['.', 'e', 'E']) {
                    digits.to_string()
                } else {
                    format!("{digits}.0")
                };
                Ok(Typed {
                    text,
                    ty: Ty::FloatLiteral,
                })
            }
            Lit::Int(int) => {
                let digits = int.base10_digits();
                let (text, ty) = match int.suffix() {
                    "" => (digits.to_string(), Ty::IntLiteral),
                    "u32" => (format!("{digits}u"), Ty::U32),
                    "i32" => (format!("{digits}i"), Ty::I32),
                    _ => {
                        return Err(self.refuse(
                            int.span(),
                            "an integer literal must be untyped, `u32` or `i32`",
                        ));
                    }
                };
                Ok(Typed { text, ty })
            }
            Lit::Bool(b) => Ok(Typed {
                text: b.value.to_string(),
                ty: Ty::Bool,
            }),
            other => Err(self.refuse(
                other.span(),
                "only numbers and booleans are literals in the subset",
            )),
        }
    }

    /// A value named by a single identifier.
    fn name(&self, path: &syn::ExprPath) -> Result<String, Error> {
        let single = path.qself.is_none()
            && path.path.leading_colon.is_none()
            && path.path.segments.len() == 1
            && path.path.segments[0].arguments.is_empty();
        if !single {
            return Err(self.refuse(
                path.span(),
                "only a plain name is in the subset: shared math has one namespace, as WGSL does",
            ));
        }
        let name = path.path.segments[0].ident.to_string();
        match name.as_str() {
            "self" => Err(self.refuse(path.span(), "`self` is not in the subset: pass values")),
            "R" => Err(self.refuse(path.span(), "`R` is a type, not a value")),
            _ => Ok(name),
        }
    }

    /// A struct literal's name and its field values in declared order.
    fn struct_arguments(
        &self,
        literal: &syn::ExprStruct,
        scope: &Scope,
    ) -> Result<(String, Vec<String>), Error> {
        if let Some(rest) = &literal.rest {
            return Err(self.refuse(
                rest.span(),
                "`..base` is not in the subset: name every field",
            ));
        }
        let name = literal
            .path
            .get_ident()
            .map(ToString::to_string)
            .unwrap_or_default();
        let Some(order) = self.tables.structs.get(&name) else {
            return Err(self.refuse(
                literal.path.span(),
                "only a struct declared in the sources can be built",
            ));
        };
        let mut values = BTreeMap::new();
        for field in &literal.fields {
            self.no_attributes(&field.attrs)?;
            let Member::Named(member) = &field.member else {
                return Err(self.refuse(field.span(), "tuple fields are not in the subset"));
            };
            values.insert(member.to_string(), self.delimited(&field.expr, scope)?.text);
        }
        let mut arguments = Vec::new();
        for (field, _) in order {
            let value = values.remove(field).ok_or_else(|| {
                self.refuse(
                    literal.span(),
                    format!("field `{field}` of `{name}` is missing"),
                )
            })?;
            arguments.push(value);
        }
        Ok((name, arguments))
    }

    /// `e` where its position already delimits it (an argument, a `let`
    /// value, a returned value), so redundant outer parentheses are dropped.
    fn delimited(&self, mut e: &Expr, scope: &Scope) -> Result<Typed, Error> {
        while let Expr::Paren(inner) = e {
            self.no_attributes(&inner.attrs)?;
            e = &inner.expr;
        }
        self.expr(e, scope)
    }

    /// A statement's value. An array or struct literal too long for one line
    /// (after `lead` characters) gets one element per line.
    fn statement_value(&self, e: &Expr, scope: &Scope, lead: usize) -> Result<String, Error> {
        let one_line = self.delimited(e, scope)?.text;
        if lead + one_line.len() <= MAX_LINE {
            return Ok(one_line);
        }
        let (head, items) = match e {
            Expr::Array(array) => (
                "array".to_string(),
                self.arguments(array.elems.iter(), scope)?
                    .into_iter()
                    .map(|element| element.text)
                    .collect(),
            ),
            Expr::Struct(literal) => self.struct_arguments(literal, scope)?,
            _ => return Ok(one_line),
        };
        let mut text = format!("{head}(\n");
        for item in items {
            text.push_str("        ");
            text.push_str(&item);
            text.push_str(",\n");
        }
        text.push_str("    )");
        Ok(text)
    }

    fn conditional(&self, conditional: &ExprIf, scope: &Scope) -> Result<Typed, Error> {
        let condition = self.delimited(&conditional.cond, scope)?;
        let then = self.branch(&conditional.then_branch, scope)?;
        let Some((_, otherwise)) = &conditional.else_branch else {
            return Err(self.refuse(
                conditional.span(),
                "an `if` needs an `else`: in the subset it is an expression",
            ));
        };
        let otherwise = match otherwise.as_ref() {
            Expr::Block(block) if block.attrs.is_empty() && block.label.is_none() => {
                self.branch(&block.block, scope)?
            }
            Expr::If(nested) => self.conditional(nested, scope)?,
            other => return Err(self.refuse(other.span(), "this `else` is not in the subset")),
        };
        Ok(Typed {
            text: format!(
                "select({}, {}, {})",
                otherwise.text, then.text, condition.text
            ),
            ty: Ty::unify(&then.ty, &otherwise.ty),
        })
    }

    /// An `if` arm: a block holding one expression.
    fn branch(&self, block: &Block, scope: &Scope) -> Result<Typed, Error> {
        match block.stmts.as_slice() {
            [Stmt::Expr(e, None)] => self.delimited(e, scope),
            _ => Err(self.refuse(
                block.span(),
                "an `if` arm must be a single expression: both arms become WGSL `select` operands",
            )),
        }
    }

    // ---- types ----

    /// A type's WGSL spelling and its tracked type.
    fn ty(&self, ty: &Type) -> Result<(String, Ty), Error> {
        match ty {
            Type::Path(path)
                if path.qself.is_none()
                    && path.path.segments.len() == 1
                    && path.path.segments[0].arguments.is_empty() =>
            {
                let name = path.path.segments[0].ident.to_string();
                match name.as_str() {
                    "R" => Ok(("f32".to_string(), Ty::Real)),
                    "u32" => Ok((name, Ty::U32)),
                    "i32" => Ok((name, Ty::I32)),
                    "bool" => Ok((name, Ty::Bool)),
                    "f32" | "f64" => Err(self.refuse(
                        ty.span(),
                        format!("`{name}` is not in the subset: write `R`, or the f32 and f64 builds differ"),
                    )),
                    _ if self.tables.structs.contains_key(&name) => {
                        Ok((name.clone(), Ty::Struct(name)))
                    }
                    _ => Err(self.refuse(ty.span(), format!("type `{name}` is not in the subset"))),
                }
            }
            Type::Array(array) => {
                let length = match &array.len {
                    Expr::Lit(syn::ExprLit {
                        lit: Lit::Int(len),
                        attrs,
                    }) => {
                        self.no_attributes(attrs)?;
                        len.base10_parse::<usize>().ok()
                    }
                    _ => None,
                };
                let Some(length) = length else {
                    return Err(self.refuse(
                        array.len.span(),
                        "an array length must be an integer literal",
                    ));
                };
                let (element, element_ty) = self.ty(&array.elem)?;
                Ok((
                    format!("array<{element}, {length}>"),
                    Ty::Array(Box::new(element_ty), length),
                ))
            }
            Type::Paren(inner) => self.ty(&inner.elem),
            other => Err(self.refuse(other.span(), "this type is not in the subset")),
        }
    }
}

/// Whether `e` is a name, or a field of one: cheap and pure to repeat.
fn is_place(e: &Expr) -> bool {
    match e {
        Expr::Path(_) => true,
        Expr::Field(field) => is_place(&field.base),
        _ => false,
    }
}

/// The attributes an expression the subset handles can carry.
fn expression_attributes(e: &Expr) -> &[Attribute] {
    match e {
        Expr::Lit(x) => &x.attrs,
        Expr::Path(x) => &x.attrs,
        Expr::Paren(x) => &x.attrs,
        Expr::Binary(x) => &x.attrs,
        Expr::Unary(x) => &x.attrs,
        Expr::Call(x) => &x.attrs,
        Expr::MethodCall(x) => &x.attrs,
        Expr::Field(x) => &x.attrs,
        Expr::Index(x) => &x.attrs,
        Expr::Array(x) => &x.attrs,
        Expr::Struct(x) => &x.attrs,
        Expr::If(x) => &x.attrs,
        Expr::Cast(x) => &x.attrs,
        Expr::Block(x) => &x.attrs,
        _ => &[],
    }
}

fn join(typed: &[Typed]) -> String {
    typed
        .iter()
        .map(|t| t.text.as_str())
        .collect::<Vec<_>>()
        .join(", ")
}

const fn binary_operator(op: &BinOp) -> Option<&'static str> {
    Some(match op {
        BinOp::Add(_) => "+",
        BinOp::Sub(_) => "-",
        BinOp::Mul(_) => "*",
        BinOp::Div(_) => "/",
        BinOp::Rem(_) => "%",
        BinOp::And(_) => "&&",
        BinOp::Or(_) => "||",
        BinOp::BitXor(_) => "^",
        BinOp::BitAnd(_) => "&",
        BinOp::BitOr(_) => "|",
        BinOp::Shl(_) => "<<",
        BinOp::Shr(_) => ">>",
        BinOp::Eq(_) => "==",
        BinOp::Lt(_) => "<",
        BinOp::Le(_) => "<=",
        BinOp::Ne(_) => "!=",
        BinOp::Ge(_) => ">=",
        BinOp::Gt(_) => ">",
        _ => return None,
    })
}

fn comments(out: &mut String, lines: &[String], indent: &str) {
    for line in lines {
        if line.is_empty() {
            push_line(out, &format!("{indent}//"));
        } else {
            push_line(out, &format!("{indent}// {line}"));
        }
    }
}

fn push_line(out: &mut String, line: &str) {
    out.push_str(line);
    out.push('\n');
}
