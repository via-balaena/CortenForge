#!/usr/bin/env python3
"""Extract the MJCF documents in the repository's Rust string literals and Markdown fences.

    extract_mjcf.py extract <out_dir>   every doc to <out_dir>/docs/<id>.xml, and one record per
                                        `<mujoco` occurrence (loadable or not) to <out_dir>/manifest.jsonl
    extract_mjcf.py drift [--append]    compare the working tree's docs with the parity-census snapshot

A doc is one `<mujoco ...>...</mujoco>` span of a string literal (or of a fenced
Markdown block) that holds a complete document; its id is sha256(text)[:16]. A
`format!` template is recorded but not extracted: its text is not a document
until it runs. In `///` and `//!` doc comments, a doc in a string literal of a
fenced Rust code block (a doctest) is extracted as rustdoc compiles it, and a
doc in an `xml` or other fenced block like a Markdown fence; a `<mujoco` in
prose is recorded but not extracted. The rules for which fence is a doctest,
hidden `#` lines, fences and doc runs follow rustdoc 1.96.0 and CommonMark;
test_extract_mjcf.py checks them against rustdoc's results on its cases. A `<mujoco` in
a doc comment rustdoc may compile but this does not model is recorded as
`doc-unmodelled`: a block doc comment (`/** */`, `/*! */`), a `doc` attribute,
an indented code block, a block quote, or a doctest that does not lex.
Read-only on the repository, except `drift --append`.

`drift` exits 1 when the working tree holds a doc the snapshot lacks, or a
`doc-unmodelled` record (rewrite it as a fenced block in `///` or `//!`). With `--append` it adds
those docs to sim/L0/tests/assets/census/docs/, appends their rows to its
manifest.tsv, and prints their ids — the ids-file gen_census_golden.py takes:

    extract_mjcf.py drift --append > new_ids.txt
    <oracle>/venv/bin/python -I gen_census_golden.py <docs> <golden> new_ids.txt

A doc the snapshot holds and the working tree no longer does stays: the snapshot only grows.
"""
import collections
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile

FORMAT_MACROS = {
    "format", "write", "writeln", "print", "println", "eprint", "eprintln",
    "format_args", "panic", "assert", "assert_eq", "assert_ne",
}


def lex(src):
    """Yield tokens: ('str', start, end, value, kind), ('comment', start, end, text, doc),
    ('brace', pos, ch), ('other', pos, ch)."""
    i, n = 0, len(src)
    out = []
    while i < n:
        c = src[i]
        if c == "/" and src.startswith("//", i):
            j = src.find("\n", i)
            j = n if j < 0 else j
            text = src[i:j]
            doc = (text.startswith("///") and not text.startswith("////")) or text.startswith("//!")
            out.append(("comment", i, j, text, doc))
            i = j
            continue
        if c == "/" and src.startswith("/*", i):
            depth, j = 1, i + 2
            while j < n and depth:
                if src.startswith("/*", j):
                    depth += 1
                    j += 2
                elif src.startswith("*/", j):
                    depth -= 1
                    j += 2
                else:
                    j += 1
            text = src[i:j]
            doc = (text.startswith("/**") and not text.startswith("/***") and text != "/**/") or text.startswith("/*!")
            out.append(("comment", i, j, text, doc))
            i = j
            continue
        # raw strings / byte / c strings
        m = re.compile(r'(br|cr|r)(#*)"').match(src, i)
        if m and (i == 0 or not (src[i - 1].isalnum() or src[i - 1] == "_")):
            hashes = m.group(2)
            close = '"' + hashes
            j = src.find(close, m.end())
            if j < 0:
                raise ValueError("unterminated raw string at %d" % i)
            out.append(("str", i, j + len(close), src[m.end():j], "raw"))
            i = j + len(close)
            continue
        m = re.compile(r'(b|c)?"').match(src, i)
        if m and (m.group(1) is None or i == 0 or not (src[i - 1].isalnum() or src[i - 1] == "_")):
            j = m.end()
            buf = []
            while j < n and src[j] != '"':
                if src[j] == "\\":
                    nx = src[j + 1]
                    if nx == "\n":
                        j += 2
                        while j < n and src[j] in " \t\r\n":
                            j += 1
                        continue
                    if nx == "u":
                        k = src.find("}", j)
                        buf.append(chr(int(src[j + 3:k], 16)))
                        j = k + 1
                        continue
                    if nx == "x":
                        buf.append(chr(int(src[j + 2:j + 4], 16)))
                        j += 4
                        continue
                    buf.append({"n": "\n", "t": "\t", "r": "\r", "0": "\0", "\\": "\\", '"': '"', "'": "'"}[nx])
                    j += 2
                    continue
                buf.append(src[j])
                j += 1
            out.append(("str", i, j + 1, "".join(buf), "cooked"))
            i = j + 1
            continue
        if c == "'":
            # char literal or lifetime
            if i + 1 < n and src[i + 1] == "\\":
                j = src.find("'", i + 2)
                if src[i + 2] == "'":  # '\''
                    j = i + 3
                out.append(("other", i, "char"))
                i = j + 1
                continue
            if i + 2 < n and src[i + 2] == "'":
                out.append(("other", i, "char"))
                i += 3
                continue
            i += 1
            continue
        if c in "{}()[]":
            out.append(("brace", i, c))
        elif not c.isspace():
            out.append(("other", i, c))
        i += 1
    return out


def prev_code(src, toks, idx):
    """Text of the code tokens immediately before token idx (comments skipped), ~80 chars."""
    k = idx - 1
    while k >= 0 and toks[k][0] == "comment":
        k -= 1
    if k < 0:
        return ""
    end = toks[k][2] if toks[k][0] in ("str",) else toks[k][1] + 1
    start = max(0, end - 120)
    return src[start:end]


def macro_of(prev):
    m = re.search(r"([A-Za-z_][A-Za-z0-9_]*)!\s*\(\s*(?:[A-Za-z_][A-Za-z0-9_.]*\s*,\s*)?$", prev)
    return m.group(1) if m else None


def placeholders(tmpl):
    s = tmpl.replace("{{", "").replace("}}", "")
    return re.findall(r"\{[^{}]*\}", s)


def test_ranges(src, toks):
    """Byte ranges of `#[cfg(test)] mod x { ... }` blocks and `#[test]`-attributed fns."""
    opens = []
    match = {}
    for t in toks:
        if t[0] == "brace":
            if t[2] in "{":
                opens.append(t[1])
            elif t[2] == "}":
                if opens:
                    match[opens.pop()] = t[1]
    ranges = []
    for m in re.finditer(r"#\[cfg\(test\)\]", src):
        k = src.find("{", m.end())
        if k >= 0 and re.match(r"\s*(#\[[^\]]*\]\s*)*(pub(\([^)]*\))?\s+)?mod\s+\w+\s*$", src[m.end():k]) and k in match:
            ranges.append((k, match[k]))
    for m in re.finditer(r"#\[(tokio::)?test\]", src):
        k = src.find("{", m.end())
        if k >= 0 and k in match:
            ranges.append((k, match[k]))
    return ranges


RUSTDOC_TEST_WORDS = {"should_panic", "no_run", "ignore"}
RUSTDOC_TEST_ATTRS = {"test_harness", "compile_fail", "standalone_crate"}


def fence_tokens(info):
    """The words of a fence info string as rustdoc splits them: on commas and whitespace, with
    `{...}` attribute groups (`{.class}`) dropped."""
    return [w for w in re.split(r"[\s,]+", re.sub(r"\{[^}]*\}?", " ", info)) if w]


def is_rust_fence(info):
    """Whether rustdoc compiles a fenced block with this info string as a doctest, modelled on
    rustdoc's `LangString::parse` and checked against rustdoc 1.96.0 on the info strings in
    test_extract_mjcf.py (a non-Rust word glued to an attribute group, `text{.x}`, which rustdoc
    runs as a doctest, is read here as not Rust): untagged is Rust; `rust` keeps it
    Rust; `should_panic`, `no_run`, `ignore` and `ignore-*` keep it Rust only before any other
    word, `compile_fail`, `test_harness` and `standalone_crate` before any other word or after a
    Rust tag; `edition*` changes nothing; `custom`, and any other word on its own, make it not
    Rust (an error code such as `E0308` is such a word on stable rustdoc)."""
    seen_rust = seen_other = custom = False
    for w in fence_tokens(info):
        if w in RUSTDOC_TEST_WORDS or w.startswith("ignore-"):
            seen_rust = not seen_other
        elif w == "rust":
            seen_rust = True
        elif w in RUSTDOC_TEST_ATTRS:
            seen_rust = not seen_other or seen_rust
        elif w == "custom":
            custom = True
        elif not w.startswith("edition"):
            seen_other = True
    return not custom and (not seen_other or seen_rust)


def is_line_doc(tok):
    """Whether a token is a `///` or `//!` line doc comment (`////` is a plain comment)."""
    return tok[0] == "comment" and tok[3][:3] in ("///", "//!") and not tok[3].startswith("////")


def doc_blocks(src, toks):
    """The doc text of each item, from its `///` (or `//!`) line comments, unindented as rustdoc
    does. A blank line or a plain comment between two of them does not end the item's doc; code
    or the other marker does.

    Yields (lines, source_lines): one entry per doc comment line with the marker and the common
    leading whitespace removed, and each entry's 1-based line."""
    runs = [[]]
    for t in toks:
        if is_line_doc(t):
            if runs[-1] and runs[-1][-1][3][:3] != t[3][:3]:
                runs.append([])
            runs[-1].append(t)
        elif t[0] != "comment" and runs[-1]:
            runs.append([])
    for run in filter(None, runs):
        bodies = [t[3][3:] for t in run]
        indent = min((len(b) - len(b.lstrip()) for b in bodies if b.strip()), default=0)
        lines = [b[indent:] if b.strip() else "" for b in bodies]
        yield lines, [src.count("\n", 0, t[1]) + 1 for t in run]


FENCE_OPEN = re.compile(r"( {0,3})(`{3,}|~{3,})(.*)$")


def doc_fences(lines):
    """Fenced blocks of a doc comment's Markdown, by CommonMark's rules: a fence opens with three
    or more backticks or tildes indented at most three spaces, closes with at least as many of the
    same character and nothing else on the line, and its body loses up to the opening fence's
    indentation. Yields (info, first body line index, body lines)."""
    i = 0
    while i < len(lines):
        m = FENCE_OPEN.fullmatch(lines[i])
        if not m or (m.group(2)[0] == "`" and "`" in m.group(3)):
            i += 1
            continue
        indent, fence, info = len(m.group(1)), m.group(2), m.group(3)
        close = re.compile(r" {0,3}%s{%d,}\s*" % (re.escape(fence[0]), len(fence)))
        j = i + 1
        while j < len(lines) and not close.fullmatch(lines[j]):
            j += 1
        body = [line[min(indent, len(line) - len(line.lstrip(" "))):] for line in lines[i + 1:j]]
        yield info, i + 1, body
        i = j + 1


def doctest_code(body):
    """A Rust doctest's code as rustdoc compiles it (its `map_line`): a line whose trimmed text
    starts `# ` or is `#` alone is hidden and contributes the rest of its trimmed text; a line
    starting `##` keeps one `#`."""
    out = []
    for line in body:
        t = line.strip()
        if t.startswith("##"):
            out.append(line.replace("##", "#", 1))
        elif t.startswith("# "):
            out.append(t[2:])
        elif t == "#":
            out.append("")
        else:
            out.append(line)
    return "\n".join(out)


def literal_records(src, toks, tranges, line_of):
    """One record per string literal holding `<mujoco`: its kind, macro and text."""
    recs = []
    for idx, t in enumerate(toks):
        if t[0] != "str" or "<mujoco" not in t[3]:
            continue
        mac = macro_of(prev_code(src, toks, idx))
        text = t[3]
        rec = dict(line=line_of(t[1]), lit=t[4], macro=mac, in_test_block=any(a <= t[1] <= b for a, b in tranges))
        if mac == "concat":
            rec["kind"] = "concat"
            parts = [text]
            for tk in toks[idx + 1:]:
                if tk[0] == "str":
                    parts.append(tk[3])
                elif not (tk[0] == "comment" or (tk[0] == "other" and tk[2] == ",")):
                    break
            text = "".join(parts)
        elif mac in FORMAT_MACROS:
            ph = placeholders(text)
            rec["placeholders"] = len(ph)
            if ph:
                rec["kind"] = "format-template"
                rec["placeholder_names"] = sorted(set(ph))[:8]
            else:
                rec["kind"] = "format-noarg"
                text = text.replace("{{", "{").replace("}}", "}")
        else:
            rec["kind"] = "literal"
        rec["text"] = text
        recs.append(rec)
    return recs


def doc_comment_records(src, toks):
    """Records for the `<mujoco` occurrences in `///` and `//!` doc comments (see the module
    docstring). A `<mujoco` on a line rustdoc may compile that this does not model (an indented
    code block, a block quote, or a Rust fence that does not lex) is `doc-unmodelled`."""
    recs = []
    for lines, src_lines in doc_blocks(src, toks):
        if not any("<mujoco" in line for line in lines):
            continue
        fenced = set()
        for info, start, body in doc_fences(lines):
            fenced.update(range(start - 1, start + len(body) + 1))
            if not any("<mujoco" in line for line in body):
                continue
            if is_rust_fence(info):
                code = doctest_code(body)
                try:
                    ctoks = lex(code)
                except ValueError as e:
                    recs.append(dict(kind="doc-unmodelled", line=src_lines[start], text=None,
                                     reason=f"doctest does not lex: {e}"))
                    continue

                def line_of(p, start=start, code=code):
                    return src_lines[start + code.count("\n", 0, p)]
                for t in ctoks:
                    if t[0] == "comment" and "<mujoco" in t[3]:
                        recs.append(dict(kind="doc-comment", line=line_of(t[1]), text=None))
                for rec in literal_records(code, ctoks, [], line_of):
                    rec["doctest"] = True
                    rec["kind"] = "doc-" + rec["kind"]
                    recs.append(rec)
            else:
                recs.append(dict(kind="doc-md-block", lang=info.strip(), line=src_lines[start],
                                 text="\n".join(body) + "\n"))
        for i, line in enumerate(lines):
            if "<mujoco" in line and i not in fenced:
                if line.startswith("    ") or line.lstrip().startswith(">"):
                    recs.append(dict(kind="doc-unmodelled", line=src_lines[i], text=None,
                                     reason="indented code block or block quote"))
                else:
                    recs.append(dict(kind="doc-comment", line=src_lines[i], text=None))
    return recs


DOC_ATTR = re.compile(r"(#!?\[\s*|cfg_attr\([^()]*,\s*)doc\s*=\s*$")


def rs_records(src):
    """Every record of a Rust source file, each with its `text` (None when not extracted)."""
    toks = lex(src)
    recs = []
    for t in toks:
        if t[0] == "comment" and "<mujoco" in t[3] and not is_line_doc(t):
            line = src.count("\n", 0, t[1]) + 1
            if t[4]:
                recs.append(dict(kind="doc-unmodelled", line=line, text=None, reason="block doc comment"))
            else:
                recs.append(dict(kind="comment", line=line, text=None))
    recs += doc_comment_records(src, toks)
    for rec, t in zip(literal_records(src, toks, test_ranges(src, toks), lambda p: src.count("\n", 0, p) + 1),
                      [t for t in toks if t[0] == "str" and "<mujoco" in t[3]]):
        if DOC_ATTR.search(src[max(0, t[1] - 200):t[1]]):
            rec = dict(kind="doc-unmodelled", line=rec["line"], text=None, reason="doc attribute")
        recs.append(rec)
    return recs


def doc_spans(text):
    """The documents in a record's text: each `<mujoco ...>...</mujoco>` or `<mujoco .../>` span."""
    return re.findall(r"<mujoco\b.*?</mujoco>|<mujoco\b[^>]*/>", text, re.S)


def extract(root, out_dir):
    """Write every doc under <out_dir>/docs and return the manifest records."""
    meta = json.loads(subprocess.run(["cargo", "metadata", "--no-deps", "--format-version", "1"],
                                     cwd=root, capture_output=True, text=True, check=True).stdout)
    wsroot = meta["workspace_root"]
    crate_dirs = sorted(
        ((p["name"], os.path.relpath(os.path.dirname(p["manifest_path"]), wsroot)) for p in meta["packages"]),
        key=lambda x: -len(x[1]),
    )

    def owner(path):
        for name, d in crate_dirs:
            if d == "." or path.startswith(d + "/"):
                return name
        return None

    files = subprocess.run(["git", "-C", root, "ls-files", "*.rs", "*.md"], capture_output=True, text=True,
                           check=True).stdout.split()
    os.makedirs(os.path.join(out_dir, "docs"), exist_ok=True)
    records = []
    for rel in files:
        with open(os.path.join(root, rel), encoding="utf-8") as f:
            src = f.read()
        if "<mujoco" not in src:
            continue
        recs = []
        if rel.endswith(".md"):
            for m in re.finditer(r"```(\w*)\n(.*?)```", src, re.S):
                if "<mujoco" in m.group(2):
                    line = src.count("\n", 0, m.start()) + 1
                    recs.append(dict(kind="md-block", lang=m.group(1), line=line, text=m.group(2)))
        else:
            recs = rs_records(src)
        for r in recs:
            r["file"] = rel
            r["crate"] = owner(rel)
            text = r.pop("text")
            if text is None or r["kind"] in ("format-template", "doc-format-template"):
                records.append(r)
                continue
            r["complete"] = "</mujoco>" in text or re.search(r"<mujoco[^>]*/>", text) is not None
            if r["kind"] in ("md-block", "doc-md-block") or r["complete"]:
                # one document per <mujoco ...> ... </mujoco> span (a literal can hold several)
                r["doc_ids"] = []
                for span in doc_spans(text):
                    h = hashlib.sha256(span.encode()).hexdigest()[:16]
                    path = os.path.join(out_dir, "docs", h + ".xml")
                    if not os.path.exists(path):
                        with open(path, "w") as f:
                            f.write(span)
                    r["doc_ids"].append(h)
            records.append(r)
    return records


def repo_root():
    return subprocess.run(["git", "rev-parse", "--show-toplevel"], capture_output=True, text=True,
                          check=True).stdout.strip()


def drift(append):
    root = repo_root()
    census = os.path.join(root, "sim", "L0", "tests", "assets", "census")
    snapshot = {f[:-len(".xml")] for f in os.listdir(os.path.join(census, "docs")) if f.endswith(".xml")}
    with tempfile.TemporaryDirectory() as tmp:
        records = extract(root, tmp)
        unmodelled = [r for r in records if r["kind"] == "doc-unmodelled"]
        for r in unmodelled:
            print(f"drift: {r['file']}:{r['line']}: MJCF in a doc comment this does not model "
                  f"({r['reason']})", file=sys.stderr)
        if unmodelled:
            return 1
        head = {f[:-len(".xml")] for f in os.listdir(os.path.join(tmp, "docs"))}
        added = sorted(head - snapshot)
        print(f"drift: {len(head)} docs in the working tree, {len(snapshot)} in the snapshot; "
              f"{len(added)} added, {len(snapshot - head)} no longer in the tree (kept)", file=sys.stderr)
        if not added:
            return 0
        if not append:
            print("drift: new docs; re-run with --append and generate their golden", file=sys.stderr)
            return 1
        rows = sorted({(d, r["file"], r["line"]) for r in records for d in r.get("doc_ids", []) if d in added})
        for doc in added:
            shutil.copy(os.path.join(tmp, "docs", doc + ".xml"), os.path.join(census, "docs", doc + ".xml"))
        with open(os.path.join(census, "manifest.tsv"), "a") as f:
            for doc, file, line in rows:
                f.write(f"{doc}\t{file}:{line}\n")
        print("\n".join(added))
    return 0


def main():
    args = sys.argv[1:]
    if len(args) == 2 and args[0] == "extract":
        records = extract(repo_root(), args[1])
        with open(os.path.join(args[1], "manifest.jsonl"), "w") as f:
            for r in records:
                f.write(json.dumps(r) + "\n")
        print(f"{len(records)} records", file=sys.stderr)
        return 0
    if args in (["drift"], ["drift", "--append"]):
        return drift(append=len(args) == 2)
    print(__doc__, file=sys.stderr)
    return 2


if __name__ == "__main__":
    sys.exit(main())
