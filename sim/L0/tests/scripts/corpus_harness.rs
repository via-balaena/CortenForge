//! Load and run MJCF documents; print one fingerprint line per document.
//!
//! ```text
//! cargo build --release -p sim-conformance-tests --example corpus_harness
//! printf 'str\t<doc.xml>\nfile\t<model.xml>\n' | target/release/examples/corpus_harness
//! ```
//!
//! Each stdin line is `str<TAB>path` (read the file, `load_model`) or
//! `file<TAB>path` (`load_model_from_file`, which resolves includes and
//! assets). Each stdout line is a JSON object:
//!
//! `{"path", "mode", "parse"?, "parse_err"?, "status": "ok"|"err"|"panic", "err"?,
//!   "model_fp"?, "traj": "ok"|"err"|"panic", "traj_fp"?, "traj_err"?, "ms"}`
//!
//! `model_fp` hashes the model's `{:#?}` text with every map and set block
//! sorted, so it changes with the model, not with hash order; `traj_fp` hashes
//! the bits of qpos, qvel, act and time after 100 steps from a fixed
//! excitation. Comparing runs in separate processes finds documents whose model
//! or trajectory depends on hash order (`corpus_nondeterminism.py`); comparing
//! a commit with its parent lists what the commit changed. The `{:#?}` text is
//! hashed as it streams, so memory stays bounded by its largest map block —
//! the whole text of a large model runs to gigabytes.

use std::collections::hash_map::DefaultHasher;
use std::hash::{Hash, Hasher};
use std::io::{BufRead, Write};
use std::panic::{AssertUnwindSafe, catch_unwind};
use std::time::Instant;

use serde_json::{Map, Value, json};

const STEPS: usize = 100;

fn indent(line: &str) -> usize {
    line.len() - line.trim_start().len()
}

fn opens_map_block(line: &str) -> bool {
    let t = line.trim();
    t == "{" || t.ends_with(": {")
}

/// Sort the entries of every map or set block (a `{` with no type name before
/// it) in a `{:#?}` text, recursively; an entry starts one indent level in.
fn canonical(lines: &[&str]) -> Vec<String> {
    let mut out = Vec::new();
    let mut i = 0;
    while i < lines.len() {
        let line = lines[i];
        out.push(line.to_string());
        i += 1;
        if !opens_map_block(line) {
            continue;
        }
        let ind = indent(line);
        let start = i;
        while i < lines.len()
            && !(indent(lines[i]) == ind && lines[i].trim_start().starts_with('}'))
        {
            i += 1;
        }
        let body = &lines[start..i];
        let mut entries: Vec<Vec<String>> = Vec::new();
        let mut k = 0;
        while k < body.len() {
            let first = k;
            k += 1;
            while k < body.len() && indent(body[k]) > ind + 4 {
                k += 1;
            }
            // The entry's closing line (`),` `},` `],`) sits at its own indent.
            while k < body.len()
                && indent(body[k]) == ind + 4
                && matches!(body[k].trim().chars().next(), Some(')' | '}' | ']'))
            {
                k += 1;
            }
            entries.push(canonical(&body[first..k]));
        }
        entries.sort();
        out.extend(entries.into_iter().flatten());
        if let Some(close) = lines.get(i) {
            out.push((*close).to_string());
            i += 1;
        }
    }
    out
}

/// A fingerprint of a `{:#?}` text, fed as it is formatted: lines outside a
/// map block are hashed as they arrive, and a map block is buffered until its
/// closing line, then canonicalised and hashed.
#[derive(Default)]
struct StreamFingerprint {
    partial: String,
    block: Vec<String>,
    block_indent: Option<usize>,
    hasher: DefaultHasher,
    lines: u64,
}

impl StreamFingerprint {
    fn line(&mut self, line: String) {
        if let Some(ind) = self.block_indent {
            let closes = indent(&line) == ind && line.trim_start().starts_with('}');
            self.block.push(line);
            if closes {
                let block = std::mem::take(&mut self.block);
                let refs: Vec<&str> = block.iter().map(String::as_str).collect();
                for l in canonical(&refs) {
                    l.hash(&mut self.hasher);
                    self.lines += 1;
                }
                self.block_indent = None;
            }
        } else if opens_map_block(&line) {
            self.block_indent = Some(indent(&line));
            self.block.push(line);
        } else {
            line.hash(&mut self.hasher);
            self.lines += 1;
        }
    }

    fn finish(mut self) -> String {
        if !self.partial.is_empty() {
            let rest = std::mem::take(&mut self.partial);
            self.line(rest);
        }
        for l in std::mem::take(&mut self.block) {
            l.hash(&mut self.hasher);
        }
        self.lines.hash(&mut self.hasher);
        format!("{:016x}", self.hasher.finish())
    }
}

impl std::fmt::Write for StreamFingerprint {
    fn write_str(&mut self, s: &str) -> std::fmt::Result {
        let mut rest = s;
        while let Some(i) = rest.find('\n') {
            self.partial.push_str(&rest[..i]);
            let line = std::mem::take(&mut self.partial);
            self.line(line);
            rest = &rest[i + 1..];
        }
        self.partial.push_str(rest);
        Ok(())
    }
}

fn panic_text(payload: &(dyn std::any::Any + Send)) -> String {
    payload
        .downcast_ref::<&str>()
        .map(|s| (*s).to_string())
        .or_else(|| payload.downcast_ref::<String>().cloned())
        .unwrap_or_else(|| "<non-string panic>".to_string())
}

/// The bits of the state after `STEPS` steps from qvel[i] = 0.1 (1 + i mod 3)
/// and every ctrl at 0.25, hashed.
fn trajectory_fingerprint(model: &sim_core::Model) -> Result<String, String> {
    let mut data = model
        .try_make_data()
        .map_err(|e| format!("make_data: {e}"))?;
    for (i, v) in data.qvel.iter_mut().enumerate() {
        *v = 0.1 * (1.0 + (i % 3) as f64);
    }
    data.ctrl.fill(0.25);
    for _ in 0..STEPS {
        data.step(model).map_err(|e| format!("{e:?}"))?;
    }
    let bits: Vec<u64> = data
        .qpos
        .iter()
        .chain(data.qvel.iter())
        .chain(data.act.iter())
        .chain(std::iter::once(&data.time))
        .map(|x| x.to_bits())
        .collect();
    let mut hasher = DefaultHasher::new();
    bits.hash(&mut hasher);
    Ok(format!("{:016x}", hasher.finish()))
}

fn fingerprint(mode: &str, path: &str) -> Value {
    let started = Instant::now();
    let mut rec = Map::new();
    rec.insert("path".into(), json!(path));
    rec.insert("mode".into(), json!(mode));
    let text = if mode == "file" {
        None
    } else {
        Some(std::fs::read_to_string(path).unwrap_or_default())
    };
    // The parser alone, which is what parse-only tests see.
    if let Some(xml) = &text {
        match catch_unwind(AssertUnwindSafe(|| sim_mjcf::parse_mjcf_str(xml))) {
            Err(p) => rec.extend([
                ("parse".into(), json!("panic")),
                ("parse_err".into(), json!(panic_text(p.as_ref()))),
            ]),
            Ok(Err(e)) => rec.extend([
                ("parse".into(), json!("err")),
                ("parse_err".into(), json!(e.to_string())),
            ]),
            Ok(Ok(_)) => {
                rec.insert("parse".into(), json!("ok"));
            }
        }
    }
    let loaded = catch_unwind(AssertUnwindSafe(|| match &text {
        None => sim_mjcf::load_model_from_file(path),
        Some(xml) => sim_mjcf::load_model(xml),
    }));
    match loaded {
        Err(p) => rec.extend([
            ("status".into(), json!("panic")),
            ("err".into(), json!(panic_text(p.as_ref()))),
        ]),
        Ok(Err(e)) => rec.extend([
            ("status".into(), json!("err")),
            ("err".into(), json!(e.to_string())),
        ]),
        Ok(Ok(model)) => {
            let mut sink = StreamFingerprint::default();
            let formatted = std::fmt::Write::write_fmt(&mut sink, format_args!("{model:#?}"));
            rec.insert("status".into(), json!("ok"));
            if formatted.is_ok() {
                rec.insert("model_fp".into(), json!(sink.finish()));
            }
            match catch_unwind(AssertUnwindSafe(|| trajectory_fingerprint(&model))) {
                Err(p) => rec.extend([
                    ("traj".into(), json!("panic")),
                    ("traj_err".into(), json!(panic_text(p.as_ref()))),
                ]),
                Ok(Err(e)) => {
                    rec.extend([("traj".into(), json!("err")), ("traj_err".into(), json!(e))])
                }
                Ok(Ok(fp)) => {
                    rec.extend([("traj".into(), json!("ok")), ("traj_fp".into(), json!(fp))])
                }
            }
        }
    }
    rec.insert("ms".into(), json!(started.elapsed().as_millis()));
    Value::Object(rec)
}

fn main() {
    // A caught panic is a recorded result; its default report would flood stderr.
    std::panic::set_hook(Box::new(|_| {}));
    let mut out = std::io::stdout().lock();
    for line in std::io::stdin().lock().lines() {
        let line = line.unwrap_or_else(|e| panic!("read stdin: {e}"));
        let Some((mode, path)) = line.split_once('\t') else {
            continue;
        };
        writeln!(out, "{}", fingerprint(mode, path))
            .unwrap_or_else(|e| panic!("write stdout: {e}"));
        out.flush().unwrap_or_else(|e| panic!("flush stdout: {e}"));
    }
}
