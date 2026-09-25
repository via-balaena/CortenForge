//! The `write` and `check` commands, through the library and the binary.

#![allow(clippy::unwrap_used, clippy::panic)]

use std::process::Command;

use sim_wgsl_gen::{Error, check, regenerate_command, write};

const SOURCE: &str = "/// Twice `a`.\nfn twice(a: R) -> R {\n    a * 2.0\n}\n";

/// A scratch directory holding `source.rs`, and the paths of the source and
/// of the output to generate.
fn scratch() -> (tempfile::TempDir, String, String) {
    let dir = tempfile::tempdir().unwrap();
    let input = dir.path().join("source.rs");
    std::fs::write(&input, SOURCE).unwrap();
    let output = dir.path().join("out.wgsl");
    (
        dir,
        input.to_str().unwrap().to_string(),
        output.to_str().unwrap().to_string(),
    )
}

#[test]
fn write_then_check_passes_and_an_edit_makes_it_stale() {
    let (_dir, input, output) = scratch();
    write(&output, &[&input]).unwrap();
    check(&output, &[&input]).unwrap();

    let edited = std::fs::read_to_string(&output)
        .unwrap()
        .replace("* 2.0", "* 3.0");
    std::fs::write(&output, edited).unwrap();
    match check(&output, &[&input]) {
        Err(Error::Stale {
            difference,
            command,
            ..
        }) => {
            assert!(
                difference.contains("3.0") && difference.contains("2.0"),
                "{difference}"
            );
            assert_eq!(command, regenerate_command(&output, &[&input]));
        }
        other => panic!("expected stale, got {other:?}"),
    }
}

#[test]
fn a_missing_file_is_an_io_error_naming_it() {
    let (_dir, input, output) = scratch();
    let err = check(&output, &[&input]).unwrap_err();
    assert!(
        matches!(&err, Error::Io { path, .. } if *path == output),
        "{err:?}"
    );
    let err = write(&output, &["no/such/source.rs"]).unwrap_err();
    assert!(err.to_string().starts_with("no/such/source.rs: "), "{err}");
    let err = write("no/such/dir/out.wgsl", &[&input]).unwrap_err();
    assert!(matches!(err, Error::Io { .. }), "{err:?}");
}

#[test]
fn the_regenerate_command_runs_from_the_workspace_root() {
    assert_eq!(
        regenerate_command("a/out.wgsl", &["a/x.rs", "a/y.rs"]),
        "cargo run -p sim-wgsl-gen -- write a/out.wgsl a/x.rs a/y.rs"
    );
}

#[test]
fn the_binary_writes_checks_and_rejects_bad_usage() {
    let binary = env!("CARGO_BIN_EXE_wgsl-gen");
    let (_dir, input, output) = scratch();
    let run = |args: &[&str]| Command::new(binary).args(args).output().unwrap();

    assert!(run(&["write", &output, &input]).status.success());
    assert!(run(&["check", &output, &input]).status.success());

    std::fs::write(&output, "stale").unwrap();
    let stale = run(&["check", &output, &input]);
    assert!(!stale.status.success());
    assert!(String::from_utf8_lossy(&stale.stderr).contains("is stale"));

    for bad in [&[][..], &["write"][..], &["erase", &output, &input][..]] {
        let result = run(bad);
        assert!(!result.status.success(), "{bad:?}");
        assert!(
            String::from_utf8_lossy(&result.stderr).contains("usage:"),
            "{bad:?}"
        );
    }
}

#[test]
fn nothing_is_written_when_there_is_nothing_valid_to_write() {
    let binary = env!("CARGO_BIN_EXE_wgsl-gen");
    let (dir, input, output) = scratch();

    // No inputs: an empty module would overwrite a real one.
    let empty = Command::new(binary)
        .args(["write", &output])
        .output()
        .unwrap();
    assert!(!empty.status.success());
    assert!(String::from_utf8_lossy(&empty.stderr).contains("no sources"));
    assert!(!std::path::Path::new(&output).exists());

    // A source inside the subset that naga rejects (`select` over arrays).
    let rejected = dir.path().join("rejected.rs");
    std::fs::write(
        &rejected,
        "fn f(c: bool, a: [R; 2], b: [R; 2]) -> [R; 2] {\n    if c { a } else { b }\n}\n",
    )
    .unwrap();
    let err = write(&output, &[rejected.to_str().unwrap()]).unwrap_err();
    assert!(matches!(err, Error::Invalid { .. }), "{err:?}");
    assert!(!std::path::Path::new(&output).exists());
    assert!(std::path::Path::new(&input).exists());
}
