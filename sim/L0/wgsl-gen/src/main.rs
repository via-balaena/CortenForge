//! `sim-wgsl-gen write|check <output.wgsl> <input.rs>...`
//!
//! Run from the workspace root, so the source paths recorded in the output's
//! header are the ones a freshness test compares against.

use std::process::ExitCode;

const USAGE: &str = "usage: sim-wgsl-gen write|check <output.wgsl> <input.rs>...";

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let Some((command, rest)) = args.split_first() else {
        eprintln!("{USAGE}");
        return ExitCode::FAILURE;
    };
    let Some((output, inputs)) = rest.split_first() else {
        eprintln!("{USAGE}");
        return ExitCode::FAILURE;
    };
    let inputs: Vec<&str> = inputs.iter().map(String::as_str).collect();
    let result = match command.as_str() {
        "write" => sim_wgsl_gen::write(output, &inputs),
        "check" => sim_wgsl_gen::check(output, &inputs),
        _ => {
            eprintln!("{USAGE}");
            return ExitCode::FAILURE;
        }
    };
    match result {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("{e}");
            ExitCode::FAILURE
        }
    }
}
