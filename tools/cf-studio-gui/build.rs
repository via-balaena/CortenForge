//! Compile the Slint markup into Rust at build time.

fn main() {
    if let Err(e) = slint_build::compile("ui/app.slint") {
        eprintln!("failed to compile ui/app.slint: {e}");
        std::process::exit(1);
    }
}
