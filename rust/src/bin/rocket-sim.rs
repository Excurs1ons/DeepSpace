fn main() {
    let args = deepspace::app::CliArgs::parse();
    if args.headless {
        let mut app = deepspace::app::SimulationApp::new(&args);
        app.run();
    } else {
        eprintln!("Usage: deepspace --headless [--mission <path>] [--csv <path>]");
        std::process::exit(1);
    }
}
