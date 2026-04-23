use std::env;
use std::fs::File;

use pcb_router_lib::global_param::GlobalParam;
use pcb_router_lib::grid_based_router::GridBasedRouter;
use pcb_router_lib::kicad_parser::KicadPcbDatabase;

fn main() {
    let args: Vec<String> = env::args().collect();

    if args.len() < 2 {
        eprintln!("Usage: pcb_router <input.kicad_pcb> [grid_scale] [num_iterations] [enlarge_boundary] [layer_change_weight] [track_obstacle_weight]");
        std::process::exit(1);
    }

    let design_name = &args[1];
    println!("Parsing design: {}", design_name);

    let file = match File::open(design_name) {
        Ok(f) => f,
        Err(e) => {
            eprintln!("Failed to open '{}': {}", design_name, e);
            std::process::exit(1);
        }
    };

    let db = match KicadPcbDatabase::from_reader(file) {
        Ok(db) => db,
        Err(e) => {
            eprintln!("Failed to parse '{}': {}", design_name, e);
            std::process::exit(1);
        }
    };

    db.print_design_statistics();

    let mut params = GlobalParam::default();

    if let Some(s) = args.get(2).and_then(|a| a.parse().ok()) {
        params.set_grid_scale(s);
    }
    if let Some(n) = args.get(3).and_then(|a| a.parse::<u32>().ok()) {
        params.num_rip_up_reroute_iteration = n;
    }
    if let Some(b) = args.get(4).and_then(|a| a.parse::<u32>().ok()) {
        params.enlarge_boundary = b;
    }
    if let Some(w) = args.get(5).and_then(|a| a.parse::<f64>().ok()) {
        params.layer_change_cost = w.abs();
    }
    if let Some(w) = args.get(6).and_then(|a| a.parse::<f64>().ok()) {
        params.trace_basic_cost = w;
    }

    println!("Starting router...");
    let mut router = GridBasedRouter::new(params);
    router.initialization(&db);
    router.route_all();

    println!(
        "Routed WL: {:.4}, # vias: {}, # bends: {}",
        router.get_routed_wirelength(),
        router.get_routed_num_vias(),
        router.get_routed_num_bends(),
    );
}
