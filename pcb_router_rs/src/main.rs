use std::env;
use std::fs::File;

use pcb_router_lib::global_param::GlobalParam;
use pcb_router_lib::grid_based_router::GridBasedRouter;
use pcb_router_lib::kicad_parser::KicadPcbDatabase;
use pcb_router_lib::location::Location;

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

    println!("--- ROUTING DATA START ---");
    for net in &router.best_solution {
        for path in &net.grid_paths {
            let pts: Vec<Location> = path.segments.iter().cloned().collect();
            if pts.is_empty() {
                continue;
            }
            let mut prev_loc = pts[0];
            for &loc in &pts {
                if loc == prev_loc {
                    continue;
                }
                if loc.z() != prev_loc.z() {
                    // Via
                    let x = (loc.x() as f64) / (router.params.input_scale as f64) + router.min_x - (router.params.enlarge_boundary as f64 / 2.0 / router.params.input_scale as f64);
                    let y = (loc.y() as f64) / (router.params.input_scale as f64) + router.min_y - (router.params.enlarge_boundary as f64 / 2.0 / router.params.input_scale as f64);
                    
                    // Try to get netclass info
                    let (via_dia, via_drill) = if net.grid_netclass_id >= 0 {
                        if let Some(nc) = router.board_grid.grid_netclasses.get(net.grid_netclass_id as usize) {
                            (nc.via_dia as f64 / router.params.input_scale as f64, nc.via_drill as f64 / router.params.input_scale as f64)
                        } else {
                            (0.8, 0.4)
                        }
                    } else {
                        (0.8, 0.4)
                    };
                    println!("VIA {} {} {} {} {} {}", net.net_id, x, y, loc.z(), via_dia, via_drill);
                } else {
                    // Segment
                    let x1 = (prev_loc.x() as f64) / (router.params.input_scale as f64) + router.min_x - (router.params.enlarge_boundary as f64 / 2.0 / router.params.input_scale as f64);
                    let y1 = (prev_loc.y() as f64) / (router.params.input_scale as f64) + router.min_y - (router.params.enlarge_boundary as f64 / 2.0 / router.params.input_scale as f64);
                    let x2 = (loc.x() as f64) / (router.params.input_scale as f64) + router.min_x - (router.params.enlarge_boundary as f64 / 2.0 / router.params.input_scale as f64);
                    let y2 = (loc.y() as f64) / (router.params.input_scale as f64) + router.min_y - (router.params.enlarge_boundary as f64 / 2.0 / router.params.input_scale as f64);
                    let layer_name = router.grid_layer_to_name.get(loc.z() as usize).map(|s| s.as_str()).unwrap_or("Unknown");
                    
                    let width = if net.grid_netclass_id >= 0 {
                        if let Some(nc) = router.board_grid.grid_netclasses.get(net.grid_netclass_id as usize) {
                            nc.trace_width as f64 / router.params.input_scale as f64
                        } else {
                            0.25
                        }
                    } else {
                        0.25
                    };
                    println!("SEGMENT {} {} {} {} {} {} {}", net.net_id, layer_name, x1, y1, x2, y2, width);
                }
                prev_loc = loc;
            }
        }
    }
    println!("--- ROUTING DATA END ---");
}
