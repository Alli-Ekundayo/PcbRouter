use std::env;

use pcb_router_lib::global_param::GlobalParam;
use pcb_router_lib::grid_based_router::GridBasedRouter;
use pcb_router_lib::kicad_parser::KicadPcbDatabase;
use pcb_router_lib::location::Location;

fn main() {
    let args: Vec<String> = env::args().collect();

    if args.len() < 2 {
        eprintln!("Usage: pcb_router <input.kicad_pcb> [output.kicad_pcb] [grid_scale] [num_iterations] [enlarge_boundary] [layer_change_weight] [track_obstacle_weight]");
        std::process::exit(1);
    }

    let input_name = &args[1];
    let output_name = args.get(2).cloned().unwrap_or_else(|| {
        let path = std::path::Path::new(input_name);
        let stem = path.file_stem().and_then(|s| s.to_str()).unwrap_or("output");
        let ext = path.extension().and_then(|s| s.to_str()).unwrap_or("kicad_pcb");
        format!("{}_routed.{}", stem, ext)
    });

    let mut arg_offset = 0;
    // Check if the second arg looks like a number (grid_scale) or a filename
    if args.len() > 2 && args[2].parse::<f64>().is_ok() {
        // Second arg is a number, so output_name should be default
        arg_offset = 1;
    }

    let output_name = if arg_offset == 1 {
        let path = std::path::Path::new(input_name);
        let stem = path.file_stem().and_then(|s| s.to_str()).unwrap_or("output");
        let ext = path.extension().and_then(|s| s.to_str()).unwrap_or("kicad_pcb");
        format!("{}_routed.{}", stem, ext)
    } else {
        output_name
    };

    println!("Parsing design: {}", input_name);

    let mut db = match KicadPcbDatabase::from_file(input_name) {
        Ok(db) => db,
        Err(e) => {
            eprintln!("Failed to parse '{}': {}", input_name, e);
            std::process::exit(1);
        }
    };

    db.print_design_statistics();

    let mut params = GlobalParam::default();

    let mut idx = 2 + (1 - arg_offset);
    if let Some(s) = args.get(idx).and_then(|a| a.parse().ok()) {
        params.set_grid_scale(s);
        idx += 1;
    }
    if let Some(n) = args.get(idx).and_then(|a| a.parse::<u32>().ok()) {
        params.num_rip_up_reroute_iteration = n;
        idx += 1;
    }
    if let Some(b) = args.get(idx).and_then(|a| a.parse::<u32>().ok()) {
        params.enlarge_boundary = b;
        idx += 1;
    }
    if let Some(w) = args.get(idx).and_then(|a| a.parse::<f64>().ok()) {
        params.layer_change_cost = w.abs();
        idx += 1;
    }
    if let Some(w) = args.get(idx).and_then(|a| a.parse::<f64>().ok()) {
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

    println!("Collecting results...");
    
    use kiutils_rs::{PcbSegment, PcbVia};
    let mut new_segments = Vec::new();
    let mut new_vias = Vec::new();

    let grid_layer_to_name = &router.grid_layer_to_name;

    for net_solution in &router.best_solution {
        let net_id = net_solution.net_id;
        
        let nc = if net_solution.grid_netclass_id >= 0 {
             router.board_grid.grid_netclasses.get(net_solution.grid_netclass_id as usize)
        } else {
             None
        };
        
        let trace_width = nc.map(|n| n.trace_width as f64 / router.params.input_scale as f64).unwrap_or(0.25);
        let via_dia = nc.map(|n| n.via_dia as f64 / router.params.input_scale as f64).unwrap_or(0.8);
        let via_drill = nc.map(|n| n.via_drill as f64 / router.params.input_scale as f64).unwrap_or(0.4);

        for path in &net_solution.grid_paths {
            let pts: Vec<Location> = path.segments.iter().cloned().collect();
            if pts.is_empty() {
                continue;
            }
            let mut prev_loc = pts[0];
            for &loc in &pts {
                if loc == prev_loc {
                    continue;
                }
                
                let x1 = (prev_loc.x() as f64) / (router.params.input_scale as f64) + router.min_x - (router.params.enlarge_boundary as f64 / 2.0 / router.params.input_scale as f64);
                let y1 = (prev_loc.y() as f64) / (router.params.input_scale as f64) + router.min_y - (router.params.enlarge_boundary as f64 / 2.0 / router.params.input_scale as f64);
                let x2 = (loc.x() as f64) / (router.params.input_scale as f64) + router.min_x - (router.params.enlarge_boundary as f64 / 2.0 / router.params.input_scale as f64);
                let y2 = (loc.y() as f64) / (router.params.input_scale as f64) + router.min_y - (router.params.enlarge_boundary as f64 / 2.0 / router.params.input_scale as f64);

                if loc.z() != prev_loc.z() {
                    // Via
                    new_vias.push(PcbVia {
                        at: Some([x1, y1]),
                        size: Some(via_dia),
                        drill: Some(via_drill),
                        net: Some(net_id),
                        layers: vec!["F.Cu".to_string(), "B.Cu".to_string()],
                        locked: false,
                        uuid: None,
                        drill_shape: None,
                        drill_x: None,
                        drill_y: None,
                        via_type: None,
                    });
                } else {
                    // Segment
                    let layer_name = grid_layer_to_name.get(loc.z() as usize).map(|s| s.as_str()).unwrap_or("Unknown");
                    new_segments.push(PcbSegment {
                        start: Some([x1, y1]),
                        end: Some([x2, y2]),
                        width: Some(trace_width),
                        layer: Some(layer_name.to_string()),
                        net: Some(net_id),
                        uuid: None,
                        locked: false,
                    });
                }
                prev_loc = loc;
            }
        }
    }

    println!("Adding {} segments and {} vias to PCB...", new_segments.len(), new_vias.len());
    db.add_routing_results(new_segments, new_vias);

    println!("Saving routed PCB to: {}", output_name);
    if let Err(e) = db.write_to_file(&output_name) {
        eprintln!("Failed to write routed PCB: {}", e);
        std::process::exit(1);
    }

    println!("Done.");
}
