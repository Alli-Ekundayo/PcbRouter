# PcbRouter

Printed Circuit Board (PCB) router — implemented in Rust.

Supports the `.kicad_pcb` format (KiCad v5.1.2+).

## Prerequisites

- [Rust](https://www.rust-lang.org/tools/install) (edition 2021, any recent stable toolchain)

## Building

```
cd pcb_router_rs
cargo build --release
```

The compiled binary is placed at `pcb_router_rs/target/release/pcb_router`.

## Running

```
pcb_router_rs/target/release/pcb_router <input.kicad_pcb> [grid_scale] [num_iterations] [enlarge_boundary] [layer_change_weight] [track_obstacle_weight]
```

| Argument | Description | Default |
|---|---|---|
| `input.kicad_pcb` | Input KiCad PCB file (required) | — |
| `grid_scale` | Grid resolution scale factor | 1.0 |
| `num_iterations` | Number of rip-up/reroute iterations | (see `GlobalParam`) |
| `enlarge_boundary` | Board boundary enlargement | (see `GlobalParam`) |
| `layer_change_weight` | Cost penalty for layer changes (vias) | (see `GlobalParam`) |
| `track_obstacle_weight` | Cost penalty for traces near obstacles | (see `GlobalParam`) |

After routing, statistics (wirelength, via count, bend count) are printed to stdout.

## Testing

```
cd pcb_router_rs
cargo test
```

## License

- BSD-3-clause License [[Link]](LICENSE)
