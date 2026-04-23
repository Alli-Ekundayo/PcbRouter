//! KiCad PCB (.kicad_pcb) file parser using kiutils-rs.
//!
//! Parses the KiCad S-expression format into structures sufficient
//! to drive the grid-based router:
//!   - Board layers (copper layers only)
//!   - Netclasses
//!   - Nets
//!   - Modules / footprints (instances)
//!   - Pads (via / SMD / through-hole)

pub mod types;

use std::collections::HashMap;
use types::*;
use kiutils_rs::{PcbDocument, PcbFile, PcbSegment, PcbVia, WriteMode};
use std::fs;
use std::io::{self, Read};

/// Central database built from a parsed `.kicad_pcb` file.
#[derive(Debug, Default)]
pub struct KicadPcbDatabase {
    pub layers: Vec<Layer>,
    /// Map from KiCad layer number to layer name.
    pub layer_id_to_name: HashMap<i32, String>,
    pub netclasses: Vec<Netclass>,
    pub nets: Vec<Net>,
    pub instances: Vec<Instance>,
    /// All pads from all instances, including their absolute position.
    pub pads: Vec<Pad>,
    /// Original document for native write support.
    pub doc: Option<PcbDocument>,
    new_segments: Vec<PcbSegment>,
    new_vias: Vec<PcbVia>,
}

impl KicadPcbDatabase {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn from_file<P: AsRef<std::path::Path>>(path: P) -> io::Result<Self> {
        let content = fs::read_to_string(path)?;
        Self::from_string(&content)
    }

    /// Parse a `.kicad_pcb` file from a reader.
    pub fn from_reader<R: Read>(mut reader: R) -> io::Result<Self> {
        let mut content = String::new();
        reader.read_to_string(&mut content)?;
        Self::from_string(&content)
    }

    /// Internal helper to parse from a string with legacy support.
    fn from_string(content: &str) -> io::Result<Self> {
        // KiCad 5 uses '(module ...)', while KiCad 6+ (and kiutils-rs) use '(footprint ...)'
        // We do a simple replacement to bridge the gap.
        let converted = content.replace("(module ", "(footprint ");
        
        // Since kiutils-rs 0.2.0 PcbFile only reads from path, we use a temporary file.
        let temp_path = std::env::temp_dir().join(format!("pcb_router_temp_{}.kicad_pcb", 
            std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap().as_nanos()));
        
        fs::write(&temp_path, converted)?;
        
        let doc_result = PcbFile::read(&temp_path);
        
        // Clean up temp file immediately
        let _ = fs::remove_file(&temp_path);
        
        let doc = doc_result.map_err(|e| io::Error::new(io::ErrorKind::Other, e.to_string()))?;
        
        let mut db = KicadPcbDatabase::default();
        db.load_from_ast(doc.ast());
        db.doc = Some(doc);
        Ok(db)
    }

    fn load_from_ast(&mut self, ast: &kiutils_rs::PcbAst) {
        // Map layers
        for (idx, layer) in ast.layers.iter().enumerate() {
            let id = idx as i32;
            let name = layer.name.clone().unwrap_or_else(|| format!("Layer_{}", id));
            let layer_type = match layer.layer_type.as_deref() {
                Some("signal") | Some("power") => LayerType::Copper,
                _ if name.starts_with('F') || name.starts_with('B') || name == "In1.Cu"
                    || name.contains(".Cu") => LayerType::Copper,
                _ => LayerType::NonCopper,
            };
            self.layer_id_to_name.insert(id, name.clone());
            self.layers.push(Layer { id, name, layer_type });
        }

        // Map netclasses (Temporarily disabled - field missing in kiutils-rs 0.2.0)
        /*
        if let Some(setup) = &ast.setup {
            for nc_in in &setup.net_classes {
                let nc = Netclass {
                    name: nc_in.name.clone(),
                    description: nc_in.description.clone().unwrap_or_default(),
                    clearance: nc_in.clearance.unwrap_or(0.0),
                    trace_width: nc_in.trace_width.unwrap_or(0.0),
                    via_dia: nc_in.via_dia.unwrap_or(0.0),
                    via_drill: nc_in.via_drill.unwrap_or(0.0),
                    uvia_dia: nc_in.uvia_dia.unwrap_or(0.0),
                    uvia_drill: nc_in.uvia_drill.unwrap_or(0.0),
                };
                self.netclasses.push(nc);
            }
        }
        */

        // Map nets
        for net in &ast.nets {
            self.nets.push(Net {
                id: net.code.unwrap_or(0),
                name: net.name.clone().unwrap_or_default(),
                netclass_name: String::new(),
                pins: Vec::new(),
            });
        }

        // Map footprints and pads
        for fp in &ast.footprints {
            let ref_name = fp.reference.clone().unwrap_or_default();
            let pos = fp.at.unwrap_or([0.0, 0.0]);
            let rot = fp.rotation.unwrap_or(0.0);
            let layer = fp.layer.clone().unwrap_or_default();

            self.instances.push(Instance {
                reference: ref_name.clone(),
                position: (pos[0], pos[1]),
                rotation: rot,
                layer,
            });

            for pad in &fp.pads {
                let pad_pos = pad.at.unwrap_or([0.0, 0.0]);
                // Simplified absolute position (ignoring rotation for legacy parity)
                let abs_pad_pos = (pos[0] + pad_pos[0], pos[1] + pad_pos[1]);

                self.pads.push(Pad {
                    reference: ref_name.clone(),
                    number: pad.number.clone().unwrap_or_default(),
                    pad_type: pad.pad_type.clone().unwrap_or_default(),
                    shape: pad.shape.clone().unwrap_or_default(),
                    position: abs_pad_pos,
                    size: {
                        let s = pad.size.unwrap_or([0.0, 0.0]);
                        (s[0], s[1])
                    },
                    net_id: pad.net.as_ref().and_then(|n| n.code).unwrap_or(-1),
                    layers: pad.layers.clone(),
                    instance_position: (pos[0], pos[1]),
                });
            }
        }
    }

    pub fn add_routing_results(&mut self, segments: Vec<PcbSegment>, vias: Vec<PcbVia>) {
        // We store them locally instead of modifying the doc directly to avoid serialization errors
        self.new_segments = segments;
        self.new_vias = vias;
    }

    pub fn write_to_file<P: AsRef<std::path::Path>>(&self, path: P) -> io::Result<()> {
        if let Some(doc) = &self.doc {
            // 1. Write the base document to a temporary file
            let temp_path = std::env::temp_dir().join(format!("pcb_router_base_{}.kicad_pcb", 
                std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap().as_nanos()));
            
            doc.write_mode(&temp_path, WriteMode::Canonical)
                .map_err(|e| io::Error::new(io::ErrorKind::Other, e.to_string()))?;
            
            // 2. Read it back as a string
            let mut content = fs::read_to_string(&temp_path)?;
            let _ = fs::remove_file(&temp_path);
            
            // 3. Find the last ')' and insert our results before it
            if let Some(last_pos) = content.rfind(')') {
                let mut results = String::new();
                for s in &self.new_segments {
                    results.push_str(&format!("  (segment (start {} {}) (end {} {}) (width {}) (layer \"{}\") (net {}))\n",
                        s.start.unwrap_or([0.0, 0.0])[0], s.start.unwrap_or([0.0, 0.0])[1],
                        s.end.unwrap_or([0.0, 0.0])[0], s.end.unwrap_or([0.0, 0.0])[1],
                        s.width.unwrap_or(0.2),
                        s.layer.as_ref().unwrap_or(&"F.Cu".to_string()),
                        s.net.unwrap_or(0)));
                }
                for v in &self.new_vias {
                    results.push_str(&format!("  (via (at {} {}) (size {}) (drill {}) (layers \"{}\" \"{}\") (net {}))\n",
                        v.at.unwrap_or([0.0, 0.0])[0], v.at.unwrap_or([0.0, 0.0])[1],
                        v.size.unwrap_or(0.8),
                        v.drill.unwrap_or(0.4),
                        v.layers.get(0).cloned().unwrap_or_else(|| "F.Cu".to_string()),
                        v.layers.get(1).cloned().unwrap_or_else(|| "B.Cu".to_string()),
                        v.net.unwrap_or(0)));
                }
                
                content.insert_str(last_pos, &results);
            }
            
            // 4. Convert back to legacy 'module' tokens if needed for KiCad 5 compatibility
            let final_content = content.replace("(footprint", "(module");
            
            // 5. Write final content to destination
            fs::write(path, final_content)?;
        } else {
            return Err(io::Error::new(io::ErrorKind::Other, "No original document loaded"));
        }
        Ok(())
    }

    pub fn get_copper_layers(&self) -> Vec<&Layer> {
        self.layers.iter().filter(|l| l.layer_type == LayerType::Copper).collect()
    }

    pub fn get_num_copper_layers(&self) -> usize {
        self.get_copper_layers().len()
    }

    pub fn print_design_statistics(&self) {
        println!("Layers: {}", self.layers.len());
        println!("Copper layers: {}", self.get_num_copper_layers());
        println!("Nets: {}", self.nets.len());
        println!("Netclasses: {}", self.netclasses.len());
        println!("Instances: {}", self.instances.len());
        println!("Pads: {}", self.pads.len());
    }
}
