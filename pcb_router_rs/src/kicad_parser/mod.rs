//! Simplified KiCad PCB (.kicad_pcb) file parser.
//!
//! Parses a subset of the KiCad 5.x S-expression format that is sufficient
//! to drive the grid-based router:
//!   - Board layers (copper layers only)
//!   - Netclasses
//!   - Nets
//!   - Modules / footprints (instances)
//!   - Pads (via / SMD / through-hole)
//!   - Existing segments and vias (pre-routed)

pub mod types;

use std::collections::HashMap;
use std::io::{self, Read};
use types::*;

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
}

impl KicadPcbDatabase {
    pub fn new() -> Self {
        Self::default()
    }

    /// Parse a `.kicad_pcb` file from a reader.
    pub fn from_reader<R: Read>(reader: R) -> io::Result<Self> {
        let mut db = KicadPcbDatabase::new();
        let content = {
            let mut buf_reader = io::BufReader::new(reader);
            let mut s = String::new();
            buf_reader.read_to_string(&mut s)?;
            s
        };
        db.parse(&content);
        Ok(db)
    }

    fn parse(&mut self, content: &str) {
        // Tokenise the S-expression into a flat list
        let tokens = tokenize(content);
        let mut pos = 0;
        // The top-level node is "(kicad_pcb ...)"
        parse_sexp(&tokens, &mut pos, self);
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

// ── S-expression tokeniser ───────────────────────────────────────────────────

fn tokenize(s: &str) -> Vec<String> {
    let mut tokens = Vec::new();
    let mut chars = s.chars().peekable();
    while let Some(&c) = chars.peek() {
        match c {
            '(' | ')' => {
                tokens.push(c.to_string());
                chars.next();
            }
            '"' => {
                chars.next();
                let mut tok = String::new();
                for ch in chars.by_ref() {
                    if ch == '"' { break; }
                    tok.push(ch);
                }
                tokens.push(tok);
            }
            ' ' | '\t' | '\n' | '\r' => { chars.next(); }
            _ => {
                let mut tok = String::new();
                while let Some(&ch) = chars.peek() {
                    if ch == '(' || ch == ')' || ch == ' ' || ch == '\t'
                        || ch == '\n' || ch == '\r'
                    {
                        break;
                    }
                    tok.push(ch);
                    chars.next();
                }
                tokens.push(tok);
            }
        }
    }
    tokens
}

/// Recursively skip one S-expression node.
fn skip_sexp(tokens: &[String], pos: &mut usize) {
    if *pos >= tokens.len() { return; }
    if tokens[*pos] == "(" {
        *pos += 1;
        while *pos < tokens.len() && tokens[*pos] != ")" {
            skip_sexp(tokens, pos);
        }
        if *pos < tokens.len() { *pos += 1; } // consume ")"
    } else {
        *pos += 1;
    }
}

fn parse_sexp(tokens: &[String], pos: &mut usize, db: &mut KicadPcbDatabase) {
    if *pos >= tokens.len() { return; }
    if tokens[*pos] != "(" { *pos += 1; return; }
    *pos += 1; // consume "("

    if *pos >= tokens.len() { return; }
    let tag = tokens[*pos].clone();
    *pos += 1;

    match tag.as_str() {
        "kicad_pcb" => {
            while *pos < tokens.len() && tokens[*pos] != ")" {
                parse_sexp(tokens, pos, db);
            }
        }
        "layers" => {
            while *pos < tokens.len() && tokens[*pos] != ")" {
                parse_layer_entry(tokens, pos, db);
            }
        }
        "net_class" => {
            parse_netclass(tokens, pos, db);
            return; // ")" already consumed
        }
        "net" => {
            parse_net(tokens, pos, db);
            return;
        }
        "module" | "footprint" => {
            parse_instance(tokens, pos, db);
            return;
        }
        _ => {
            // Skip unknown nodes
            while *pos < tokens.len() && tokens[*pos] != ")" {
                skip_sexp(tokens, pos);
            }
        }
    }

    if *pos < tokens.len() && tokens[*pos] == ")" {
        *pos += 1;
    }
}

fn parse_layer_entry(tokens: &[String], pos: &mut usize, db: &mut KicadPcbDatabase) {
    if *pos >= tokens.len() { return; }
    if tokens[*pos] != "(" { skip_sexp(tokens, pos); return; }
    *pos += 1; // consume "("

    // Format: (layer_number "name" type)
    let id_str = if *pos < tokens.len() { tokens[*pos].clone() } else { return; };
    *pos += 1;
    let name = if *pos < tokens.len() { tokens[*pos].clone() } else { return; };
    *pos += 1;
    let type_str = if *pos < tokens.len() { tokens[*pos].clone() } else { String::new() };
    // Skip rest
    while *pos < tokens.len() && tokens[*pos] != ")" {
        *pos += 1;
    }
    if *pos < tokens.len() { *pos += 1; }

    if let Ok(id) = id_str.parse::<i32>() {
        let layer_type = match type_str.to_lowercase().as_str() {
            "signal" | "power" => LayerType::Copper,
            _ if name.starts_with('F') || name.starts_with('B') || name == "In1.Cu"
                || name.contains(".Cu") => LayerType::Copper,
            _ => LayerType::NonCopper,
        };
        db.layer_id_to_name.insert(id, name.clone());
        db.layers.push(Layer { id, name, layer_type });
    }
}

fn parse_netclass(tokens: &[String], pos: &mut usize, db: &mut KicadPcbDatabase) {
    let mut nc = Netclass::default();
    while *pos < tokens.len() && tokens[*pos] != ")" {
        if tokens[*pos] == "(" {
            *pos += 1;
            if *pos < tokens.len() {
                let tag = tokens[*pos].clone();
                *pos += 1;
                let val = if *pos < tokens.len() { tokens[*pos].clone() } else { String::new() };
                match tag.as_str() {
                    "description" => nc.description = val.clone(),
                    "clearance" => nc.clearance = val.parse().unwrap_or(0.0),
                    "trace_width" => nc.trace_width = val.parse().unwrap_or(0.0),
                    "via_dia" => nc.via_dia = val.parse().unwrap_or(0.0),
                    "via_drill" => nc.via_drill = val.parse().unwrap_or(0.0),
                    "uvia_dia" => nc.uvia_dia = val.parse().unwrap_or(0.0),
                    "uvia_drill" => nc.uvia_drill = val.parse().unwrap_or(0.0),
                    "net_class" => nc.name = val.clone(),
                    _ => {}
                }
                // Skip to end of sub-expression
                while *pos < tokens.len() && tokens[*pos] != ")" {
                    *pos += 1;
                }
                if *pos < tokens.len() { *pos += 1; }
            }
        } else {
            // First bare token after "net_class" tag is the name
            if nc.name.is_empty() {
                nc.name = tokens[*pos].clone();
            }
            *pos += 1;
        }
    }
    if *pos < tokens.len() { *pos += 1; } // consume ")"
    db.netclasses.push(nc);
}

fn parse_net(tokens: &[String], pos: &mut usize, db: &mut KicadPcbDatabase) {
    let id_str = if *pos < tokens.len() { tokens[*pos].clone() } else { String::new() };
    *pos += 1;
    let name = if *pos < tokens.len() { tokens[*pos].clone() } else { String::new() };
    *pos += 1;
    while *pos < tokens.len() && tokens[*pos] != ")" { *pos += 1; }
    if *pos < tokens.len() { *pos += 1; }

    if let Ok(id) = id_str.parse::<i32>() {
        db.nets.push(Net { id, name, netclass_name: String::new(), pins: Vec::new() });
    }
}

fn parse_instance(tokens: &[String], pos: &mut usize, db: &mut KicadPcbDatabase) {
    let mut inst = Instance::default();
    // First token is the reference
    if *pos < tokens.len() && tokens[*pos] != "(" {
        inst.reference = tokens[*pos].clone();
        *pos += 1;
    }
    while *pos < tokens.len() && tokens[*pos] != ")" {
        if tokens[*pos] != "(" { *pos += 1; continue; }
        *pos += 1;
        if *pos >= tokens.len() { break; }
        let tag = tokens[*pos].clone();
        *pos += 1;
        match tag.as_str() {
            "at" => {
                let x: f64 = tokens.get(*pos).and_then(|s| s.parse().ok()).unwrap_or(0.0);
                let y: f64 = tokens.get(*pos + 1).and_then(|s| s.parse().ok()).unwrap_or(0.0);
                inst.position = (x, y);
            }
            "pad" => {
                let pad = parse_pad(tokens, pos, &inst);
                db.pads.push(pad);
                continue; // ")" already consumed inside parse_pad
            }
            "fp_text" | "reference" => {
                if *pos < tokens.len() && tokens[*pos] != "(" {
                    inst.reference = tokens[*pos].clone();
                }
            }
            _ => {}
        }
        while *pos < tokens.len() && tokens[*pos] != ")" { skip_sexp(tokens, pos); }
        if *pos < tokens.len() { *pos += 1; }
    }
    if *pos < tokens.len() { *pos += 1; }
    db.instances.push(inst);
}

fn parse_pad(tokens: &[String], pos: &mut usize, inst: &Instance) -> Pad {
    let mut pad = Pad {
        reference: inst.reference.clone(),
        instance_position: inst.position,
        ..Default::default()
    };
    // pad number
    if *pos < tokens.len() && tokens[*pos] != "(" {
        pad.number = tokens[*pos].clone();
        *pos += 1;
    }
    // pad type
    if *pos < tokens.len() && tokens[*pos] != "(" {
        pad.pad_type = tokens[*pos].clone();
        *pos += 1;
    }
    // pad shape
    if *pos < tokens.len() && tokens[*pos] != "(" {
        pad.shape = tokens[*pos].clone();
        *pos += 1;
    }

    while *pos < tokens.len() && tokens[*pos] != ")" {
        if tokens[*pos] != "(" { *pos += 1; continue; }
        *pos += 1;
        if *pos >= tokens.len() { break; }
        let tag = tokens[*pos].clone();
        *pos += 1;
        match tag.as_str() {
            "at" => {
                let x: f64 = tokens.get(*pos).and_then(|s| s.parse().ok()).unwrap_or(0.0);
                let y: f64 = tokens.get(*pos + 1).and_then(|s| s.parse().ok()).unwrap_or(0.0);
                pad.position = (x + inst.position.0, y + inst.position.1);
            }
            "size" => {
                let w: f64 = tokens.get(*pos).and_then(|s| s.parse().ok()).unwrap_or(0.0);
                let h: f64 = tokens.get(*pos + 1).and_then(|s| s.parse().ok()).unwrap_or(0.0);
                pad.size = (w, h);
            }
            "net" => {
                let net_id: i32 = tokens.get(*pos).and_then(|s| s.parse().ok()).unwrap_or(-1);
                pad.net_id = net_id;
            }
            "layers" => {
                while *pos < tokens.len() && tokens[*pos] != ")" {
                    pad.layers.push(tokens[*pos].clone());
                    *pos += 1;
                }
            }
            _ => {}
        }
        while *pos < tokens.len() && tokens[*pos] != ")" { skip_sexp(tokens, pos); }
        if *pos < tokens.len() { *pos += 1; }
    }
    if *pos < tokens.len() { *pos += 1; } // consume outer ")"
    pad
}

#[cfg(test)]
mod tests {
    use super::*;

    const SIMPLE_PCB: &str = r#"
(kicad_pcb (version 20171130) (host pcbnew 5.1.2)
  (layers
    (0 F.Cu signal)
    (31 B.Cu signal)
    (32 B.Adhes user)
  )
  (net 0 "")
  (net 1 "GND")
  (net 2 "VCC")
  (net_class Default "This is the default net class."
    (clearance 0.2)
    (trace_width 0.25)
    (via_dia 0.8)
    (via_drill 0.4)
    (uvia_dia 0.3)
    (uvia_drill 0.1)
  )
  (module R1
    (at 100 100)
    (pad 1 smd rect
      (at -1.5 0)
      (size 1.5 1.5)
      (layers F.Cu)
      (net 1 "GND")
    )
    (pad 2 smd rect
      (at 1.5 0)
      (size 1.5 1.5)
      (layers F.Cu)
      (net 2 "VCC")
    )
  )
)
"#;

    #[test]
    fn test_tokenize() {
        let tokens = tokenize("(a b (c d))");
        assert_eq!(tokens, vec!["(", "a", "b", "(", "c", "d", ")", ")"]);
    }

    #[test]
    fn test_parse_layers() {
        let db = {
            let mut db = KicadPcbDatabase::new();
            db.parse(SIMPLE_PCB);
            db
        };
        // F.Cu and B.Cu should be present
        let copper: Vec<_> = db.get_copper_layers();
        assert!(copper.iter().any(|l| l.name == "F.Cu"), "F.Cu not found");
        assert!(copper.iter().any(|l| l.name == "B.Cu"), "B.Cu not found");
    }

    #[test]
    fn test_parse_nets() {
        let mut db = KicadPcbDatabase::new();
        db.parse(SIMPLE_PCB);
        assert!(db.nets.len() >= 2);
        assert!(db.nets.iter().any(|n| n.name == "GND"));
        assert!(db.nets.iter().any(|n| n.name == "VCC"));
    }

    #[test]
    fn test_parse_netclass() {
        let mut db = KicadPcbDatabase::new();
        db.parse(SIMPLE_PCB);
        assert!(!db.netclasses.is_empty());
        let nc = &db.netclasses[0];
        assert!((nc.clearance - 0.2).abs() < 1e-9);
        assert!((nc.trace_width - 0.25).abs() < 1e-9);
    }

    #[test]
    fn test_parse_pads() {
        let mut db = KicadPcbDatabase::new();
        db.parse(SIMPLE_PCB);
        assert_eq!(db.pads.len(), 2);
        assert!(db.pads.iter().any(|p| p.net_id == 1));
        assert!(db.pads.iter().any(|p| p.net_id == 2));
    }
}
