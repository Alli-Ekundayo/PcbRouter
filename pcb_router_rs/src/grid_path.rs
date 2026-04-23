use crate::location::Location;

/// A single routed path segment between two grid pins.
///
/// Internally stores both a compact *segments* list (waypoints only) and a
/// dense *locations* list (every individual grid step).
#[derive(Debug, Clone, Default)]
pub struct GridPath {
    /// Dense point list (every grid step along the route).
    pub locations: std::collections::LinkedList<Location>,
    /// Compact waypoint list (start/end of each straight run plus vias).
    pub segments: std::collections::LinkedList<Location>,
}

impl GridPath {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn add_location(&mut self, l: Location) {
        self.locations.push_back(l);
    }

    pub fn copy_locations_to_segments(&mut self) {
        self.segments = self.locations.clone();
    }

    /// Remove collinear intermediate points from `segments`.
    pub fn remove_redundant_points(&mut self) {
        if self.segments.len() <= 2 {
            return;
        }

        let pts: Vec<Location> = self.segments.iter().cloned().collect();
        let mut keep = vec![true; pts.len()];

        for i in 1..pts.len() - 1 {
            let prev = &pts[i - 1];
            let cur = &pts[i];
            let next = &pts[i + 1];
            // Mark collinear (same direction) middle point for removal
            if cur.m_x - prev.m_x == next.m_x - cur.m_x
                && cur.m_y - prev.m_y == next.m_y - cur.m_y
            {
                keep[i] = false;
            }
        }

        self.segments = pts
            .into_iter()
            .zip(keep.into_iter())
            .filter_map(|(l, k)| if k { Some(l) } else { None })
            .collect();
    }

    /// Expand compact segments back into a dense location list.
    pub fn transform_segments_to_locations(&mut self) {
        self.locations.clear();
        let pts: Vec<Location> = self.segments.iter().cloned().collect();
        if pts.is_empty() {
            return;
        }
        self.locations.push_back(pts[0]);

        for i in 1..pts.len() {
            let prev = &pts[i - 1];
            let cur = &pts[i];

            if cur.m_z != prev.m_z {
                // Via
                self.locations.push_back(*cur);
            } else {
                let dx = if cur.m_x > prev.m_x { 1 } else if cur.m_x < prev.m_x { -1 } else { 0 };
                let dy = if cur.m_y > prev.m_y { 1 } else if cur.m_y < prev.m_y { -1 } else { 0 };
                let mut step = *prev;
                while step != *cur {
                    step.m_x += dx;
                    step.m_y += dy;
                    self.locations.push_back(step);
                }
            }
        }
    }

    /// Routed wire-length in database units.
    pub fn get_routed_wirelength(&self, grid_factor: f32) -> f64 {
        let pts: Vec<&Location> = self.segments.iter().collect();
        if pts.is_empty() {
            return 0.0;
        }
        let mut wl = 0.0_f64;
        for i in 1..pts.len() {
            let a = pts[i - 1];
            let b = pts[i];
            if b.m_x != a.m_x || b.m_y != a.m_y {
                wl += grid_factor as f64 * Location::get_distance_2d(a, b);
            }
        }
        wl
    }

    /// Number of vias in this path.
    pub fn get_routed_num_vias(&self) -> i32 {
        let pts: Vec<&Location> = self.segments.iter().collect();
        let mut count = 0;
        for i in 1..pts.len() {
            if pts[i].m_z != pts[i - 1].m_z {
                count += 1;
            }
        }
        count
    }

    /// Number of direction changes (bends) in this path.
    pub fn get_routed_num_bends(&self) -> i32 {
        let pts: Vec<&Location> = self.segments.iter().collect();
        if pts.len() < 3 {
            return 0;
        }
        let mut bends = 0;
        for i in 1..pts.len() - 1 {
            let prev = pts[i - 1];
            let cur = pts[i];
            let next = pts[i + 1];
            if cur.m_z == prev.m_z && cur.m_z == next.m_z {
                let same_dir =
                    cur.m_x - prev.m_x == next.m_x - cur.m_x
                    && cur.m_y - prev.m_y == next.m_y - cur.m_y;
                if !same_dir {
                    bends += 1;
                }
            }
        }
        bends
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_path(locs: &[(i32, i32, i32)]) -> GridPath {
        let mut gp = GridPath::new();
        for &(x, y, z) in locs {
            gp.segments.push_back(Location::new(x, y, z));
        }
        gp
    }

    #[test]
    fn test_remove_redundant_collinear() {
        // Horizontal run: (0,0,0) -> (1,0,0) -> (2,0,0) -> (3,0,0)
        // Middle points should be removed.
        let mut gp = make_path(&[(0,0,0),(1,0,0),(2,0,0),(3,0,0)]);
        gp.remove_redundant_points();
        let pts: Vec<_> = gp.segments.iter().cloned().collect();
        assert_eq!(pts, vec![Location::new(0,0,0), Location::new(3,0,0)]);
    }

    #[test]
    fn test_remove_redundant_preserves_bend() {
        // L-shaped: (0,0,0) -> (2,0,0) -> (2,2,0)
        let mut gp = make_path(&[(0,0,0),(2,0,0),(2,2,0)]);
        gp.remove_redundant_points();
        let pts: Vec<_> = gp.segments.iter().cloned().collect();
        assert_eq!(pts.len(), 3);
    }

    #[test]
    fn test_get_routed_num_vias() {
        let gp = make_path(&[(0,0,0),(0,0,1)]);
        assert_eq!(gp.get_routed_num_vias(), 1);
    }

    #[test]
    fn test_get_routed_num_bends() {
        // L: (0,0,0) -> (3,0,0) -> (3,3,0) — one bend
        let gp = make_path(&[(0,0,0),(3,0,0),(3,3,0)]);
        assert_eq!(gp.get_routed_num_bends(), 1);
    }

    #[test]
    fn test_get_routed_num_bends_straight() {
        let gp = make_path(&[(0,0,0),(5,0,0)]);
        assert_eq!(gp.get_routed_num_bends(), 0);
    }

    #[test]
    fn test_wirelength() {
        // Horizontal 4 units, grid_factor = 0.1 => wirelength ≈ 0.4
        let gp = make_path(&[(0,0,0),(4,0,0)]);
        let wl = gp.get_routed_wirelength(0.1);
        assert!((wl - 0.4).abs() < 1e-5, "wl = {}", wl);
    }

    #[test]
    fn test_transform_segments_to_locations() {
        // Two-point horizontal segment
        let mut gp = make_path(&[(0,0,0),(3,0,0)]);
        gp.transform_segments_to_locations();
        let locs: Vec<_> = gp.locations.iter().cloned().collect();
        assert_eq!(locs.len(), 4); // (0,0,0),(1,0,0),(2,0,0),(3,0,0)
        assert_eq!(locs[0], Location::new(0,0,0));
        assert_eq!(locs[3], Location::new(3,0,0));
    }
}
