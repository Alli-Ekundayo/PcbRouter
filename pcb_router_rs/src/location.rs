use std::collections::BinaryHeap;
use std::cmp::Ordering;
use std::fmt;

/// 3-D grid location (x, y, layer).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct Location {
    pub m_x: i32,
    pub m_y: i32,
    pub m_z: i32,
}

impl Location {
    pub fn new(x: i32, y: i32, z: i32) -> Self {
        Location { m_x: x, m_y: y, m_z: z }
    }

    #[inline]
    pub fn x(&self) -> i32 { self.m_x }
    #[inline]
    pub fn y(&self) -> i32 { self.m_y }
    #[inline]
    pub fn z(&self) -> i32 { self.m_z }

    /// Chebyshev (chessboard) distance in 2D.
    pub fn chebyshev_2d(a: &Location, b: &Location) -> i32 {
        let dx = (a.m_x - b.m_x).abs();
        let dy = (a.m_y - b.m_y).abs();
        dx.max(dy)
    }

    /// Euclidean-ish diagonal distance in 2D (octile distance).
    pub fn get_distance_2d(a: &Location, b: &Location) -> f64 {
        let dx = (a.m_x - b.m_x).abs() as f64;
        let dy = (a.m_y - b.m_y).abs() as f64;
        let min_d = dx.min(dy);
        let max_d = dx.max(dy);
        min_d * std::f64::consts::SQRT_2 + (max_d - min_d)
    }
}

impl fmt::Display for Location {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "({}, {}, {})", self.m_x, self.m_y, self.m_z)
    }
}

/// Priority-queue entry that reverses the order so BinaryHeap (max-heap)
/// behaves as a min-heap when keyed on cost.
#[derive(Debug, Clone, PartialEq)]
struct PqEntry<T> {
    priority: f32,
    item: T,
}

impl<T: PartialEq> Eq for PqEntry<T> {}

impl<T: PartialEq> PartialOrd for PqEntry<T> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl<T: PartialEq> Ord for PqEntry<T> {
    fn cmp(&self, other: &Self) -> Ordering {
        // Invert to make BinaryHeap a min-heap
        other.priority.partial_cmp(&self.priority)
            .unwrap_or(Ordering::Equal)
    }
}

/// Min-priority queue (lower priority number = higher urgency).
pub struct LocationQueue<T> {
    heap: BinaryHeap<PqEntry<T>>,
}

impl<T: PartialEq + Clone> LocationQueue<T> {
    pub fn new() -> Self {
        LocationQueue { heap: BinaryHeap::new() }
    }

    pub fn is_empty(&self) -> bool {
        self.heap.is_empty()
    }

    pub fn len(&self) -> usize {
        self.heap.len()
    }

    pub fn push(&mut self, item: T, priority: f32) {
        self.heap.push(PqEntry { priority, item });
    }

    /// Peek at the best (lowest-priority) item.
    pub fn front(&self) -> Option<&T> {
        self.heap.peek().map(|e| &e.item)
    }

    /// Priority of the best item.
    pub fn front_key(&self) -> Option<f32> {
        self.heap.peek().map(|e| e.priority)
    }

    pub fn pop(&mut self) -> Option<T> {
        self.heap.pop().map(|e| e.item)
    }
}

impl<T: PartialEq + Clone> Default for LocationQueue<T> {
    fn default() -> Self { Self::new() }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_location_equality() {
        let a = Location::new(1, 2, 0);
        let b = Location::new(1, 2, 0);
        let c = Location::new(1, 3, 0);
        assert_eq!(a, b);
        assert_ne!(a, c);
    }

    #[test]
    fn test_location_distance_2d() {
        let a = Location::new(0, 0, 0);
        let b = Location::new(3, 3, 0);
        let d = Location::get_distance_2d(&a, &b);
        assert!((d - 3.0 * std::f64::consts::SQRT_2).abs() < 1e-10);

        let c = Location::new(0, 0, 0);
        let d2 = Location::get_distance_2d(&c, &Location::new(4, 0, 0));
        assert!((d2 - 4.0).abs() < 1e-10);
    }

    #[test]
    fn test_location_chebyshev() {
        let a = Location::new(0, 0, 0);
        let b = Location::new(3, 5, 0);
        assert_eq!(Location::chebyshev_2d(&a, &b), 5);
    }

    #[test]
    fn test_location_queue_min_heap() {
        let mut q: LocationQueue<Location> = LocationQueue::new();
        q.push(Location::new(1, 0, 0), 5.0);
        q.push(Location::new(2, 0, 0), 2.0);
        q.push(Location::new(3, 0, 0), 8.0);
        q.push(Location::new(4, 0, 0), 1.0);

        // Should come out in ascending priority order
        assert_eq!(q.front_key(), Some(1.0));
        assert_eq!(q.pop().unwrap(), Location::new(4, 0, 0));
        assert_eq!(q.pop().unwrap(), Location::new(2, 0, 0));
        assert_eq!(q.pop().unwrap(), Location::new(1, 0, 0));
        assert_eq!(q.pop().unwrap(), Location::new(3, 0, 0));
        assert!(q.is_empty());
    }

    #[test]
    fn test_location_display() {
        let loc = Location::new(3, 7, 1);
        assert_eq!(format!("{}", loc), "(3, 7, 1)");
    }
}
