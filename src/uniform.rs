use cgmath::Point2;
use super::{Center, Intersects};
use core::fmt::Show;

#[deriving(Clone)]
struct Item<C, V> {
    next: Option<i32>,
    collider: C,
    value: V
}

#[deriving(Clone)]
pub struct Uniform2D<C, V> {
    scale: f32,
    scale_inv: f32,
    size: i32,
    items: Vec<Item<C, V>>,
    free: Vec<i32>,
    grid: Vec<Option<i32>>
}

impl<C: Center<Point2<f32>>, V> Uniform2D<C, V> {
    pub fn new(size: i32, scale: f32) -> Uniform2D<C, V> {
        Uniform2D {
            scale_inv: (size as f32 / 2.) / scale,
            scale: scale,
            size: size,
            items: Vec::new(),
            free: Vec::new(),
            grid: Vec::from_fn((size*size) as uint, |_| None)
        }
    }

    fn scale(&self, pt: f32) -> f32 {
        ((self.scale_inv * pt) + (self.size as f32 / 2.))
    }

    fn get_offset(&self, pt: f32) -> Option<i32> {
        self.in_range(self.scale(pt) as i32)
    }

    fn in_range(&self, i: i32) -> Option<i32> {
        if i >= self.size || i < 0 {
            None
        } else {
            Some(i)
        }
    }

    fn link(&mut self, item: Item<C, V>) -> i32 {
        if let Some(idx) = self.free.pop() {
            self.items[idx as uint] = item;
            idx
        } else {
            let last = self.items.len();
            self.items.push(item);
            last as i32
        }
    }

    pub fn clear(&mut self) {
        self.items.clear();
        self.free.clear();
        for g in self.grid.iter_mut() {
            *g = None;
        }
    }

    pub fn insert(&mut self, collider: C, value: V) {
        let pt = collider.center();
        let x = self.get_offset(pt.x);
        let y = self.get_offset(pt.y);

        let (x, y) = match (x, y) {
            (Some(x), Some(y)) => (x, y),
            _ => return
        };

        let idx = (x * self.size + y) as uint;

        let item = Item {
            next: self.grid[idx],
            collider: collider,
            value: value
        };

        let off = self.link(item);
        self.grid[idx] = Some(off as i32);
    }

    pub fn collision_iter<'a>(&'a self, collide: &'a C) -> Uniform2DIterator<'a, C, V> {
        Uniform2DIterator {
            grid: self,
            x: -2,
            y: -1,
            collide: collide,
            cell: None
        }
    }
}


impl<C: Center<Point2<f32>>, V: Eq> Uniform2D<C, V> {
    fn find(&self, idx: uint, v: &V) -> Option<i32> {
        let mut start = self.grid[idx];
        while let Some(idx) = start {
            if &self.items[idx as uint].value == v {
                return Some(idx);
            }
            start = self.items[idx as uint].next
        }
        None
    }

    pub fn remove(&mut self, collider: &C, value: &V) -> bool {
        let pt = collider.center();
        let x = self.get_offset(pt.x);
        let y = self.get_offset(pt.y);

        let (x, y) = match (x, y) {
            (Some(x), Some(y)) => (x, y),
            _ => return false
        };

        let idx = (x * self.size + y) as uint;
        if let Some(i) = self.find(idx, value) {
            self.free.push(i);

            // remove link if it is at the start of the list
            let mut start = self.grid[idx];
            if let Some(s) = start {
                if s == i {
                    self.grid[idx] = self.items[i as uint].next;
                    return true;
                }
            }

            // remove link if it is linked in the items
            while let Some(s) = start {
                if self.items[s as uint].next == Some(i) {
                    self.items[s as uint].next = self.items[i as uint].next;
                }
                start = self.items[s as uint].next
            }
        }
        false
    }
}

pub struct Uniform2DIterator<'a, C:'a, V:'a> {
    grid: &'a Uniform2D<C, V>,
    collide: &'a C,
    x: i8,
    y: i8,
    cell: Option<i32>
}

impl<'a, C: Center<Point2<f32>>+Intersects<C>+Show, V> Iterator<(&'a C, &'a V)> for Uniform2DIterator<'a, C, V> {
    #[inline(never)]
    fn next(&mut self) -> Option<(&'a C, &'a V)> {
        loop {
            while let Some(idx) = self.cell {
                let cell = &self.grid.items[idx as uint];
                self.cell = cell.next;

                if self.collide.intersect(&cell.collider) {
                    return Some((&cell.collider, &cell.value));
                }
            }

            // next cell
            self.x += 1;
            if self.x == 2 && self.y == 1 {
                return None;
            } else if self.x == 2 {
                self.x = -1;
                self.y += 1;
            }

            let point = self.collide.center();
            let x = self.grid.scale(point.x) as i32;
            let y = self.grid.scale(point.y) as i32;
            let x = self.grid.in_range(x + self.x as i32);
            let y = self.grid.in_range(y + self.y as i32);

            self.cell = match (x, y) {
                (Some(x), Some(y)) => self.grid.grid[(x*self.grid.size+y) as uint],
                _ => None
            };
        }
    }
}

#[cfg(test)]
mod tests {
    use super::Uniform2D;
    use super::super::sphere::Circle;
    use std::collections::HashSet;
    use cgmath::Point2;

    #[test]
    fn get_offset() {
        let grid: Uniform2D<Circle<f32>, int> = Uniform2D::new(2, 1.);
        assert_eq!(grid.get_offset(0.), Some(1));
        assert_eq!(grid.get_offset(-1.), Some(0));
        assert_eq!(grid.get_offset(1.), None);

        let grid: Uniform2D<Circle<f32>, int> = Uniform2D::new(2, 2.);
        assert_eq!(grid.get_offset(0.), Some(1));
        assert_eq!(grid.get_offset(-1.), Some(0));
        assert_eq!(grid.get_offset(-2.), Some(0));
        assert_eq!(grid.get_offset(1.), Some(1));
        assert_eq!(grid.get_offset(2.), None);

        let grid: Uniform2D<Circle<f32>, int> = Uniform2D::new(4, 2.);
        assert_eq!(grid.get_offset(0.), Some(2));
        assert_eq!(grid.get_offset(-1.), Some(1));
        assert_eq!(grid.get_offset(-2.), Some(0));
        assert_eq!(grid.get_offset(1.), Some(3));
        assert_eq!(grid.get_offset(2.), None);
    }

    #[test]
    fn insert() {
        let to_find = vec![(Circle::new(Point2::new(0f32, 0f32), 1f32), 0i),
                           (Circle::new(Point2::new(0f32, 0f32), 1f32), 1i),
                           (Circle::new(Point2::new(0f32, 0f32), 1f32), 2i)];

        let mut grid: Uniform2D<Circle<f32>, int> = Uniform2D::new(2, 1.);
        for &(k, v) in to_find.iter() {
            grid.insert(k, v);
        }

        let mut set: HashSet<int> = [0, 1, 2].iter().map(|&x| x).collect();
        for (_, v) in grid.collision_iter(&Circle::new(Point2::new(0f32, 0f32), 1f32)) {
            set.remove(v);
        }
        assert_eq!(set.len(), 0);
    }

    #[test]
    fn remove() {
        let to_find = vec![(Circle::new(Point2::new(0f32, 0f32), 1f32), 0i),
                           (Circle::new(Point2::new(0f32, 0f32), 1f32), 1i),
                           (Circle::new(Point2::new(0f32, 0f32), 1f32), 2i)];

        let mut grid: Uniform2D<Circle<f32>, int> = Uniform2D::new(2, 1.);
        for &(k, v) in to_find.iter() {
            grid.insert(k, v);
        }

        let mut set: HashSet<int> = [0, 1, 2].iter().map(|&x| x).collect();
        for (_, v) in grid.collision_iter(&Circle::new(Point2::new(0f32, 0f32), 1f32)) {
            set.remove(v);
        }
        assert_eq!(set.len(), 0);

        grid.remove(&to_find[0].0, &to_find[0].1);
        let mut set: HashSet<int> = [0, 1, 2].iter().map(|&x| x).collect();
        for (_, v) in grid.collision_iter(&Circle::new(Point2::new(0f32, 0f32), 1f32)) {
            set.remove(v);
        }
        assert_eq!(set.len(), 1);

        grid.remove(&to_find[2].0, &to_find[2].1);
        let mut set: HashSet<int> = [0, 1, 2].iter().map(|&x| x).collect();
        for (_, v) in grid.collision_iter(&Circle::new(Point2::new(0f32, 0f32), 1f32)) {
            set.remove(v);
        }
        assert_eq!(set.len(), 2);

        grid.remove(&to_find[1].0, &to_find[1].1);
        let mut set: HashSet<int> = [0, 1, 2].iter().map(|&x| x).collect();
        for (_, v) in grid.collision_iter(&Circle::new(Point2::new(0f32, 0f32), 1f32)) {
            set.remove(v);
        }
        assert_eq!(set.len(), 3);
    }
}