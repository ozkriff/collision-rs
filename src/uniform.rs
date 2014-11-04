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
    grid: Vec<Option<i32>>
}

impl<C: Center<Point2<f32>>, V> Uniform2D<C, V> {
    pub fn new(size: i32, scale: f32) -> Uniform2D<C, V> {
        Uniform2D {
            scale_inv: (size as f32 / 2.) / scale,
            scale: scale,
            size: size,
            items: Vec::new(),
            grid: Vec::from_fn((size*size) as uint, |_| None)
        }
    }

    fn scale(&self, pt: f32) -> f32 {
        ((self.scale_inv * pt) + (self.size as f32 / 2.))
    }

    fn get_off(&self, pt: f32) -> Option<i32> {
        self.in_range(self.scale(pt) as i32)
    }

    fn in_range(&self, i: i32) -> Option<i32> {
        if i >= self.size || i < 0 {
            None
        } else {
            Some(i)
        }
    }

    pub fn clear(&mut self) {
        self.items.clear();
        for g in self.grid.iter_mut() {
            *g = None;
        }
    }

    pub fn insert(&mut self, collider: C, value: V) {
        let pt = collider.center();
        let x = self.get_off(pt.x);
        let y = self.get_off(pt.y);

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

        let off = self.items.len();
        self.items.push(item);
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

    #[test]
    fn get_off() {
        let grid: Uniform2D<Circle<f32>, int> = Uniform2D::new(2, 1.);
        assert_eq!(grid.get_off(0.), Some(1));
        assert_eq!(grid.get_off(-1.), Some(0));
        assert_eq!(grid.get_off(1.), None);

        let grid: Uniform2D<Circle<f32>, int> = Uniform2D::new(2, 2.);
        assert_eq!(grid.get_off(0.), Some(1));
        assert_eq!(grid.get_off(-1.), Some(0));
        assert_eq!(grid.get_off(-2.), Some(0));
        assert_eq!(grid.get_off(1.), Some(1));
        assert_eq!(grid.get_off(2.), None);

        let grid: Uniform2D<Circle<f32>, int> = Uniform2D::new(4, 2.);
        assert_eq!(grid.get_off(0.), Some(2));
        assert_eq!(grid.get_off(-1.), Some(1));
        assert_eq!(grid.get_off(-2.), Some(0));
        assert_eq!(grid.get_off(1.), Some(3));
        assert_eq!(grid.get_off(2.), None);
    }
}