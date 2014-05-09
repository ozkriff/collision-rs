
use std::vec::Vec;
use std::iter::range_step;
use std::num::pow;
use std::mem::swap;

use cgmath::aabb::{Aabb, Aabb3};
use cgmath::array::{Array, build};
use cgmath::point::{Point, Point3};
use cgmath::vector::{Vector, Vector3};
use cgmath::partial_ord::PartOrdPrim;

use Intersects;

pub struct BvhBuilder<T> {
    max: Point3<f32>,
    min: Point3<f32>,
    data: Vec<(Aabb3<f32>, Option<T>)>,
    _data: Vec<(Aabb3<f32>, Option<T>)>,
    reorder: Vec<(u32, u32)>
}

pub struct Bvh<T> {
    depth: uint,
    data: Vec<(Aabb3<f32>, Option<T>)>,
    _data: Vec<(Aabb3<f32>, Option<T>)>,
    reorder: Vec<(u32, u32)>
}

impl<T: Clone> BvhBuilder<T> {
    pub fn new() -> BvhBuilder<T> {
        BvhBuilder {
            max: Point3::new(0f32, 0., 0.),
            min: Point3::new(0f32, 0., 0.),
            data: Vec::new(),
            _data: Vec::new(),
            reorder: Vec::new()
        }
    }

    pub fn add(&mut self, collider: Aabb3<f32>, data: T) {
        self.min = build(|i| self.min.i(i).min(*collider.min.i(i)));
        self.max = build(|i| self.max.i(i).max(*collider.max.i(i)));
        self.data.push((collider.clone(), Some(data)));
    }

    pub fn build(mut self) -> Bvh<T> { 
        let step = {
            let base = self.min;
            let scale = Vector3::new(1023f32, 1023f32, 1023f32)
                    .div_v(&self.max.sub_p(&self.min));

            self.reorder.truncate(0);
            for (id, &(ref aabb, _)) in self.data.iter().enumerate() {
                self.reorder.push((to_morton3(&aabb.center(), &base, &scale), id as u32))
            }
            self.reorder.sort();

            self._data.truncate(0);
            let mut reorder_iter = self.reorder.iter();
            for idx in range(0, self.reorder.len()*2-1) {
                self._data.push(if idx & 0x1 == 0 {
                    let &(_, v) = reorder_iter.next().unwrap();
                    let &(aabb, ref dat) = self.data.get(v as uint);
                    (aabb, dat.clone())
                } else {
                    (Aabb3::new(Point3::new(0f32, 0., 0.),
                                Point3::new(0f32, 0., 0.)),
                     None)
                });
            }

            let mut step = 1;
            loop {
                let reach = pow(2u, step);
                let half_reach = pow(2u, step-1);

                if reach > self._data.len() {
                    break
                }

                for i in range_step(reach-1, self._data.len(), pow(2u, step+1)) {
                    let left = i - half_reach;
                    let mut right = i + half_reach;
                    let mut hr = half_reach;
                    while right >= self._data.len() {
                        hr /= 2;
                        right = i + hr;
                    }
                    let new = match (self._data.get(left), self._data.get(right)) {
                        (&(ref left, _), &(ref right, _)) => left.merge(right)
                    };
                    *self._data.get_mut(i) = (new, None);
                }

                step += 1;
            }
            step
        };

        let mut out = Bvh {
            depth: step - 1,
            data: Vec::new(),
            _data: Vec::new(),
            reorder: Vec::new()
        };

        swap(&mut out.data, &mut self._data);
        swap(&mut out._data, &mut self.data);
        swap(&mut out.reorder, &mut self.reorder);

        out
    }
}

pub fn to_morton3(c: &Point3<f32>, base: &Point3<f32>, scale: &Vector3<f32>) -> u32 {
    fn to_morton(val: u32) -> u32 {
        let mut out = 0;
        let mut mask = 0b1;
        let mut rotate = 0;
        for _ in range(0, 10) {
            out |= (val & mask) << rotate;
            mask <<= 1;
            rotate += 2;
        }
        out
    }
    let x = ((c.x - base.x) * scale.x) as u32;
    let y = ((c.y - base.y) * scale.y) as u32;
    let z = ((c.z - base.z) * scale.z) as u32;
    ((to_morton(x)<<2) | (to_morton(y)<<1) | to_morton(z))
}

impl<T> Bvh<T> {
    fn depth(&self, idx: uint) -> uint {
        let mut depth = 0;
        let mut mask = 0b1;
        loop {
            if idx & mask != mask {
                break
            }

            depth += 1;
            mask |= 1 << depth;
        }
        assert!(depth != 0);
        depth  
    }

    fn children(&self, idx: uint) -> (uint, uint) {
        let depth = self.depth(idx);

        let mut half = pow(2u, depth-1);
        let left = idx - half;
        let mut right = idx + half;

        // if the right is beyond the end, reselect a child node
        while right >= self.data.len() {
            half /= 2;
            right = idx + half;
        }

        (left, right)
    }

    pub fn collision_iter<'a, 'b>(&'a self, collider: &'b Aabb3<f32>) -> BvhCollisionIter<'a, 'b, T> {
        BvhCollisionIter {
            bt: pow(2u, self.depth) - 1,
            last: pow(2u, self.depth+1),
            parent: Vec::new(),
            tree: self,
            collider: collider
        }
    }

    pub fn to_builder(mut self) -> BvhBuilder<T> {
        self.data.truncate(0);
        self._data.truncate(0);
        self.reorder.truncate(0);

        let mut out = BvhBuilder {
            max: Point3::new(0f32, 0., 0.),
            min: Point3::new(0f32, 0., 0.),
            data: Vec::new(),
            _data: Vec::new(),
            reorder: Vec::new()
        };

        swap(&mut out.data, &mut self.data);
        swap(&mut out._data, &mut self._data);
        swap(&mut out.reorder, &mut self.reorder);

        out
    }
}

pub struct BvhCollisionIter<'a, 'b, T> {
    tree: &'a Bvh<T>,
    bt: uint,
    last: uint,
    parent: Vec<uint>,
    collider: &'b Aabb3<f32>
}

impl<'a, 'b, T> Iterator<(&'a Aabb3<f32>, &'a T)> for BvhCollisionIter<'a, 'b, T> {
    fn next(&mut self) -> Option<(&'a Aabb3<f32>, &'a T)> {
        loop {
            let &(ref aabb, ref dat) = self.tree.data.get(self.bt);

            if dat.is_some() {
                self.last = self.bt;
                self.bt = match self.parent.pop() {
                    Some(p) => p,
                    None => return None
                };
                if aabb.intersect(self.collider) {
                    return Some((aabb, dat.as_ref().unwrap()));      
                }          
            }

            let (left, right) = self.tree.children(self.bt);
            // returning from right
            if self.last == right {
                self.parent.push(self.bt);
                self.last = self.bt;
                self.bt = left;
            // return from left, all children have been exhausted so ascend
            // or if we are new to this node, but do not collide with it
            } else if self.last == left || !aabb.intersect(self.collider) {
                self.last = self.bt;
                self.bt = match self.parent.pop() {
                    Some(p) => p,
                    None => return None
                }
            // this collides with the cell, descend to the right branch
            } else {
                self.parent.push(self.bt);
                self.last = self.bt;
                self.bt = right;
            }
        }
    }
}