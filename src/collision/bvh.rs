
use std::vec::Vec;
use std::iter::range_step;
use std::num::{pow, one};
use std::num::FromPrimitive;
use std::mem::swap;
use std::default::Default;

use cgmath::array::{Array, build};
use cgmath::point::{Point, Point2, Point3};
use cgmath::vector::{Vector, Vector2, Vector3};
use cgmath::partial_ord::PartOrdPrim;

use {Max, Min, Merge, Center, Intersects};

pub struct BvhBuilder<T, C, P> {
    max: P,
    min: P,
    data: Vec<(C, Option<T>)>,
    _data: Vec<(C, Option<T>)>,
    reorder: Vec<(u32, u32)>
}

pub struct Bvh<T, C> {
    depth: uint,
    data: Vec<(C, Option<T>)>,
    _data: Vec<(C, Option<T>)>,
    reorder: Vec<(u32, u32)>
}

impl
<
    S: PartOrdPrim + FromPrimitive,
    V: Vector<S, Slice>,
    P: Point<S, V, Slice> + ToMorton<P, V> + Clone,
    Slice,
    T: Clone,
    C: Merge+Center<P>+Intersects<C>+Default+Max<P>+Min<P>+Clone
> BvhBuilder<T, C, P> {
    pub fn new() -> BvhBuilder<T, C, P> {
        BvhBuilder {
            max: Point::origin(),
            min: Point::origin(),
            data: Vec::new(),
            _data: Vec::new(),
            reorder: Vec::new()
        }
    }

    pub fn add(&mut self, collider: C, data: T) {
        let min = collider.min();
        let max = collider.max();
        self.min = build(|i| self.min.i(i).min(*min.i(i)));
        self.max = build(|i| self.max.i(i).max(*max.i(i)));
        self.data.push((collider.clone(), Some(data)));
    }

    pub fn build(mut self) -> Bvh<T, C> { 
        let step = if self.data.len() != 0 {
            let base = self.min.clone();
            let scale: S = FromPrimitive::from_int(1023).unwrap();
            let unit_vector: V = one();
            let scale = unit_vector.mul_s(scale).div_v(&self.max.sub_p(&self.min));

            self.reorder.truncate(0);
            for (id, &(ref aabb, _)) in self.data.iter().enumerate() {
                self.reorder.push((aabb.center().to_morton(&base, &scale), id as u32))
            }
            self.reorder.sort();

            self._data.truncate(0);
            let mut reorder_iter = self.reorder.iter();
            for idx in range(0, self.reorder.len()*2-1) {
                self._data.push(
                    if idx & 0x1 == 0 {
                        let &(_, v) = reorder_iter.next().unwrap();
                        let &(ref aabb, ref dat) = self.data.get(v as uint);
                        (aabb.clone(), dat.clone())
                    } else {
                        (Default::default(), None)
                    }
                );
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
        } else {
            self.data.truncate(0);
            self.reorder.truncate(0);
            self._data.truncate(0);
            0
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

pub trait ToMorton<P, V> {
    fn to_morton(&self, b: &P, s: &V) -> u32;
}

impl<S: PartOrdPrim+NumCast> ToMorton<Point3<S>, Vector3<S>> for Point3<S> {
    fn to_morton(&self, base: &Point3<S>, scale: &Vector3<S>) -> u32 {
        fn to_morton_code(val: u32) -> u32 {
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
        let x: u32 = ((self.x - base.x) * scale.x).to_u32().unwrap();
        let y: u32 = ((self.y - base.y) * scale.y).to_u32().unwrap();
        let z: u32 = ((self.z - base.z) * scale.z).to_u32().unwrap();
        ((to_morton_code(x)<<2) | (to_morton_code(y)<<1) | to_morton_code(z))        
    }
}

impl<S: PartOrdPrim+NumCast> ToMorton<Point2<S>, Vector2<S>> for Point2<S> {
    fn to_morton(&self, base: &Point2<S>, scale: &Vector2<S>) -> u32 {
        fn to_morton_code(val: u32) -> u32 {
            let mut out = 0;
            let mut mask = 0b1;
            let mut rotate = 0;
            for _ in range(0, 10) {
                out |= (val & mask) << rotate;
                mask <<= 1;
                rotate += 1;
            }
            out
        }
        let x: u32 = ((self.x - base.x) * scale.x).to_u32().unwrap();
        let y: u32 = ((self.y - base.y) * scale.y).to_u32().unwrap();
        ((to_morton_code(x)<<1) | to_morton_code(y))        
    }
}

impl<
    S: PartOrdPrim + FromPrimitive,
    V: Vector<S, Slice>,
    P: Point<S, V, Slice> + ToMorton<P, V>,
    Slice,
    T: Clone,
    C: Merge+Center<P>+Intersects<C>+Default+Max<P>+Min<P>+Clone
> Bvh<T, C> {
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

    pub fn collision_iter<'a, 'b>(&'a self, collider: &'b C) -> BvhCollisionIter<'a, 'b, T, C> {
        BvhCollisionIter {
            bt: pow(2u, self.depth) - 1,
            last: pow(2u, self.depth+1),
            parent: Vec::new(),
            tree: self,
            collider: collider
        }
    }

    pub fn to_builder(mut self) -> BvhBuilder<T, C, P> {
        self.data.truncate(0);
        self._data.truncate(0);
        self.reorder.truncate(0);

        let mut out = BvhBuilder {
            max: Point::origin(),
            min: Point::origin(),
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

pub struct BvhCollisionIter<'a, 'b, T, C> {
    tree: &'a Bvh<T, C>,
    bt: uint,
    last: uint,
    parent: Vec<uint>,
    collider: &'b C
}

impl<
    'a, 'b,
    T: Clone,
    S: PartOrdPrim + FromPrimitive,
    V: Vector<S, Slice>,
    P: Point<S, V, Slice> + ToMorton<P, V>,
    Slice,
    C: Merge+Center<P>+Intersects<C>+Default+Max<P>+Min<P>+Clone
> Iterator<(&'a C, &'a T)> for BvhCollisionIter<'a, 'b, T, C> {
    fn next(&mut self) -> Option<(&'a C, &'a T)> {
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