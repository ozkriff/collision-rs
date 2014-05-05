#![crate_id="collision#0.1"]
#![crate_type = "rlib"]
#![crate_type = "dylib"]
#![comment = "A collision detection library for rust"]
#![license = "ASL2"]

extern crate cgmath;

use cgmath::aabb::{Aabb2, Aabb3};
use cgmath::partial_ord::PartOrdPrim;

pub trait Intersects<OTHER> {
    fn intersect(&self, other: &OTHER) -> bool;
}

impl<S: PartOrdPrim> Intersects<Aabb2<S>> for Aabb2<S> {
    fn intersect(&self, other: &Aabb2<S>) -> bool {
        !(self.max.x < other.min.x ||
          self.max.y < other.min.y ||
          self.min.x > other.max.x ||
          self.min.y > other.max.y)
    }
}

impl<S: PartOrdPrim> Intersects<Aabb3<S>> for Aabb3<S> {
    fn intersect(&self, other: &Aabb3<S>) -> bool {
        !(self.max.x < other.min.x ||
          self.max.y < other.min.y ||
          self.max.z < other.min.z ||
          self.min.x > other.max.x ||
          self.min.y > other.max.y ||
          self.min.z > other.max.z)
    }
}