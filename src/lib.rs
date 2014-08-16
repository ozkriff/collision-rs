//! A collision detection library for rust

#![license = "ASL2"]

extern crate sync;
extern crate cgmath;
#[cfg(test)] extern crate test;

use cgmath::{Point2, Point3};
use cgmath::BaseNum;

mod linear;
mod sparse;
pub mod bvh;
pub mod aabb;
pub mod sphere;
#[cfg(test)] mod tests;

pub mod octtree {
    pub use linear::Linear;
    pub use sparse::Sparse;
    pub use sparse::CollisionIter;
}

pub trait Intersects<OTHER> {
    fn intersect(&self, other: &OTHER) -> bool;
}

pub trait Center<P> {
    fn center(&self) -> P;
}

pub trait Max<P> {
    fn max(&self) -> P;
}

pub trait Min<P> {
    fn min(&self) -> P;
}

pub trait Merge {
    fn merge(&self, &Self) -> Self;
}

pub trait CheckRange2<S: Clone> {
    fn check_x(&self, centre: S, scale: S) -> (bool, bool);
    fn check_y(&self, centre: S, scale: S) -> (bool, bool);

    fn check2(&self, centre: &Point2<S>, scale: S) -> [bool, ..4] {
        let (lt_x, gt_x) = self.check_x(centre.x.clone(), scale.clone());
        let (lt_y, gt_y) = self.check_y(centre.y.clone(), scale.clone());

        [lt_x && lt_y,
         lt_x && gt_y,
         gt_x && lt_y,
         gt_x && gt_y]
    }
}

pub trait CheckRange3<S: Clone>: CheckRange2<S> {
    fn check_z(&self, centre: S, scale: S) -> (bool, bool);

    fn check3(&self, centre: &Point3<S>, scale: S) -> [bool, ..8] {
        let (lt_x, gt_x) = self.check_x(centre.x.clone(), scale.clone());
        let (lt_y, gt_y) = self.check_y(centre.y.clone(), scale.clone());
        let (lt_z, gt_z) = self.check_z(centre.z.clone(), scale.clone());

        [lt_x && lt_y && lt_z, lt_x && lt_y && gt_z,
         lt_x && gt_y && lt_z, lt_x && gt_y && gt_z,
         gt_x && lt_y && lt_z, gt_x && lt_y && gt_z,
         gt_x && gt_y && lt_z, gt_x && gt_y && gt_z]
    }
}

impl<S: BaseNum> Intersects<Point2<S>> for Point2<S> {
    fn intersect(&self, other: &Point2<S>) -> bool {
        self.x == other.x &&
        self.y == other.y
    }
}

impl<S: BaseNum> Intersects<Point3<S>> for Point3<S> {
    fn intersect(&self, other: &Point3<S>) -> bool {
        self.x == other.x &&
        self.y == other.y &&
        self.z == other.z
    }
}

impl<S: Float+FromPrimitive> CheckRange2<S> for Point3<S> {
    fn check_x(&self, centre: S, _: S) -> (bool, bool) {
        (self.x <= centre, self.x > centre)
    }

    fn check_y(&self, centre: S, _: S) -> (bool, bool) {
        (self.y <= centre, self.y > centre)
    }
}

impl<S: Float+FromPrimitive> CheckRange3<S> for Point3<S> {
    fn check_z(&self, centre: S, _: S) -> (bool, bool) {
        (self.z <= centre, self.z > centre)
    }
}
