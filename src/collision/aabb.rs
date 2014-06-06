// Copyright 2013 The CGMath Developers. For a full listing of the authors,
// refer to the AUTHORS file at the top-level directory of this distribution.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//! Axis-aligned bounding boxes

use std::fmt;
use std::num::{zero, one};
use std::iter::{FromIterator, Iterator};
use std::default::Default;

use cgmath::point::{Point, Point2, Point3};
use cgmath::vector::{Vector, Vector2, Vector3};
use cgmath::num::BaseNum;
use cgmath::approx::ApproxEq;

use {Max, Min, Center, Merge, Intersects, CheckRange2, CheckRange3};

pub trait Aabb
<
    S: BaseNum,
    V: Vector<S>,
    P: Point<S, V>
> {
    fn new(p1: P, p2: P) -> Self;
    fn aabb_min<'a>(&'a self) -> &'a P;
    fn aabb_max<'a>(&'a self) -> &'a P;
    #[inline] fn dim(&self) -> V { self.aabb_max().sub_p(self.aabb_min()) }
    #[inline] fn volume(&self) -> S { self.dim().comp_mul() }

    // Tests whether a point is cointained in the box, inclusive for min corner
    // and exclusive for the max corner.
    fn contains(&self, p: &P) -> bool;

    // Returns a new AABB that is grown to include the given point.
    fn grow(&self, p: &P) -> Self {
        let min = self.aabb_min().min(p);
        let max = self.aabb_max().max(p);
        Aabb::new(min, max)
    }

    // Returns a new AABB that has its points translated by the given vector.
    fn add_v(&self, v: &V) -> Self {
        Aabb::new(self.aabb_min().add_v(v), self.aabb_max().add_v(v))
    }

    fn mul_s(&self, s: S) -> Self {
        Aabb::new(self.aabb_min().mul_s(s.clone()), self.aabb_max().mul_s(s.clone()))
    }

    fn mul_v(&self, v: &V) -> Self {
        let min : P = Point::from_vec(&self.aabb_min().to_vec().mul_v(v));
        let max : P = Point::from_vec(&self.aabb_max().to_vec().mul_v(v));
        Aabb::new(min, max)
    }
}

#[deriving(PartialEq, Clone)]
pub struct Aabb2<S> {
    pub min: Point2<S>,
    pub max: Point2<S>,
}

impl<S: BaseNum> Aabb2<S> {
    /// Construct a new axis-aligned bounding box from two points.
    #[inline]
    pub fn new(p1: Point2<S>, p2: Point2<S>) -> Aabb2<S> {
        Aabb2 {
            min: Point2::new(p1.x.partial_min(p2.x),
                             p1.y.partial_min(p2.y)),
            max: Point2::new(p1.x.partial_max(p2.x),
                             p1.y.partial_max(p2.y)),
        }
    }
}

impl<S: BaseNum> Aabb<S, Vector2<S>, Point2<S>> for Aabb2<S> {
    fn new(p1: Point2<S>, p2: Point2<S>) -> Aabb2<S> { Aabb2::new(p1, p2) }
    #[inline] fn aabb_min<'a>(&'a self) -> &'a Point2<S> { &self.min }
    #[inline] fn aabb_max<'a>(&'a self) -> &'a Point2<S> { &self.max }

    #[inline]
    fn contains(&self, p: &Point2<S>) -> bool {
        let v_min = p.sub_p(self.aabb_min());
        let v_max = self.aabb_max().sub_p(p);
        v_min.x >= zero() && v_min.y >= zero() &&
        v_max.x >  zero() && v_max.y >  zero()
    }
}

impl<S: fmt::Show+BaseNum> fmt::Show for Aabb2<S> {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "[{} - {}]", self.min, self.max)
    }
}

impl<S: BaseNum> FromIterator<Point2<S>> for Aabb2<S> {
    fn from_iter<T: Iterator<Point2<S>>>(iterator: T) -> Aabb2<S> {
        let mut iterator = iterator;

        let (mut max, mut min) = match iterator.next() {
            Some(m) => (Point2::new(m.x, m.y), Point2::new(m.x, m.y)),
            None => return Aabb2::new(Point2::new(zero(), zero()),
                                      Point2::new(zero(), zero())),
        };

        for point in iterator {
            max.x = max.x.partial_max(point.x);
            max.y = max.y.partial_max(point.y);
            min.x = min.x.partial_min(point.x);
            min.y = min.y.partial_min(point.y);
        }

        Aabb2 {
            min: min,
            max: max
        }
    }
}

#[deriving(Clone, PartialEq)]
pub struct Aabb3<S> {
    pub min: Point3<S>,
    pub max: Point3<S>,
}

impl<S: BaseNum> Aabb3<S> {
    /// Construct a new axis-aligned bounding box from two points.
    #[inline]
    pub fn new(p1: Point3<S>, p2: Point3<S>) -> Aabb3<S> {
        Aabb3 {
            min: Point3::new(p1.x.partial_min(p2.x),
                             p1.y.partial_min(p2.y),
                             p1.z.partial_min(p2.z)),
            max: Point3::new(p1.x.partial_max(p2.x),
                             p1.y.partial_max(p2.y),
                             p1.z.partial_max(p2.z)),
        }
    }
}

impl<S: BaseNum> Aabb<S, Vector3<S>, Point3<S>> for Aabb3<S> {
    fn new(p1: Point3<S>, p2: Point3<S>) -> Aabb3<S> { Aabb3::new(p1, p2) }
    #[inline] fn aabb_min<'a>(&'a self) -> &'a Point3<S> { &self.min }
    #[inline] fn aabb_max<'a>(&'a self) -> &'a Point3<S> { &self.max }

    #[inline]
    fn contains(&self, p: &Point3<S>) -> bool {
        let v_min = p.sub_p(self.aabb_min());
        let v_max = self.aabb_max().sub_p(p);
        v_min.x >= zero() && v_min.y >= zero() && v_min.z >= zero() &&
        v_max.x >  zero() && v_max.y >  zero() && v_max.z >  zero()
    }
}

impl<S: fmt::Show+BaseNum> fmt::Show for Aabb3<S> {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "[{} - {}]", self.min, self.max)
    }
}

impl<S: BaseNum> FromIterator<Point3<S>> for Aabb3<S> {
    fn from_iter<T: Iterator<Point3<S>>>(iterator: T) -> Aabb3<S> {
        let mut iterator = iterator;

        let (mut max, mut min) = match iterator.next() {
            Some(m) => (Point3::new(m.x, m.y, m.z), Point3::new(m.x, m.y, m.z)),
            None => return Aabb3::new(Point3::new(zero(), zero(), zero()),
                                      Point3::new(zero(), zero(), zero())),
        };

        for point in iterator {
            max.x = max.x.partial_max(point.x);
            max.y = max.y.partial_max(point.y);
            max.z = max.z.partial_max(point.z);
            min.x = min.x.partial_min(point.x);
            min.y = min.y.partial_min(point.y);
            min.z = min.z.partial_min(point.z);
        }

        Aabb3 {
            min: min,
            max: max
        }
    }
}

impl<S: Float+ApproxEq<S>> CheckRange2<S> for Aabb3<S> {
    fn check_x(&self, centre: S, _: S) -> (bool, bool) {
        (self.min.x <= centre,
         self.max.x > centre)
    }

    fn check_y(&self, centre: S, _: S) -> (bool, bool) {
        (self.min.y <= centre,
         self.max.y > centre)
    }
}

impl<S: Float+ApproxEq<S>> CheckRange3<S> for Aabb3<S> {
    fn check_z(&self, centre: S, _: S) -> (bool, bool) {
        (self.min.z <= centre,
         self.max.z > centre)
    }
}

impl<S: BaseNum> Intersects<Aabb2<S>> for Aabb2<S> {
    fn intersect(&self, other: &Aabb2<S>) -> bool {
        !(self.max.x < other.min.x ||
          self.max.y < other.min.y ||
          self.min.x > other.max.x ||
          self.min.y > other.max.y)
    }
}

impl<S: BaseNum> Intersects<Aabb3<S>> for Aabb3<S> {
    fn intersect(&self, other: &Aabb3<S>) -> bool {
        !(self.max.x < other.min.x ||
          self.max.y < other.min.y ||
          self.max.z < other.min.z ||
          self.min.x > other.max.x ||
          self.min.y > other.max.y ||
          self.min.z > other.max.z)
    }
}

impl<S: BaseNum> Center<Point2<S>> for Aabb2<S> {
    fn center(&self) -> Point2<S> {
        let two = one::<S>() + one::<S>();
        self.aabb_min().add_v(&self.dim().div_s(two))
    }
}

impl<S: BaseNum> Center<Point3<S>> for Aabb3<S> {
    fn center(&self) -> Point3<S> {
        let two = one::<S>() + one::<S>();
        self.aabb_min().add_v(&self.dim().div_s(two))
    }
}

impl<S: BaseNum> Max<Point2<S>> for Aabb2<S> {
    fn max(&self) -> Point2<S> {
        self.max.clone()
    }
}

impl<S: BaseNum> Max<Point3<S>> for Aabb3<S> {
    fn max(&self) -> Point3<S> {
        self.max.clone()
    }
}

impl<S: BaseNum> Min<Point2<S>> for Aabb2<S> {
    fn min(&self) -> Point2<S> {
        self.max.clone()
    }
}

impl<S: BaseNum> Min<Point3<S>> for Aabb3<S> {
    fn min(&self) -> Point3<S> {
        self.min.clone()
    }
}

impl<S: BaseNum> Merge for Aabb2<S> {
    fn merge(&self, other: &Aabb2<S>) -> Aabb2<S> {
        let max = Point2::new(self.max.x.partial_max(other.max.x),
                              self.max.y.partial_max(other.max.y));
        let min = Point2::new(self.min.x.partial_min(other.min.x),
                              self.min.y.partial_min(other.min.y));

        Aabb2 {
            min: min,
            max: max
        }
    }
}

impl<S: BaseNum> Merge for Aabb3<S> {
    fn merge(&self, other: &Aabb3<S>) -> Aabb3<S> {
        let max = Point3::new(self.max.x.partial_max(other.max.x),
                              self.max.y.partial_max(other.max.y),
                              self.max.z.partial_max(other.max.z));
        let min = Point3::new(self.min.x.partial_min(other.min.x),
                              self.min.y.partial_min(other.min.y),
                              self.min.z.partial_min(other.min.z));

        Aabb3 {
            min: min,
            max: max
        }
    }
}

impl<S: BaseNum> Default for Aabb3<S> {
    fn default() -> Aabb3<S> {
        Aabb3 {
            min: Point::origin(),
            max: Point::origin()
        }
    }
}

impl<S: BaseNum> Default for Aabb2<S> {
    fn default() -> Aabb2<S> {
        Aabb2 {
            min: Point::origin(),
            max: Point::origin()
        }
    }
}