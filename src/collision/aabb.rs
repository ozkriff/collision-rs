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

use cgmath::point::{Point, Point2, Point3};
use cgmath::vector::{Vector, Vector2, Vector3};
use cgmath::array::build;
use cgmath::partial_ord::PartOrdPrim;
use cgmath::approx::ApproxEq;

use {Center, Merge, Intersects, CheckRange2, CheckRange3};

pub trait Aabb
<
    S: PartOrdPrim,
    V: Vector<S, Slice>,
    P: Point<S, V, Slice>,
    Slice
> {
    fn new(p1: P, p2: P) -> Self;
    fn min<'a>(&'a self) -> &'a P;
    fn max<'a>(&'a self) -> &'a P;
    #[inline] fn dim(&self) -> V { self.max().sub_p(self.min()) }
    #[inline] fn volume(&self) -> S { self.dim().comp_mul() }

    // Tests whether a point is cointained in the box, inclusive for min corner
    // and exclusive for the max corner.
    #[inline] fn contains(&self, p: &P) -> bool {
        p.sub_p(self.min()).iter().all(|x| *x >= zero::<S>()) &&
        self.max().sub_p(p).iter().all(|x| *x > zero::<S>())
    }

    // Returns a new AABB that is grown to include the given point.
    fn grow(&self, p: &P) -> Self {
        let min : P = build(|i| self.min().i(i).min(*p.i(i)));
        let max : P = build(|i| self.max().i(i).max(*p.i(i)));
        Aabb::new(min, max)
    }

    // Returns a new AABB that has its points translated by the given vector.
    fn add_v(&self, v: &V) -> Self {
        Aabb::new(self.min().add_v(v), self.max().add_v(v))
    }

    fn mul_s(&self, s: S) -> Self {
        Aabb::new(self.min().mul_s(s.clone()), self.max().mul_s(s.clone()))
    }

    fn mul_v(&self, v: &V) -> Self {
        let min : P = Point::from_vec(&self.min().to_vec().mul_v(v));
        let max : P = Point::from_vec(&self.max().to_vec().mul_v(v));
        Aabb::new(min, max)
    }
}

#[deriving(Clone, Eq)]
pub struct Aabb2<S> {
    pub min: Point2<S>,
    pub max: Point2<S>,
}

impl<S: PartOrdPrim> Aabb2<S> {
    /// Construct a new axis-aligned bounding box from two points.
    #[inline]
    pub fn new(p1: Point2<S>, p2: Point2<S>) -> Aabb2<S> {
        Aabb2 {
            min: Point2::new(p1.x.min(p2.x),
                             p1.y.min(p2.y)),
            max: Point2::new(p1.x.max(p2.x),
                             p1.y.max(p2.y)),
        }
    }
}

impl<S: PartOrdPrim> Aabb<S, Vector2<S>, Point2<S>, [S, ..2]> for Aabb2<S> {
    fn new(p1: Point2<S>, p2: Point2<S>) -> Aabb2<S> { Aabb2::new(p1, p2) }
    #[inline] fn min<'a>(&'a self) -> &'a Point2<S> { &self.min }
    #[inline] fn max<'a>(&'a self) -> &'a Point2<S> { &self.max }
}

impl<S: fmt::Show> fmt::Show for Aabb2<S> {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f.buf, "[{} - {}]", self.min, self.max)
    }
}

impl<S: PartOrdPrim> FromIterator<Point2<S>> for Aabb2<S> {
    fn from_iter<T: Iterator<Point2<S>>>(iterator: T) -> Aabb2<S> {
        let mut iterator = iterator;

        let (mut max, mut min) = match iterator.next() {
            Some(m) => (Point2::new(m.x, m.y), Point2::new(m.x, m.y)),
            None => return Aabb2::new(Point2::new(zero(), zero()),
                                      Point2::new(zero(), zero())),
        };

        for point in iterator {
            max.x = max.x.max(point.x);
            max.y = max.y.max(point.y);
            min.x = min.x.min(point.x);
            min.y = min.y.min(point.y);
        }

        Aabb2 {
            min: min,
            max: max
        }
    }
}

#[deriving(Clone, Eq)]
pub struct Aabb3<S> {
    pub min: Point3<S>,
    pub max: Point3<S>,
}

impl<S: PartOrdPrim> Aabb3<S> {
    /// Construct a new axis-aligned bounding box from two points.
    #[inline]
    pub fn new(p1: Point3<S>, p2: Point3<S>) -> Aabb3<S> {
        Aabb3 {
            min: Point3::new(p1.x.min(p2.x),
                             p1.y.min(p2.y),
                             p1.z.min(p2.z)),
            max: Point3::new(p1.x.max(p2.x),
                             p1.y.max(p2.y),
                             p1.z.max(p2.z)),
        }
    }
}

impl<S: PartOrdPrim> Aabb<S, Vector3<S>, Point3<S>, [S, ..3]> for Aabb3<S> {
    fn new(p1: Point3<S>, p2: Point3<S>) -> Aabb3<S> { Aabb3::new(p1, p2) }
    #[inline] fn min<'a>(&'a self) -> &'a Point3<S> { &self.min }
    #[inline] fn max<'a>(&'a self) -> &'a Point3<S> { &self.max }
}

impl<S: fmt::Show> fmt::Show for Aabb3<S> {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f.buf, "[{} - {}]", self.min, self.max)
    }
}

impl<S: PartOrdPrim> FromIterator<Point3<S>> for Aabb3<S> {
    fn from_iter<T: Iterator<Point3<S>>>(iterator: T) -> Aabb3<S> {
        let mut iterator = iterator;

        let (mut max, mut min) = match iterator.next() {
            Some(m) => (Point3::new(m.x, m.y, m.z), Point3::new(m.x, m.y, m.z)),
            None => return Aabb3::new(Point3::new(zero(), zero(), zero()),
                                      Point3::new(zero(), zero(), zero())),
        };

        for point in iterator {
            max.x = max.x.max(point.x);
            max.y = max.y.max(point.y);
            max.z = max.z.max(point.z);
            min.x = min.x.min(point.x);
            min.y = min.y.min(point.y);
            min.z = min.z.min(point.z);
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

impl<S: PartOrdPrim> Center<Point2<S>> for Aabb2<S> {
    fn center(&self) -> Point2<S> {
        let two = one::<S>() + one::<S>();
        self.min().add_v(&self.dim().div_s(two))
    }
}

impl<S: PartOrdPrim> Center<Point3<S>> for Aabb3<S> {
    fn center(&self) -> Point3<S> {
        let two = one::<S>() + one::<S>();
        self.min().add_v(&self.dim().div_s(two))
    }
}

impl<S: PartOrdPrim> Merge for Aabb2<S> {
    fn merge(&self, other: &Aabb2<S>) -> Aabb2<S> {
        let max = Point2::new(self.max.x.max(other.max.x),
                              self.max.y.max(other.max.y));
        let min = Point2::new(self.min.x.min(other.min.x),
                              self.min.y.min(other.min.y));

        Aabb2 {
            min: min,
            max: max
        }
    }
}

impl<S: PartOrdPrim> Merge for Aabb3<S> {
    fn merge(&self, other: &Aabb3<S>) -> Aabb3<S> {
        let max = Point3::new(self.max.x.max(other.max.x),
                              self.max.y.max(other.max.y),
                              self.max.z.max(other.max.z));
        let min = Point3::new(self.min.x.min(other.min.x),
                              self.min.y.min(other.min.y),
                              self.min.z.min(other.min.z));

        Aabb3 {
            min: min,
            max: max
        }
    }
}