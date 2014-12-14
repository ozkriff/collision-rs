
use std::fmt;
use std::num::Float;
use std::default::Default;

use cgmath::{Point, Point3, Point2};
use cgmath::{EuclideanVector, Vector, Vector3, Vector2};
use cgmath::{BaseNum, BaseFloat};
use cgmath::ApproxEq;

use {Max, Min, Center, Merge, Intersects, CheckRange2, CheckRange3};

#[deriving(PartialEq, Clone, Encodable, Decodable, Copy)]
pub struct Sphere<S> {
    pub center: Point3<S>,
    pub radius: S
}

impl<S> Sphere<S> {
    pub fn new(center: Point3<S>, size: S) -> Sphere<S> {
        Sphere {
            center: center,
            radius: size
        }
    }
}

impl<S: BaseNum+BaseFloat+fmt::Show> Merge for Sphere<S> {
    fn merge(&self, other: &Sphere<S>) -> Sphere<S> {
        let diff = other.center.sub_p(&self.center);
        let dist = diff.length();

        if dist + self.radius < other.radius {
            *other
        } else if dist + other.radius < self.radius {
            *self
        } else {
            let one: S = Float::one();
            let rm = (dist+self.radius+other.radius) / (one + one);
            let u = diff.normalize();
            let cm = self.center.add_v(&u.mul_s(rm - self.radius));
            Sphere{
                center: cm,
                radius: rm
            }
        }
    }
}

impl<S: Clone> Center<Point3<S>> for Sphere<S> {
    fn center(&self) -> Point3<S> {
        self.center.clone()
    }
}

impl<S: BaseNum+BaseFloat> Intersects<Sphere<S>> for Sphere<S> {
    fn intersect(&self, other: &Sphere<S>) -> bool {
        let diff = self.center.sub_p(&other.center);
        let dist = diff.length();

        dist < self.radius + other.radius
    }
}

impl<S: BaseNum+BaseFloat> Default for Sphere<S> {
    fn default() -> Sphere<S> {
        Sphere {
            center: Point::origin(),
            radius: Float::zero()
        }
    }
}

impl<S: BaseNum+BaseFloat> Max<Point3<S>> for Sphere<S> {
    fn max(&self) -> Point3<S> {
        let unit = Vector3::new(self.radius, self.radius, self.radius);
        self.center.add_v(&unit)
    }
}

impl<S: BaseNum+BaseFloat> Min<Point3<S>> for Sphere<S> {
    fn min(&self) -> Point3<S> {
        let unit = Vector3::new(-self.radius, -self.radius, -self.radius);
        self.center.add_v(&unit)
    }
}

impl<S: BaseNum+BaseFloat> FromIterator<Point3<S>> for Sphere<S> {
    fn from_iter<T: Iterator<Point3<S>>>(iterator: T) -> Sphere<S> {
        let mut iterator = iterator;

        let (mut max, mut min) = match iterator.next() {
            Some(m) => (Point3::new(m.x, m.y, m.z), Point3::new(m.x, m.y, m.z)),
            None => return Sphere::new(Point3::new(Float::zero(), Float::zero(), Float::zero()), Float::zero()),
        };

        for point in iterator {
            max.x = max.x.max(point.x);
            max.y = max.y.max(point.y);
            max.z = max.z.max(point.z);
            min.x = min.x.min(point.x);
            min.y = min.y.min(point.y);
            min.z = min.z.min(point.z);
        }

        let one: S = Float::one();
        let two: S = one + one;
        let cross = max.sub_p(&min).div_s(two);
        let radius = cross.length();

        Sphere {
            center: min.add_v(&cross),
            radius: radius
        }
    }
}

impl<S: fmt::Show+BaseNum> fmt::Show for Sphere<S> {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "[{} - {}]", self.center, self.radius)
    }
}

impl<S: Float+ApproxEq<S>> CheckRange2<S> for Sphere<S> {
    fn check_x(&self, center: S, _: S) -> (bool, bool) {
        (self.center.x - self.radius <= center,
         self.center.x + self.radius > center)
    }

    fn check_y(&self, center: S, _: S) -> (bool, bool) {
        (self.center.y - self.radius <= center,
         self.center.y + self.radius > center)
    }
}

impl<S: Float+ApproxEq<S>> CheckRange3<S> for Sphere<S> {
    fn check_z(&self, center: S, _: S) -> (bool, bool) {
        (self.center.z - self.radius <= center,
         self.center.z + self.radius > center)
    }
}


#[deriving(PartialEq, Clone, Encodable, Decodable, Copy)]
pub struct Circle<S> {
    pub center: Point2<S>,
    pub radius: S
}

impl<S> Circle<S> {
    pub fn new(center: Point2<S>, size: S) -> Circle<S> {
        Circle {
            center: center,
            radius: size
        }
    }
}

impl<S: BaseNum+BaseFloat+fmt::Show> Merge for Circle<S> {
    fn merge(&self, other: &Circle<S>) -> Circle<S> {
        let diff = other.center.sub_p(&self.center);
        let dist = diff.length();

        if dist + self.radius < other.radius {
            *other
        } else if dist + other.radius < self.radius {
            *self
        } else {
            let one: S = Float::one();
            let rm = (dist+self.radius+other.radius) / (one + one);
            let u = diff.normalize();
            let cm = self.center.add_v(&u.mul_s(rm - self.radius));
            Circle {
                center: cm,
                radius: rm
            }
        }
    }
}

impl<S: Clone> Center<Point2<S>> for Circle<S> {
    fn center(&self) -> Point2<S> {
        self.center.clone()
    }
}

impl<S: BaseNum+BaseFloat> Intersects<Circle<S>> for Circle<S> {
    fn intersect(&self, other: &Circle<S>) -> bool {
        let diff = self.center.sub_p(&other.center);
        let dist = diff.length();

        dist < self.radius + other.radius
    }
}

impl<S: BaseNum+BaseFloat> Default for Circle<S> {
    fn default() -> Circle<S> {
        Circle {
            center: Point::origin(),
            radius: Float::zero()
        }
    }
}

impl<S: BaseNum+BaseFloat> Max<Point2<S>> for Circle<S> {
    fn max(&self) -> Point2<S> {
        let unit = Vector2::new(self.radius, self.radius);
        self.center.add_v(&unit)
    }
}

impl<S: BaseNum+BaseFloat> Min<Point2<S>> for Circle<S> {
    fn min(&self) -> Point2<S> {
        let unit = Vector2::new(-self.radius, -self.radius);
        self.center.add_v(&unit)
    }
}

impl<S: BaseNum+BaseFloat> FromIterator<Point2<S>> for Circle<S> {
    fn from_iter<T: Iterator<Point2<S>>>(iterator: T) -> Circle<S> {
        let mut iterator = iterator;

        let (mut max, mut min) = match iterator.next() {
            Some(m) => (Point2::new(m.x, m.y), Point2::new(m.x, m.y)),
            None => return Circle::new(Point2::new(Float::zero(), Float::zero()), Float::zero()),
        };

        for point in iterator {
            max.x = max.x.max(point.x);
            max.y = max.y.max(point.y);
            min.x = min.x.min(point.x);
            min.y = min.y.min(point.y);
        }

        let one: S = Float::one();
        let two: S = one + one;
        let cross = max.sub_p(&min).div_s(two);
        let radius = cross.length();

        Circle {
            center: min.add_v(&cross),
            radius: radius
        }
    }
}

impl<S: fmt::Show+BaseNum> fmt::Show for Circle<S> {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "[{} - {}]", self.center, self.radius)
    }
}

impl<S: Float+ApproxEq<S>> CheckRange2<S> for Circle<S> {
    fn check_x(&self, center: S, _: S) -> (bool, bool) {
        (self.center.x - self.radius <= center,
         self.center.x + self.radius > center)
    }

    fn check_y(&self, center: S, _: S) -> (bool, bool) {
        (self.center.y - self.radius <= center,
         self.center.y + self.radius > center)
    }
}
