
use std::num::{zero, one};

use cgmath::point::{Point, Point3};
use cgmath::vector::{EuclideanVector, Vector};
use cgmath::partial_ord::{PartOrdPrim, PartOrdFloat};

struct Sphere<S> {
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

impl<S: PartOrdPrim+PartOrdFloat<S>> FromIterator<Point3<S>> for Sphere<S> {
    fn from_iter<T: Iterator<Point3<S>>>(iterator: T) -> Sphere<S> {
        let mut iterator = iterator;

        let (mut max, mut min) = match iterator.next() {
            Some(m) => (Point3::new(m.x, m.y, m.z), Point3::new(m.x, m.y, m.z)),
            None => return Sphere::new(Point3::new(zero(), zero(), zero()), zero()),
        };

        for point in iterator {
            max.x = max.x.max(point.x);
            max.y = max.y.max(point.y);
            max.z = max.z.max(point.z);
            min.x = min.x.min(point.x);
            min.y = min.y.min(point.y);
            min.z = min.z.min(point.z);
        }

        let one: S = one();
        let two: S = one + one;
        let cross = max.sub_p(&min).div_s(two);
        let radius = cross.length();

        Sphere {
            center: min.add_v(&cross),
            radius: radius
        }
    }
}