#![allow(unstable)]

extern crate collision;
extern crate cgmath;

use collision::Intersects;
use collision::aabb::{Aabb2, Aabb3};

use cgmath::{Point, Point2, Point3};
use cgmath::{Vector3, Vector2};

#[test]
fn test_aabb2_collide() {
    let a = Aabb2::new(Point2::new(0f32, 0.),
                       Point2::new(1f32, 1.));

    let table = [(Point2::new(0.9f32, 0.9f32), true),
                 (Point2::new(0.9f32, 1.1f32), false),
                 (Point2::new(1.1f32, 0.9f32), false),
                 (Point2::new(1.1f32, 1.1f32), false),
                 (Point2::new(-1.1f32, -1.1f32), false),
                 (Point2::new(-0.9f32, -1.1f32), false),
                 (Point2::new(-1.1f32, -0.9f32), false),
                 (Point2::new(-0.9f32, -0.9f32), true)];

    for &(point, should_intersect) in table.iter() {
        let b = Aabb2::new(point, point.add_v(&Vector2::new(1f32, 1f32)));
        assert!(a.intersect(&b) == should_intersect);
    }
}

#[test]
fn test_aabb3_collide() {
    let a = Aabb3::new(Point3::new(0f32, 0., 0.),
                       Point3::new(1f32, 1., 1.));

    let table = [(Point3::new(0.9f32, 0.9f32, 0.9f32), true),
                 (Point3::new(0.9f32, 1.1f32, 0.9f32), false),
                 (Point3::new(1.1f32, 0.9f32, 0.9f32), false),
                 (Point3::new(1.1f32, 1.1f32, 0.9f32), false),
                 (Point3::new(0.9f32, 0.9f32, 1.1f32), false),
                 (Point3::new(0.9f32, 1.1f32, 1.1f32), false),
                 (Point3::new(1.1f32, 0.9f32, 1.1f32), false),
                 (Point3::new(1.1f32, 1.1f32, 1.1f32), false),
                 (Point3::new(-1.1f32, -1.1f32, -1.1f32), false),
                 (Point3::new(-0.9f32, -1.1f32, -1.1f32), false),
                 (Point3::new(-1.1f32, -0.9f32, -1.1f32), false),
                 (Point3::new(-0.9f32, -0.9f32, -1.1f32), false),
                 (Point3::new(-1.1f32, -1.1f32, -0.9f32), false),
                 (Point3::new(-0.9f32, -1.1f32, -0.9f32), false),
                 (Point3::new(-1.1f32, -0.9f32, -0.9f32), false),
                 (Point3::new(-0.9f32, -0.9f32, -0.9f32), true)];

    for &(point, should_intersect) in table.iter() {
        let b = Aabb3::new(point, point.add_v(&Vector3::new(1f32, 1f32, 1f32)));
        assert!(a.intersect(&b) == should_intersect);
    }
}

#[test]
fn test_point2_collide() {
    let a = Point2::new(1f32, 1f32);

    let table = [(Point2::new(1f32, 1f32), true),
                 (Point2::new(0f32, 1f32), false),
                 (Point2::new(1f32, 0f32), false),
                 (Point2::new(0f32, 0f32), false)];

    for &(point, should_intersect) in table.iter() {
        assert!(a.intersect(&point) == should_intersect);
    }
}

#[test]
fn test_point3_collide() {
    let a = Point3::new(1f32, 1f32, 1f32);

    let table = [(Point3::new(1f32, 1f32, 1f32), true),
                 (Point3::new(0f32, 1f32, 1f32), false),
                 (Point3::new(1f32, 0f32, 1f32), false),
                 (Point3::new(0f32, 0f32, 1f32), false),
                 (Point3::new(1f32, 1f32, 0f32), false),
                 (Point3::new(0f32, 1f32, 0f32), false),
                 (Point3::new(1f32, 0f32, 0f32), false),
                 (Point3::new(0f32, 0f32, 0f32), false)];

    for &(point, should_intersect) in table.iter() {
        assert!(a.intersect(&point) == should_intersect);
    }
}
