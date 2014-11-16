use super::Intersects;
use super::aabb::{Aabb2, Aabb3};

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

mod sparse {
    use super::super::aabb::Aabb3;
    use super::super::octtree::Sparse;

    use cgmath::Point3;

    const SIZE: int = 5;

    #[test]
    fn test_insert_points()
    {
        let mut oct: Sparse<f32, Point3<f32>, (int, int, int)> = Sparse::new(SIZE as f32, 8);

        for x in range(-SIZE, SIZE+1) {
            for y in range(-SIZE, SIZE+1) {
                for z in range(-SIZE, SIZE+1) {
                    let point = Point3::new(x as f32, y as f32, z as f32);
                    oct.insert(point, (x, y, z));
                }
            }
        }

        for x in range(-SIZE, SIZE+1) {
            for y in range(-SIZE, SIZE+1) {
                for z in range(-SIZE, SIZE+1) {
                    let point = Point3::new(x as f32, y as f32, z as f32);
                    oct.quary(&point, |_, value| {
                        assert!(*value == (x, y, z));
                    });
                }
            }
        }
    }

    #[test]
    fn test_remove_points()
    {
        let mut oct: Sparse<f32, Point3<f32>, (int, int, int)> = Sparse::new(SIZE as f32, 8);

        for x in range(-SIZE, SIZE+1) {
            for y in range(-SIZE, SIZE+1) {
                for z in range(-SIZE, SIZE+1) {
                    let point = Point3::new(x as f32, y as f32, z as f32);
                    oct.insert(point, (x, y, z));
                }
            }
        }

        for x in range(-SIZE, SIZE+1) {
            for y in range(-SIZE, SIZE+1) {
                for z in range(-SIZE, SIZE+1) {
                    let point = Point3::new(x as f32, y as f32, z as f32);
                    oct.quary(&point, |_, value| {
                        assert!(*value == (x, y, z));
                    });
                }
            }
        }

        for x in range(0, SIZE+1) {
            for y in range(-SIZE, SIZE+1) {
                for z in range(-SIZE, SIZE+1) {
                    let point = Point3::new(x as f32, y as f32, z as f32);
                    oct.remove(point);
                }
            }
        }

        for x in range(-SIZE, SIZE+1) {
            for y in range(-SIZE, SIZE+1) {
                for z in range(-SIZE, SIZE+1) {
                    let point = Point3::new(x as f32, y as f32, z as f32);
                    oct.quary(&point, |_, value| {
                        let (x, _, _) = *value;
                        assert!(x < 0);
                    });
                }
            }
        }
    }

    #[test]
    fn test_insert_aabb()
    {
        let mut oct: Sparse<f32, Aabb3<f32>, (int, int, int)> = Sparse::new((8*SIZE) as f32, 8);

        for x in range(-SIZE, SIZE+1) {
            for y in range(-SIZE, SIZE+1) {
                for z in range(-SIZE, SIZE+1) {
                    let point = Aabb3::new(Point3::new((5*x-1) as f32, (5*y-1) as f32, (5*z-1) as f32),
                                           Point3::new((5*x+1) as f32, (5*y+1) as f32, (5*z+1) as f32));
                    oct.insert(point, (x, y, z));
                }
            }
        }

        for x in range(-SIZE, SIZE+1) {
            for y in range(-SIZE, SIZE+1) {
                for z in range(-SIZE, SIZE+1) {
                    let point = Aabb3::new(Point3::new((5*x-1) as f32, (5*y-1) as f32, (5*z-1) as f32),
                                           Point3::new((5*x+1) as f32, (5*y+1) as f32, (5*z+1) as f32));
                    oct.quary(&point, |_, value| {
                        assert!(*value == (x, y, z));
                    });
                }
            }
        }
    }

    #[test]
    fn test_remove_aabb()
    {
        let mut oct: Sparse<f32, Aabb3<f32>, (int, int, int)> = Sparse::new((8*SIZE) as f32, 8);

        for x in range(-SIZE, SIZE+1) {
            for y in range(-SIZE, SIZE+1) {
                for z in range(-SIZE, SIZE+1) {
                    let point = Aabb3::new(Point3::new((5*x-1) as f32, (5*y-1) as f32, (5*z-1) as f32),
                                           Point3::new((5*x+1) as f32, (5*y+1) as f32, (5*z+1) as f32));
                    oct.insert(point, (x, y, z));
                }
            }
        }

        for x in range(-SIZE, SIZE+1) {
            for y in range(-SIZE, SIZE+1) {
                for z in range(-SIZE, SIZE+1) {
                    let point = Aabb3::new(Point3::new((5*x-1) as f32, (5*y-1) as f32, (5*z-1) as f32),
                                           Point3::new((5*x+1) as f32, (5*y+1) as f32, (5*z+1) as f32));
                    oct.quary(&point, |_, value| {
                        assert!(*value == (x, y, z));
                    });
                }
            }
        }

        for x in range(0, SIZE+1) {
            for y in range(-SIZE, SIZE+1) {
                for z in range(-SIZE, SIZE+1) {
                    let point = Aabb3::new(Point3::new((5*x-1) as f32, (5*y-1) as f32, (5*z-1) as f32),
                                           Point3::new((5*x+1) as f32, (5*y+1) as f32, (5*z+1) as f32));
                    oct.remove(point);
                }
            }
        }

        for x in range(-SIZE, SIZE+1) {
            for y in range(-SIZE, SIZE+1) {
                for z in range(-SIZE, SIZE+1) {
                    let point = Aabb3::new(Point3::new(x as f32, y as f32, z as f32),
                                           Point3::new(x as f32 + 0.1, y as f32 + 0.1, z as f32 + 0.1));
                    oct.quary(&point, |_, value| {
                        let (x, _, _) = *value;
                        assert!(x < 0);
                    });
                }
            }
        }
    }
}

mod linear {
    use super::super::aabb::Aabb3;
    use super::super::octtree::Linear;

    use cgmath::Point3;

    static SIZE: int = 3;

    #[test]
    fn test_insert_points() {
        let mut oct: Linear<f32, Point3<f32>, (int, int, int)> = Linear::new(SIZE as f32, 3);

        for x in range(-SIZE, SIZE) {
            for y in range(-SIZE, SIZE) {
                for z in range(-SIZE, SIZE) {
                    let point = Point3::new((x+1) as f32 , (y+1) as f32 , (z+1) as f32 );
                    oct.insert(point, (x, y, z));
                }
            }
        }

        for x in range(-SIZE, SIZE) {
            for y in range(-SIZE, SIZE) {
                for z in range(-SIZE, SIZE) {
                    let point = Point3::new((x+1) as f32 , (y+1) as f32 , (z+1) as f32 );
                    oct.quary(&point, |_, value| {
                        assert!((x, y, z) == *value);
                    });
                }
            }
        }
    }

    #[test]
    fn test_insert_aabb() {
        let mut oct: Linear<f32, Aabb3<f32>, (int, int, int)> = Linear::new((5*SIZE) as f32, 3);

        for x in range(-SIZE, SIZE+1) {
            for y in range(-SIZE, SIZE+1) {
                for z in range(-SIZE, SIZE+1) {
                    let point = Aabb3::new(Point3::new((5*x) as f32, (5*y) as f32, (5*z) as f32),
                                           Point3::new((5*x+1) as f32, (5*y+1) as f32, (5*z+1) as f32));
                    oct.insert(point, (x, y, z));
                }
            }
        }

        for x in range(-SIZE, SIZE+1) {
            for y in range(-SIZE, SIZE+1) {
                for z in range(-SIZE, SIZE+1) {
                    let point = Aabb3::new(Point3::new((5*x) as f32, (5*y) as f32, (5*z) as f32),
                                           Point3::new((5*x+1) as f32, (5*y+1) as f32, (5*z+1) as f32));
                    oct.quary(&point, |_, value| {
                        assert!(*value == (x, y, z));
                    });
                }
            }
        }
    }
}

mod bvh {
    use std::num::Float;
    use std::collections::HashSet;
    use std::iter::range_inclusive;

    use cgmath::Point3;
    use cgmath::Vector3;

    use super::super::sphere::Sphere;
    use super::super::aabb::Aabb3;
    use super::super::bvh::{BvhBuilder, ToMorton};
    use super::super::Intersects;

    use test::Bencher;

    static SIZE: int = 10;

    #[test]
    fn test_to_morton3() {
        let base = Point3::new(0f32, 0., 0.);
        let scale = Vector3::new(1023f32, 1023., 1023.);
        let full = Point3::new(1f32, 1f32, 1f32);
        let zero = Point3::new(0f32, 0f32, 0f32);
        let half = Point3::new(0.5f32, 0.5f32, 0.5f32);
        let x_full = Point3::new(1f32, 0f32, 0f32);
        let y_full = Point3::new(0f32, 1f32, 0f32);
        let z_full = Point3::new(0f32, 0f32, 1f32);

        assert!(0b111_111_111_111_111_111_111_111_111_111 == full.to_morton(&base, &scale));
        assert!(0b000_000_000_000_000_000_000_000_000_000 == zero.to_morton(&base, &scale));
        assert!(0b000_111_111_111_111_111_111_111_111_111 == half.to_morton(&base, &scale));
        assert!(0b001_001_001_001_001_001_001_001_001_001 == z_full.to_morton(&base, &scale));
        assert!(0b010_010_010_010_010_010_010_010_010_010 == y_full.to_morton(&base, &scale));
        assert!(0b100_100_100_100_100_100_100_100_100_100 == x_full.to_morton(&base, &scale));
    }

    #[test]
    fn test_aabb_bvh_build() {
        let mut builder = BvhBuilder::new();
        for x in range_inclusive(-SIZE, SIZE) {
            for y in range_inclusive(-SIZE, SIZE) {
                for z in range_inclusive(-SIZE, SIZE) {
                    let xf = x as f32;
                    let yf = y as f32;
                    let zf = z as f32;
                    let aabb = Aabb3::new(Point3::new(xf-0.25, yf-0.25, zf-0.25),
                                          Point3::new(zf+0.25, yf+0.25, zf+0.25));
                    builder.add(aabb, (x, y, x));
                }
            }
        }

        let _ = builder.build();
    }

    #[test]
    fn test_aabb_bvh_collide_all() {
        let mut set = HashSet::new();

        let mut builder = BvhBuilder::new();
        for x in range_inclusive(-SIZE, SIZE) {
            for y in range_inclusive(-SIZE, SIZE) {
                for z in range_inclusive(-SIZE, SIZE) {
                    let xf = x as f32;
                    let yf = y as f32;
                    let zf = z as f32;
                    let aabb = Aabb3::new(Point3::new(xf-0.25, yf-0.25, zf-0.25),
                                          Point3::new(zf+0.25, yf+0.25, zf+0.25));
                    builder.add(aabb, (x, y, z));
                    set.insert((x, y, z));
                }
            }
        }

        let aabb = Aabb3::new(Point3::new(-SIZE as f32, -SIZE as f32, -SIZE as f32),
                              Point3::new(SIZE as f32, SIZE as f32, SIZE as f32));

        let bvh = builder.build();
        for (_, dat) in bvh.collision_iter(&aabb) {
            assert!(set.remove(dat));
        }

        assert!(set.len() == 0);
    }

    #[test]
    fn test_aabb_bvh_collide_half() {
        let mut set = HashSet::new();
        let mut builder = BvhBuilder::new();
 
        let check = Aabb3::new(Point3::new(0. as f32, 0. as f32, 0. as f32),
                               Point3::new(SIZE as f32, SIZE as f32, SIZE as f32));

        for x in range_inclusive(-SIZE, SIZE) {
            for y in range_inclusive(-SIZE, SIZE) {
                for z in range_inclusive(-SIZE, SIZE) {
                    let xf = x as f32;
                    let yf = y as f32;
                    let zf = z as f32;
                    let aabb = Aabb3::new(Point3::new(xf-0.25, yf-0.25, zf-0.25),
                                          Point3::new(zf+0.25, yf+0.25, zf+0.25));
                    builder.add(aabb, (x, y, z));
                    if aabb.intersect(&check) {
                        set.insert((x, y, z));
                    }
                }
            }
        }

        let bvh = builder.build();
        for (_, dat) in bvh.collision_iter(&check) {
            assert!(set.remove(dat));
        }

        assert!(set.len() == 0);
    }

    #[test]
    fn test_sphere_bvh_build() {
        let mut builder = BvhBuilder::new();
        for x in range_inclusive(-SIZE, SIZE) {
            for y in range_inclusive(-SIZE, SIZE) {
                for z in range_inclusive(-SIZE, SIZE) {
                    let xf = x as f32;
                    let yf = y as f32;
                    let zf = z as f32;
                    let sphere = Sphere::new(Point3::new(xf, yf, zf), 0.25);
                    builder.add(sphere, (x, y, x));
                }
            }
        }

        let _ = builder.build();
    }

    #[test]
    fn test_sphere_bvh_collide_all() {
        let mut set = HashSet::new();

        let mut builder = BvhBuilder::new();
        for x in range_inclusive(-SIZE, SIZE) {
            for y in range_inclusive(-SIZE, SIZE) {
                for z in range_inclusive(-SIZE, SIZE) {
                    let xf = x as f32;
                    let yf = y as f32;
                    let zf = z as f32;
                    let sphere = Sphere::new(Point3::new(xf, yf, zf), 0.25);
                    builder.add(sphere, (x, y, z));
                    set.insert((x, y, z));
                }
            }
        }

        let sphere = Sphere::new(Point3::new(0f32, 0f32, 0f32), ((SIZE*SIZE + SIZE*SIZE + SIZE*SIZE) as f32).sqrt());

        let bvh = builder.build();
        for (_, dat) in bvh.collision_iter(&sphere) {
            assert!(set.remove(dat));
        }

        assert!(set.len() == 0);
    }

    #[test]
    fn test_sphere_bvh_collide_half() {
        let mut set = HashSet::new();
        let mut builder = BvhBuilder::new();

        let check = Sphere::new(Point3::new(0f32, 0f32, 0f32),
                                0.5 * ((SIZE*SIZE + SIZE*SIZE + SIZE*SIZE) as f32).sqrt());

        for x in range_inclusive(-SIZE, SIZE) {
            for y in range_inclusive(-SIZE, SIZE) {
                for z in range_inclusive(-SIZE, SIZE) {
                    let xf = x as f32;
                    let yf = y as f32;
                    let zf = z as f32;
                    let sphere = Sphere::new(Point3::new(xf, yf, zf), 0.25);
                    builder.add(sphere, (x, y, z));
                    if sphere.intersect(&check) {
                        set.insert((x, y, z));
                    }
                }
            }
        }

        let bvh = builder.build();
        for (_, dat) in bvh.collision_iter(&check) {
            assert!(set.remove(dat));
        }

        assert!(set.len() == 0);
    }

    #[bench]
    fn bench_aabb_build(bench: &mut Bencher) {
        bench.iter(|| {
            let mut builder = BvhBuilder::new();
            for x in range_inclusive(-SIZE, SIZE) {
                for y in range_inclusive(-SIZE, SIZE) {
                    for z in range_inclusive(-SIZE, SIZE) {
                        let xf = x as f32;
                        let yf = y as f32;
                        let zf = z as f32;
                        let aabb = Aabb3::new(Point3::new(xf-0.25, yf-0.25, zf-0.25),
                                              Point3::new(zf+0.25, yf+0.25, zf+0.25));
                        builder.add(aabb, (x, y, z));
                    }
                }
            }
            builder.build()
        });
    }

    #[bench]
    fn bench_aabb_build_add_only(bench: &mut Bencher) {
        bench.iter(|| {
            let mut builder = BvhBuilder::new();
            for x in range_inclusive(-SIZE, SIZE) {
                for y in range_inclusive(-SIZE, SIZE) {
                    for z in range_inclusive(-SIZE, SIZE) {
                        let xf = x as f32;
                        let yf = y as f32;
                        let zf = z as f32;
                        let aabb = Aabb3::new(Point3::new(xf-0.25, yf-0.25, zf-0.25),
                                              Point3::new(zf+0.25, yf+0.25, zf+0.25));
                        builder.add(aabb, (x, y, z));
                    }
                }
            }
            builder
        });
    }

    #[bench]
    fn bench_aabb_iter_half(bench: &mut Bencher) {
        let mut builder = BvhBuilder::new();
        for x in range_inclusive(-SIZE, SIZE) {
            for y in range_inclusive(-SIZE, SIZE) {
                for z in range_inclusive(-SIZE, SIZE) {
                    let xf = x as f32;
                    let yf = y as f32;
                    let zf = z as f32;
                    let aabb = Aabb3::new(Point3::new(xf-0.25, yf-0.25, zf-0.25),
                                          Point3::new(zf+0.25, yf+0.25, zf+0.25));
                    builder.add(aabb, (x, y, z));
                }
            }
        }

        let bvh = builder.build();
        let check = Aabb3::new(Point3::new(0. as f32, 0. as f32, 0. as f32),
                               Point3::new(SIZE as f32, SIZE as f32, SIZE as f32));
        
        bench.iter(|| {
            let mut sum = 0i;
            for (_, _) in bvh.collision_iter(&check) {
                sum += 1;
            }
            sum
        });
    }

    #[bench]
    fn bench_aabb_iter_one(bench: &mut Bencher) {
        let mut builder = BvhBuilder::new();
        for x in range_inclusive(-SIZE, SIZE) {
            for y in range_inclusive(-SIZE, SIZE) {
                for z in range_inclusive(-SIZE, SIZE) {
                    let xf = x as f32;
                    let yf = y as f32;
                    let zf = z as f32;
                    let aabb = Aabb3::new(Point3::new(xf-0.25, yf-0.25, zf-0.25),
                                          Point3::new(zf+0.25, yf+0.25, zf+0.25));
                    builder.add(aabb, (x, y, z));
                }
            }
        }

        let bvh = builder.build();
        let check = Aabb3::new(Point3::new(0. as f32, 0. as f32, 0. as f32),
                               Point3::new(1. as f32, 1. as f32, 1. as f32));
        
        bench.iter(|| {
            let mut sum = 0i;
            for (_, _) in bvh.collision_iter(&check) {
                sum += 1;
            }
            sum
        });
    }

    #[bench]
    fn bench_sphere_build(bench: &mut Bencher) {
        bench.iter(|| {
            let mut builder = BvhBuilder::new();
            for x in range_inclusive(-SIZE, SIZE) {
                for y in range_inclusive(-SIZE, SIZE) {
                    for z in range_inclusive(-SIZE, SIZE) {
                        let xf = x as f32;
                        let yf = y as f32;
                        let zf = z as f32;
                        let sphere = Sphere::new(Point3::new(xf, yf, zf), 0.25);
                        builder.add(sphere, (x, y, z));
                    }
                }
            }
            builder.build()
        });
    }

    #[bench]
    fn bench_sphere_build_add_only(bench: &mut Bencher) {
        bench.iter(|| {
            let mut builder = BvhBuilder::new();
            for x in range_inclusive(-SIZE, SIZE) {
                for y in range_inclusive(-SIZE, SIZE) {
                    for z in range_inclusive(-SIZE, SIZE) {
                        let xf = x as f32;
                        let yf = y as f32;
                        let zf = z as f32;
                        let sphere = Sphere::new(Point3::new(xf, yf, zf), 0.25);
                        builder.add(sphere, (x, y, z));
                    }
                }
            }
            builder
        });
    }

    #[bench]
    fn bench_sphere_iter_half(bench: &mut Bencher) {
        let mut builder = BvhBuilder::new();
        for x in range_inclusive(-SIZE, SIZE) {
            for y in range_inclusive(-SIZE, SIZE) {
                for z in range_inclusive(-SIZE, SIZE) {
                    let xf = x as f32;
                    let yf = y as f32;
                    let zf = z as f32;
                    let sphere = Sphere::new(Point3::new(xf, yf, zf), 0.25);
                    builder.add(sphere, (x, y, z));
                }
            }
        }

        let bvh = builder.build();
        let check = Sphere::new(Point3::new(0. as f32, 0. as f32, 0. as f32), SIZE as f32);
        
        bench.iter(|| {
            let mut sum = 0i;
            for (_, _) in bvh.collision_iter(&check) {
                sum += 1;
            }
            sum
        });
    }

    #[bench]
    fn bench_sphere_iter_one(bench: &mut Bencher) {
        let mut builder = BvhBuilder::new();
        for x in range_inclusive(-SIZE, SIZE) {
            for y in range_inclusive(-SIZE, SIZE) {
                for z in range_inclusive(-SIZE, SIZE) {
                    let xf = x as f32;
                    let yf = y as f32;
                    let zf = z as f32;
                    let sphere = Sphere::new(Point3::new(xf, yf, zf), 0.25);
                    builder.add(sphere, (x, y, z));
                }
            }
        }

        let bvh = builder.build();
        let check = Sphere::new(Point3::new(0f32, 0f32, 0f32), 0.25);
        
        bench.iter(|| {
            let mut sum = 0i;
            for (_, _) in bvh.collision_iter(&check) {
                sum += 1;
            }
            sum
        });
    }
}
