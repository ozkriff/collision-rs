
extern crate cgmath;
extern crate collision;

use collision::Intersects;
use cgmath::aabb::{Aabb2, Aabb3};
use cgmath::point::{Point, Point2, Point3};
use cgmath::vector::{Vector3, Vector2};

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

/*mod sparse {
    use octtree::Cube;
    use octtree::sparse::Sparse;

    use cgmath::point::Point3;

    static size: int = 50;

    #[test]
    fn test_insert_points()
    {
        let mut oct: Sparse<f32, Point3<f32>, (int, int, int)> = Sparse::new(size as f32, 8);

        for x in range(-size, size+1) {
            for y in range(-size, size+1) {
                for z in range(-size, size+1) {
                    let point = Point3::new(x as f32, y as f32, z as f32);
                    oct.insert(point, (x, y, z));
                }
            }
        }

        for x in range(-size, size+1) {
            for y in range(-size, size+1) {
                for z in range(-size, size+1) {
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
        let mut oct: Sparse<f32, Point3<f32>, (int, int, int)> = Sparse::new(size as f32, 8);

        for x in range(-size, size+1) {
            for y in range(-size, size+1) {
                for z in range(-size, size+1) {
                    let point = Point3::new(x as f32, y as f32, z as f32);
                    oct.insert(point, (x, y, z));
                }
            }
        }

        for x in range(-size, size+1) {
            for y in range(-size, size+1) {
                for z in range(-size, size+1) {
                    let point = Point3::new(x as f32, y as f32, z as f32);
                    oct.quary(&point, |_, value| {
                        assert!(*value == (x, y, z));
                    });
                }
            }
        }

        for x in range(0, size+1) {
            for y in range(-size, size+1) {
                for z in range(-size, size+1) {
                    let point = Point3::new(x as f32, y as f32, z as f32);
                    oct.remove(point);
                }
            }
        }

        for x in range(-size, size+1) {
            for y in range(-size, size+1) {
                for z in range(-size, size+1) {
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
    fn test_insert_cube()
    {
        let mut oct: Sparse<f32, Cube<f32>, (int, int, int)> = Sparse::new((8*size) as f32, 8);

        for x in range(-size, size+1) {
            for y in range(-size, size+1) {
                for z in range(-size, size+1) {
                    let point = Cube::new(Point3::new((5*x) as f32, (5*y) as f32, (5*z) as f32), 1f32);
                    oct.insert(point, (x, y, z));
                }
            }
        }

        for x in range(-size, size+1) {
            for y in range(-size, size+1) {
                for z in range(-size, size+1) {
                    let point = Cube::new(Point3::new((5*x) as f32, (5*y) as f32, (5*z) as f32), 1f32);
                    oct.quary(&point, |_, value| {
                        assert!(*value == (x, y, z));
                    });
                }
            }
        }
    }

    #[test]
    fn test_remove_cube()
    {
        let mut oct: Sparse<f32, Cube<f32>, (int, int, int)> = Sparse::new((8*size) as f32, 8);

        for x in range(-size, size+1) {
            for y in range(-size, size+1) {
                for z in range(-size, size+1) {
                    let point = Cube::new(Point3::new((5*x) as f32, (5*y) as f32, (5*z) as f32), 1f32);
                    oct.insert(point, (x, y, z));
                }
            }
        }

        for x in range(-size, size+1) {
            for y in range(-size, size+1) {
                for z in range(-size, size+1) {
                    let point = Cube::new(Point3::new((5*x) as f32, (5*y) as f32, (5*z) as f32), 1f32);
                    oct.quary(&point, |_, value| {
                        assert!(*value == (x, y, z));
                    });
                }
            }
        }

        for x in range(0, size+1) {
            for y in range(-size, size+1) {
                for z in range(-size, size+1) {
                    let point = Cube::new(Point3::new((5*x) as f32, (5*y) as f32, (5*z) as f32), 1f32);
                    oct.remove(point);
                }
            }
        }

        for x in range(-size, size+1) {
            for y in range(-size, size+1) {
                for z in range(-size, size+1) {
                    let point = Cube::new(Point3::new(x as f32, y as f32, z as f32), 0.1f32);
                    oct.quary(&point, |_, value| {
                        let (x, _, _) = *value;
                        assert!(x < 0);
                    });
                }
            }
        }
    }
}*/

mod linear {
    use cgmath::aabb::Aabb3;
    use collision::octtree::Linear;

    use cgmath::point::Point3;

    static size: int = 3;

    #[test]
    fn test_insert_points() {
        let mut oct: Linear<f32, Point3<f32>, (int, int, int)> = Linear::new(size as f32, 3);

        for x in range(-size, size) {
            for y in range(-size, size) {
                for z in range(-size, size) {
                    let point = Point3::new((x+1) as f32 , (y+1) as f32 , (z+1) as f32 );
                    oct.insert(point, (x, y, z));
                }
            }
        }

        for x in range(-size, size) {
            for y in range(-size, size) {
                for z in range(-size, size) {
                    let point = Point3::new((x+1) as f32 , (y+1) as f32 , (z+1) as f32 );
                    oct.quary(&point, |_, value| {
                        assert!((x, y, z) == *value);
                    });
                }
            }
        }
    }

    #[test]
    fn test_insert_cube() {
        let mut oct: Linear<f32, Aabb3<f32>, (int, int, int)> = Linear::new((5*size) as f32, 3);

        for x in range(-size, size+1) {
            for y in range(-size, size+1) {
                for z in range(-size, size+1) {
                    let point = Aabb3::new(Point3::new((5*x) as f32, (5*y) as f32, (5*z) as f32),
                                           Point3::new((5*x+1) as f32, (5*y+1) as f32, (5*z+1) as f32));
                    oct.insert(point, (x, y, z));
                }
            }
        }

        for x in range(-size, size+1) {
            for y in range(-size, size+1) {
                for z in range(-size, size+1) {
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