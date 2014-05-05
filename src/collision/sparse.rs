use std::num::{FromPrimitive, zero};

use sync::Arc;
use cgmath::point::Point3;

use CheckRange3;
use Intersects;

struct Branch<S, K, V> {
    children: [Node<S, K, V>, ..8]
}

enum Node<S, K, V> {
    Empty,
    Data(K, V),
    Collide(Vec<(K, V)>),
    Child(Arc<Branch<S, K, V>>),
}

pub struct Sparse<S, K, V> {
    root: Node<S, K, V>,
    scale: S,
    max_depth: uint
}

impl<S: Float+FromPrimitive, K: Clone+Send+Share+CheckRange3<S>+Intersects<K>+Eq, V: Clone+Send+Share> Branch<S, K, V> {
    fn new() -> Branch<S, K, V> {
        Branch {
            children: [Empty, Empty, Empty, Empty,
                       Empty, Empty, Empty, Empty]
        }
    }

    fn count(&self) -> (int, int) {
        let mut data = 0;
        let mut child = 0;

        for c in self.children.iter() {
            match *c {
                Data(_, _) => data += 1,
                Child(_) => child += 1,
                Collide(_) => child += 1,
                Empty => (),
            }
        }

        (data, child)
    }

    #[inline(always)]
    fn action_mut(&mut self, c: Point3<S>, scale: S, key: &K, cb: |&mut Node<S, K, V>, Point3<S>, S|) {
        let hscale = scale / (FromPrimitive::from_uint(2).unwrap());

        let (lt_x, gt_x) = key.check_x(c.x, scale.clone());
        let (lt_y, gt_y) = key.check_y(c.y, scale.clone());
        let (lt_z, gt_z) = key.check_z(c.z, scale.clone());

        if lt_x && lt_y && lt_z {
            cb(&mut self.children[0],
               Point3::new(c.x - hscale, c.y - hscale, c.z - hscale),
               hscale.clone());
        }
        if lt_x && lt_y && gt_z {
            cb(&mut self.children[1],
                Point3::new(c.x - hscale, c.y - hscale, c.z + hscale),
                hscale.clone());
        }
        if lt_x && gt_y && lt_z {
            cb(&mut self.children[2],
                Point3::new(c.x - hscale, c.y + hscale, c.z - hscale),
                hscale.clone());
        }
        if lt_x && gt_y && gt_z {
            cb(&mut self.children[3],
                Point3::new(c.x - hscale, c.y + hscale, c.z + hscale),
                hscale.clone());
        }
        if gt_x && lt_y && lt_z {
            cb(&mut self.children[4],
                Point3::new(c.x + hscale, c.y - hscale, c.z - hscale),
                hscale.clone());
        }
        if gt_x && lt_y && gt_z {
            cb(&mut self.children[5],
                Point3::new(c.x + hscale, c.y - hscale, c.z + hscale),
                hscale.clone());
        }
        if gt_x && gt_y && lt_z {
            cb(&mut self.children[6],
                Point3::new(c.x + hscale, c.y + hscale, c.z - hscale),
                hscale.clone());
        }
        if gt_x && gt_y && gt_z {
            cb(&mut self.children[7],
                Point3::new(c.x + hscale, c.y + hscale, c.z + hscale),
                hscale.clone());
        }
    }

    #[inline(always)]
    fn action<Q: CheckRange3<S>>(&self, c: Point3<S>, scale: S, key: &Q, cb: |&Node<S, K, V>, Point3<S>, S|) {
        let hscale = scale / (FromPrimitive::from_uint(2).unwrap());

        let (lt_x, gt_x) = key.check_x(c.x, scale.clone());
        let (lt_y, gt_y) = key.check_y(c.y, scale.clone());
        let (lt_z, gt_z) = key.check_z(c.z, scale.clone());

        if lt_x && lt_y && lt_z {
            cb(&self.children[0],
               Point3::new(c.x - hscale, c.y - hscale, c.z - hscale),
               hscale.clone());
        }
        if lt_x && lt_y && gt_z {
            cb(&self.children[1],
                Point3::new(c.x - hscale, c.y - hscale, c.z + hscale),
                hscale.clone());
        }
        if lt_x && gt_y && lt_z {
            cb(&self.children[2],
                Point3::new(c.x - hscale, c.y + hscale, c.z - hscale),
                hscale.clone());
        }
        if lt_x && gt_y && gt_z {
            cb(&self.children[3],
                Point3::new(c.x - hscale, c.y + hscale, c.z + hscale),
                hscale.clone());
        }
        if gt_x && lt_y && lt_z {
            cb(&self.children[4],
                Point3::new(c.x + hscale, c.y - hscale, c.z - hscale),
                hscale.clone());
        }
        if gt_x && lt_y && gt_z {
            cb(&self.children[5],
                Point3::new(c.x + hscale, c.y - hscale, c.z + hscale),
                hscale.clone());
        }
        if gt_x && gt_y && lt_z {
            cb(&self.children[6],
                Point3::new(c.x + hscale, c.y + hscale, c.z - hscale),
                hscale.clone());
        }
        if gt_x && gt_y && gt_z {
            cb(&self.children[7],
                Point3::new(c.x + hscale, c.y + hscale, c.z + hscale),
                hscale.clone());
        }
    }

    fn insert(&mut self, c: Point3<S>, scale: S, depth: uint, key: &K, value: &V) {
        self.action_mut(c, scale, key,
            |next, next_centre, next_scale| {
                next.insert(next_centre, next_scale, depth, key, value);
            }
        );
    }

    fn remove(&mut self, c: Point3<S>, scale: S, key: &K) {
        self.action_mut(c, scale, key,
            |next, next_centre, next_scale| {
                next.remove(next_centre, next_scale, key);
            }
        );
    }

    fn quary<Q: CheckRange3<S>>(&self, c: Point3<S>, scale: S, key: &Q, cb: |&K, &V|) {
        self.action(c, scale, key,
            |next, next_centre, next_scale| {
                next.quary(next_centre, next_scale, key, |k, v| { cb(k, v) });
            }
        );
    }
}

impl<S: Float+FromPrimitive, K: Clone+Send+Share+CheckRange3<S>+Intersects<K>, V: Clone+Send+Share> Clone for Branch<S, K, V> {
    fn clone(&self) -> Branch<S, K, V> {
        Branch {
            children: [
                self.children[0].clone(), self.children[1].clone(),
                self.children[2].clone(), self.children[3].clone(),
                self.children[4].clone(), self.children[5].clone(),
                self.children[6].clone(), self.children[7].clone()
            ]
        }
    }
}

impl<S: Float+FromPrimitive, K: Clone+Send+Share+CheckRange3<S>+Intersects<K>+Eq, V: Clone+Send+Share> Sparse<S, K, V> {
    pub fn new(scale: S, max_depth: uint) -> Sparse<S, K, V> {
        Sparse {
            scale: scale,
            root: Empty,
            max_depth: max_depth
        }
    }

    pub fn insert(&mut self, key: K, value: V) {
        self.root.insert(Point3::new(zero(), zero(), zero()),
                         self.scale.clone(), self.max_depth, &key, &value)
    }

    pub fn remove(&mut self, key: K) {
        self.root.remove(Point3::new(zero(), zero(), zero()),
                         self.scale.clone(), &key)
    }

    pub fn quary<Q: CheckRange3<S>>(&self, key: &Q, cb: |&K, &V|) {
        self.root.quary(Point3::new(zero(), zero(), zero()),
                        self.scale.clone(), key, cb)
    }
}

impl<S: Float+FromPrimitive, K: Clone+Send+Share+CheckRange3<S>+Intersects<K>, V: Clone+Send+Share> Clone for Sparse<S, K, V> {
    fn clone(&self) -> Sparse<S, K, V> {
        Sparse {
            scale: self.scale.clone(),
            root: self.root.clone(),
            max_depth: self.max_depth,
        }
    }
}

impl<S: Float+FromPrimitive, K: Clone+Send+Share+CheckRange3<S>+Intersects<K>, V: Clone+Send+Share> Clone for Node<S, K, V> {
    fn clone(&self) -> Node<S, K, V> {
        match *self {
            Empty => Empty,
            Data(ref key, ref value) => Data(key.clone(), value.clone()),
            Collide(ref arr) => Collide(arr.clone()),
            Child(ref ptr) => Child(ptr.clone())
        }
    }
}

impl<S: Float+FromPrimitive, K: Clone+Send+Share+CheckRange3<S>+Intersects<K>+Eq, V: Clone+Send+Share> Node<S, K, V> {
    #[inline(always)]
    fn insert(&mut self, centre: Point3<S>, scale: S, depth: uint, key: &K, value: &V) {
        let new = match *self {
            Empty => {
                Data(key.clone(), value.clone())
            },
            Data(ref k, ref v) => {
                if depth == 0 || k.intersect(key) {
                    let list = vec!((k.clone(), v.clone()), (key.clone(), value.clone()));
                    Collide(list)
                } else {
                    let mut data = Branch::new();
                    data.insert(centre.clone(), scale.clone(), depth - 1, k, v);
                    data.insert(centre.clone(), scale.clone(), depth - 1, key, value);
                    Child(Arc::new(data))
                }

            },
            Collide(ref mut data) => {
                let mut all_collide = true;
                if depth != 0 {
                    for &(ref k,_) in data.iter() {
                        if !key.intersect(k) {
                            all_collide = false;
                            break;
                        }
                    }
                }

                if all_collide {
                    data.push((key.clone(), value.clone()));
                    return;
                } else {
                    let mut new = Branch::new();
                    for &(ref k, ref v) in data.iter() {
                        new.insert(centre.clone(), scale.clone(), depth - 1, k, v);
                    }
                    new.insert(centre.clone(), scale.clone(), depth - 1, key, value);
                    Child(Arc::new(new))
                }
            },
            Child(ref mut child) => {
                child.make_unique().insert(centre, scale, depth - 1, key, value);
                
                match child.deref().count() {
                    (0, 0) => Empty,
                    (_, _) => return,
                }
            }
        };

        *self = new;
    }

    #[inline(always)]
    fn remove(&mut self, centre: Point3<S>, scale: S, key: &K) {
        let new = match *self {
            Empty => Empty,
            Data(_, _) => {
                Empty
            },
            Collide(ref mut child) => {
                let mut index = None;
                for (i, &(ref k, _)) in child.iter().enumerate() {
                    if *k == *key {
                        index = Some(i);
                        break
                    }
                }

                match index {
                    Some(i) => {
                        child.remove(i);
                    },
                    None => (),
                }
                return;
            }
            Child(ref mut child) => {
                child.make_unique().remove(centre, scale, key);
                return;
            }
        };

        *self = new;
    }

    #[inline(always)]
    fn quary<Q: CheckRange3<S>>(&self, centre: Point3<S>, scale: S, key: &Q, cb: |&K, &V|) {
        match *self {
            Empty => (),
            Data(ref key, ref v) => {
                cb(key, v);
            },
            Collide(ref child) => {
                for &(ref k, ref v) in child.iter() {
                    cb(k, v);
                }
            },
            Child(ref child) => {
                child.deref().quary(centre, scale, key, |k, v| { cb(k, v) });
            }
        }
    }
}

