use std::mem;
use std::num::Float;
use std::num::{FromPrimitive, one};

use cgmath::{Vector3, Vector};
use cgmath::{Point3, Point};
use cgmath::{Matrix};
use cgmath::BaseNum;

use CheckRange3;
use Intersects;
use self::Node::{Empty, Child, Data, Collide};

enum Node<K, V> {
    Empty,
    Child,
    Data(K, V),
    Collide(Vec<(K, V)>)
}

impl<K: Clone+Send+Sync, V: Clone+Send+Sync> Clone for Node<K, V> {
    fn clone(&self) -> Node<K, V> {
        match *self {
            Empty => Empty,
            Child => Child,
            Data(ref k, ref v) => Data(k.clone(), v.clone()),
            Collide(ref data) => Collide(data.clone())
        }
    }
}

pub struct Linear<S, K, V> {
    scale: S,
    depth: uint,
    data: Vec<Node<K, V>>
}

impl<S: Clone, K: Clone+Send+Sync, V: Clone+Send+Sync> Clone for Linear<S, K, V> {
    fn clone(&self) -> Linear<S, K, V> {
        Linear {
            scale: self.scale.clone(),
            depth: self.depth.clone(),
            data: self.data.clone()
        }
    }
}

fn calc_size(depth: uint) -> uint {
    if depth == 0 {
        return 0;
    }

    let mut size = 2;
    let mut total = 0;
    for _ in range(0, depth) {
        total += size*size*size;
        size *= 2;
    }

    total
}

#[derive(Clone)]
struct Frame<S> {
    center: Point3<S>,
    scale: S,
    base: uint,
    search: uint,
    left: uint
}


impl<S: Float+FromPrimitive+BaseNum> Frame<S> {
    fn start(depth: uint, scale: S) -> Frame<S> {
        Frame {
            center: Point3::new(Float::zero(), Float::zero(), Float::zero()),
            scale: scale,
            base: 0,
            search: 0,
            left: depth-1
        }
    }

    fn next(&self, idx: uint) -> Frame<S> {
        assert!(self.left != 0);
        let hscale = self.scale / (FromPrimitive::from_uint(2).unwrap());
        let one: S = Float::one();

        let offset = Vector3::new(
            if (idx & 4) == 0 {-one.clone()} else {one.clone()},
            if (idx & 2) == 0 {-one.clone()} else {one.clone()},
            if (idx & 1) == 0 {-one.clone()} else {one.clone()}
        ).mul_s(hscale.clone());


        Frame {
            center: self.center.add_v(&offset),
            scale: hscale,
            base: self.base*8 + 8,
            search: (self.search+idx)*8,
            left: self.left - 1
        }
    }

    fn addr(&self, idx: uint) -> uint {
        self.base + self.search + idx
    }
}

impl<S: Float+FromPrimitive+BaseNum, K: Clone+Send+Sync+CheckRange3<S>+Intersects<K>+PartialEq, V: Clone+Send+Sync> Linear<S, K, V> {
    pub fn new(size: S, depth: uint) -> Linear<S, K, V> {
        assert!(depth != 0);
        let elements = calc_size(depth);

        Linear {
            scale: size,
            depth: depth,
            data: range(0, elements).map(|_| { Empty }).collect()
        }
    }

    fn _insert(&mut self, frame: Frame<S>, key: &K, value: &V) {
        let touched = key.check3(&frame.center, frame.scale.clone());

        for (idx, &touch) in touched.iter().enumerate() {
            if touch {
                let mut next = Empty;
                mem::swap(&mut next, &mut self.data[frame.addr(idx)]);

                self.data[frame.addr(idx)] = match next {
                    Empty => Data(key.clone(), value.clone()),
                    Child => {
                        self._insert(frame.next(idx), key, value);
                        Child
                    },
                    Data(old_key, old_value) => {
                        if frame.left != 0 && !old_key.intersect(key) {
                            self._insert(frame.next(idx), &old_key, &old_value);
                            self._insert(frame.next(idx), key, value);
                            Child
                        } else {
                            let data = vec!((old_key.clone(), old_value.clone()),
                                            (key.clone(), value.clone()));
                            Collide(data)
                        }
                    },
                    Collide(mut data) => {
                        let add_to_self = if frame.left == 0 {
                            true
                        } else {
                            let mut all_collide = true;
                            for &(ref k, _) in data.iter() {
                                all_collide &= key.intersect(k);
                            }
                            all_collide
                        };

                        if add_to_self {
                            data.push((key.clone(), value.clone()));
                            Collide(data)
                        } else {
                            let frame_next = frame.next(idx);
                            for &(ref k, ref v) in data.iter() {
                                self._insert(frame_next.clone(), k, v);
                            }
                            self._insert(frame_next.clone(), key, value);
                            Child
                        }
                    }
                };
            }
        }
    }

    pub fn insert(&mut self, key: K, value: V) {
        let scale = self.scale.clone();
        let depth = self.depth.clone();
        self._insert(Frame::start(depth, scale), &key, &value);
    }

    fn _remove(&mut self, frame: Frame<S>, key: &K) {
        let touched = key.check3(&frame.center, frame.scale.clone());

        for (idx, &touch) in touched.iter().enumerate() {
            if touch {
                let mut next = Empty;
                mem::swap(&mut next, &mut self.data[frame.addr(idx)]);

                self.data[frame.addr(idx)] = match next {
                    Empty => Empty,
                    Child => {
                        self._remove(frame.next(idx), key);
                        Child
                    },
                    Data(k, v) => {
                        if k == *key {
                            Empty
                        } else {
                            Data(k, v)
                        }
                    },
                    Collide(mut data) => {
                        let mut index = None;
                        for (i, &(ref k, _)) in data.iter().enumerate() {
                            if *k == *key {
                                index = Some(i);
                                break
                            }
                        }

                        match index {
                            Some(i) => {data.remove(i);},
                            None => (),
                        };

                        Collide(data)
                    }
                };
            }
        }
    }

    pub fn remove(&mut self, key: K) {
        let scale = self.scale.clone();
        let depth = self.depth.clone();
        self._remove(Frame::start(depth, scale), &key);
    }

    fn _quary<Q: CheckRange3<S>>(&self, frame: Frame<S>, key: &Q, cb: |&K, &V|) {
        let touched = key.check3(&frame.center, frame.scale.clone());

        for (idx, &touch) in touched.iter().enumerate() {
            if touch {
                match self.data[frame.addr(idx)] {
                    Empty => (),
                    Child => self._quary(frame.next(idx), key, |k, v| { cb(k, v) }),
                    Data(ref k, ref v) => cb(k, v),
                    Collide(ref data) => {
                        for &(ref k, ref v) in data.iter() {
                            cb(k, v);
                        }
                    }
                };
            }
        }        
    }

    pub fn quary<Q: CheckRange3<S>>(&self, key: &Q, cb: |&K, &V|) {
        let scale = self.scale.clone();
        self._quary(Frame::start(self.depth, scale), key, |k, v| { cb(k, v) });
    }
}