use serde::{Deserialize, Serialize};
use rapier3d_f64::geometry::*;
use rapier3d_f64::na as na;

#[derive(Debug, Serialize, Deserialize, Clone)]
pub enum Geometry {
    Box {
        x : [f64; 3],
        rot : [f64; 4],
        size : [f64; 3]
    },
    Sphere {
        x : [f64; 3],
        r : f64
    },
    Capsule { 
        radius : f64, 
        half_height : f64,
        position : [f64; 3],
        rotation : [f64; 4]
    }
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct NamedGeometry {
    pub name : String,
    pub geometry : Geometry
}


pub trait IntoCollider {
    fn into(&self) -> Collider;
}

impl IntoCollider for Geometry {
    fn into(&self) -> Collider {
        match self {
            Geometry::Box {x, rot, size} => {
                let trans = na::Isometry3::<f64>::from_parts(
                    na::Translation3::new(x[0], x[1], x[2]),
                    na::UnitQuaternion::from_quaternion(na::Quaternion::new(rot[0], rot[1], rot[2], rot[3]))
                );
                let mut collider = ColliderBuilder::cuboid(
                    size[0] / 2., 
                    size[1] / 2., 
                    size[2] / 2.
                ).build();
                collider.set_position(trans);
                collider
            },
            Geometry::Sphere {x, r} => {
                let translation = na::Vector3::new(x[0], x[1], x[2]);
                let mut collider = ColliderBuilder::ball(*r).build();
                collider.set_translation(translation);
                collider
            },
            Geometry::Capsule { radius, half_height, position, rotation } => {
                let trans = na::Isometry3::<f64>::from_parts(
                    na::Translation3::new(position[0], position[1], position[2]),
                    na::UnitQuaternion::from_quaternion(na::Quaternion::new(rotation[0], rotation[1], rotation[2], rotation[3]))
                );
                let mut collider = ColliderBuilder::capsule_z(half_height / 2., 0.75*radius / 2.).build();
                collider.set_position(trans);
    
                collider
            }
        }
    }
}


impl IntoCollider for NamedGeometry {
    fn into(&self) -> Collider {
        IntoCollider::into(&self.geometry)
    }
}

