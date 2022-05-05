
use std::collections::HashMap;

use rapier3d_f64::na::{Point3};
use rapier3d_f64::prelude::*;
use rapier3d_f64::parry::query::*;
use k;

fn sign(x: f64) -> f64 {
    if x.abs() <= 1e-10 {
        0.
    } else {
        x.signum()
    }
}

/// Lerps from the start angle to the end while 
/// respecting collision geometry
/// Returns the new end angles along with updated joint constraints
pub fn lerp(arm: &k::SerialChain<f64>, end: &Vec<f64>, 
    robot_geometry: &HashMap<String, Collider>, static_geometry: &ColliderSet,
    dq: f64) -> Vec<f64> {
    
    let mut q = arm.joint_positions();
    
    let cutoff = 0.02;
    if collision_check(arm, robot_geometry, static_geometry, cutoff) {
        return q;
    }
    let mut last_valid_step = q.clone();
    while step(&mut q, dq, &end) {
        arm.set_joint_positions_clamped(&q);
        
        if collision_check(arm, robot_geometry, static_geometry,cutoff) {
            return last_valid_step;
        }
        last_valid_step.clone_from(&q);
    }

    end.clone()
}


fn step(x : &mut Vec<f64>, dx: f64, goal: &Vec<f64>) -> bool {
    let mut stepped = false;
    for i in 0..x.len() {
        let delta = dx.abs().min((goal[i] - x[i]).abs());
        if delta > 1e-8 {
            stepped = true;
            x[i] += delta * (goal[i] - x[i]).signum();
        }
    }
    stepped
}

fn collision_check(arm: &k::SerialChain<f64>,
    robot_geometry: &HashMap<String, Collider>, static_geometry: &ColliderSet,
    cutoff: f64) -> bool {
    arm.update_transforms();
    for joint in arm.iter_joints().filter(|j| j.is_movable()) {

        if let Some(collider) = robot_geometry.get(&joint.name) {
            let trans = joint.world_transform().unwrap();
            let trans : rapier3d_f64::na::Isometry3<f64> = unsafe { std::mem::transmute(trans) };
            let shape = collider.shape();
            let world_trans = trans * collider.position();
            if proximity(static_geometry, &world_trans, shape, cutoff).is_some() {
                return true;
            }
        }
    }
    return false;
}


fn proximity(static_geometry: &ColliderSet, trans: &rapier3d_f64::na::Isometry3<f64>, shape: &dyn Shape, max_dist : f64) -> Option<(Point3<f64>, Point3<f64>)> {
    for (_handle, collider) in static_geometry.iter() {
        let result = rapier3d_f64::parry::query::closest_points(
            &trans, 
            shape, 
            collider.position(), 
            collider.shape(), 
            max_dist
        );

        if let Ok(closest_points) = result {
            match closest_points {
                ClosestPoints::Intersecting => {
                    // If we are somehow already inside, then repel from the collider position
                    // But it doesn't do that yet.
                    return None;
                },
                ClosestPoints::WithinMargin(a, b) => {
                    return Some((a, b));
                },
                ClosestPoints::Disjoint => {
                    return None;
                }
            }
        }
    }

    None
}