mod utils;
use utils::*;

use std::collections::HashMap;

extern crate k;
use k::{nalgebra as na};
use na::{UnitQuaternion, Vector3, Quaternion};
use optimization_engine::{constraints::*, panoc::*, *};

use rapier3d_f64::prelude::*;
use rapier3d_f64::na as rna;
use rapier3d_f64::parry::query::*;

use std::time::Duration;


pub fn k_to_r(trans: &k::Isometry3<f64>) -> rna::Isometry3<f64> {
    
    let mut rna_trans = rna::Isometry3::identity();
    rna_trans.translation.vector = rna::Vector3::new(
        trans.translation.vector.x,
        trans.translation.vector.y,
        trans.translation.vector.z
    );

    rna_trans.rotation = rna::UnitQuaternion::from_quaternion(rna::Quaternion::new(
        trans.rotation.w,
        trans.rotation.i,
        trans.rotation.j,
        trans.rotation.k
    ));
    rna_trans
}

pub fn r_to_k(trans: &rna::Isometry3<f64>) -> k::Isometry3<f64> {
    let mut k_trans = k::Isometry3::identity();
    k_trans.translation.vector = k::Vector3::new(
        trans.translation.vector.x,
        trans.translation.vector.y,
        trans.translation.vector.z
    );

    k_trans.rotation = k::UnitQuaternion::from_quaternion(na::Quaternion::new(
        trans.rotation.w,
        trans.rotation.i,
        trans.rotation.j,
        trans.rotation.k
    ));
    k_trans
}

fn proximity(static_geometry: &ColliderSet,trans: &rna::Isometry3<f64>, shape: &dyn Shape, max_dist : f64) -> f64 {
    let mut dist = max_dist;
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
                    return 0.;
                },
                ClosestPoints::WithinMargin(a, b) => {
                    dist = dist.min((a - b).magnitude());
                },
                ClosestPoints::Disjoint => {

                }
            }
        }
    }

    dist
}

fn collision_cost(arm: &k::SerialChain<f64>, robot_geometry: &HashMap<String, Collider>, static_geometry: &ColliderSet) -> f64 {
    let mut c = 0.;
    let mut dists = Vec::new();
    for (name, robot_collider) in robot_geometry  {
        let max_dist = 0.05;
        let min_dist = 1e-5;
        let exp = 2;
        if let Some(joint) = arm.find(name) {
            let trans = k_to_r(&joint.world_transform().unwrap());
            let shape = robot_collider.shape();
            let world_trans = trans * robot_collider.position();
            let dist = proximity(static_geometry, &world_trans, shape, max_dist).max(min_dist);
            dists.push(dist);
            c += dist.powi(-exp) - max_dist.powi(-exp);
        }
    }
    c
}

fn regularization(current_q: &Vec<f64>, u: &[f64]) -> f64 {
    let mut c = 0.;
    for i in 0..u.len() {
        c += 0.0 * (current_q[i] - u[i]).powi(2);
    }
    c
}


pub fn solve(arm: &k::SerialChain<f64>, mut cache: &mut PANOCCache, 
    robot_geometry: &HashMap<String, Collider>, static_geometry: &ColliderSet, 
    x: &rna::Vector3<f64>, rot: &rna::UnitQuaternion<f64>,
    lb: &Vec<f64>, ub: &Vec<f64>) -> Option<Vec<f64>> {
    
    let x_k = Vector3::from_iterator(x.iter().map(|&v| v));
    let rot_k = UnitQuaternion::from_quaternion(Quaternion::new(rot.w, rot.i, rot.j, rot.k));

    let trans = arm.end_transform();
    let angle = trans.rotation.rotation_to(&rot_k).angle(); 
    let disp = (x_k - trans.translation.vector).magnitude();
    // let x_k = trans.translation.vector.lerp(&x_k, disp.min(0.02) / disp);
    // let rot_k = trans.rotation.slerp(&rot_k, angle.min(0.02) / angle);

    let current_q = arm.joint_positions();
    let cost = |u: &[f64], c: &mut f64| {
        arm.set_joint_positions_clamped(u);
        arm.update_transforms();
        *c = 0.0;

        let trans = arm.end_transform();

        *c += position_cost(&trans.translation.vector, &x_k);
        let sigma : f64 = 0.005;
        //let s2 = sigma.powi(2);
        let rotation_decay = 1.;//(-(&trans.translation.vector - &x_k).magnitude_squared() / (2. * s2)).exp();
        *c += rotation_decay * rotation_cost(&trans.rotation, &rot_k);
        *c += collision_cost(arm, &robot_geometry, static_geometry);
        *c += regularization(&current_q, u);
        Ok(())
    };

    let dcost = |u: &[f64], grad: &mut [f64]| {
        finite_difference(&cost, u, grad)
    };

    let mut u = current_q.clone();
    
    let bounds = Rectangle::new(Some(&lb[..]), Some(&ub[..]));
    let problem = Problem::new(
        &bounds,
        dcost, 
        cost
    );
    bounds.project(&mut u);
    arm.set_joint_positions_clamped(&u);

    //let jksolver = k::JacobianIkSolver::new(0.05, 0.05, 1., 100);
    let mut panoc = PANOCOptimizer::new(problem, &mut cache).with_max_iter(10).with_max_duration(Duration::from_millis(2));
    let status = panoc.solve(&mut u);
    // let jsstatus = jksolver.solve(&arm, &na::Isometry3::from_parts(
    //     na::Translation { vector: *x }, 
    //     *rot
    // ));

    // if jsstatus.is_ok() {
    //     u.copy_from_slice(arm.joint_positions().as_slice());
    // }
    if status.is_err() {
        println!("{:?}", status);
        return None;
    }
    //println!("{:?}", status.unwrap().solve_time());
    Some(u)
}
