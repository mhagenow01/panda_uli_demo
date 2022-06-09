#![allow(dead_code)]
#![allow(unused_variables)]
use std::collections::HashMap;
use std::error::Error;
use std::ptr::null;
use serde_yaml;
use serde::{Serialize, Deserialize};
use rapier3d_f64::geometry::*;
use rapier3d_f64::na;
use na::{Vector3, UnitQuaternion, Quaternion,Isometry3};
use solver::{k_to_r, r_to_k};
use optimization_engine::panoc::*;
extern crate k;
use std::os::raw::c_char;
use std::ffi::CStr;

mod solver;
mod planner;
mod geometry;
use crate::geometry::*;

#[derive(Debug, PartialEq, Serialize, Deserialize)]
pub struct SolverConfig {
    pub max_iter: usize,
    pub max_time_ms: u64,
    pub local_search_bound: f64
}

#[derive(Debug, PartialEq, Serialize, Deserialize)]
pub struct PlannerConfig {
    pub dq : f64,
    pub safety_radius: f64,
    pub enabled: bool
}

#[derive(Debug, PartialEq, Serialize, Deserialize)]
pub struct IKSolverConfig {
   pub solver: SolverConfig,
   pub planner: PlannerConfig,
}

pub struct IKSolver {
    pub cache : PANOCCache,
    pub robot : k::Chain<f64>,
    pub arm : k::SerialChain<f64>,
    pub arm_colliders : HashMap::<String, Collider>,
    pub environment : ColliderSet,
    pub lb: Vec<f64>,
    pub ub: Vec<f64>,
    pub config: IKSolverConfig
}

impl IKSolver {
    pub fn new(cache : PANOCCache, robot: k::Chain<f64>, arm : k::SerialChain<f64>, arm_colliders : HashMap<String, Collider>, environment: ColliderSet, config: IKSolverConfig) -> IKSolver {

        let mut lb = Vec::new();
        let mut ub = Vec::new();

        for joint in arm.iter_joints() {
            lb.push(joint.limits.unwrap().min + 0.1);
            ub.push(joint.limits.unwrap().max - 0.1);
        }
    
        IKSolver { cache, robot, arm, arm_colliders, environment, lb, ub, config}
    }

    pub fn set_ee(&mut self, name: &str) -> bool {
        match self.robot.find(name) {
            None => {
                false
            }
            Some(joint) => {
                self.arm = k::SerialChain::from_end(joint);
                true
            }
        }
    }
}

fn parse_c_str(s: *const c_char) -> Result<&'static str, Box<dyn Error>> {
    Ok(unsafe { CStr::from_ptr(s).to_str()? })
}


fn make_solver(urdf: &str, ee_frame: &str, arm_colliders : &str, environment : &str, config: &str) -> Option<IKSolver> {
    let robot : k::Chain<f64> = urdf_rs::read_from_string(urdf).ok()?.into();
    let arm = k::SerialChain::from_end(robot.find(ee_frame)?);
    let mut qs = Vec::new();
    for i in 0..arm.dof() {
        qs.push(0.);
    }
    arm.set_joint_positions_clamped(&qs);
    arm.update_transforms();

    let cache = PANOCCache::new(arm.dof(), 1e-6, 10);

    let environment_geometry : Vec<Geometry> = serde_json::de::from_str(environment).ok()?;
    let mut environment = ColliderSet::new();
    
    for env_geom in environment_geometry.iter() {
        environment.insert(IntoCollider::into(env_geom));
    }

    let arm_geometry : Vec<NamedGeometry> = serde_json::de::from_str::<Vec<NamedGeometry>>(arm_colliders).ok()?;
    let mut arm_colliders = HashMap::<String, Collider>::new();
    for named_geom in arm_geometry {
        let mut collider = IntoCollider::into(&named_geom);
        let joint_trans = arm.find(&named_geom.name)?.world_transform().unwrap();
        let world_trans = collider.position().clone();

        collider.set_position(k_to_r(&joint_trans.inv_mul(&r_to_k(&world_trans))));

        arm_colliders.insert(
            named_geom.name,
            collider
        );
    }


    
    Some(IKSolver::new(cache, robot, arm, arm_colliders, environment, serde_yaml::from_str(config).unwrap()))
}

#[no_mangle]
pub extern "C" fn new_solver(urdf_ptr: *const c_char, ee_frame_ptr: *const c_char, arm_colliders_ptr: *const c_char, environment_ptr: *const c_char, config_ptr: *const c_char) -> *const IKSolver {
    let urdf = parse_c_str(urdf_ptr).unwrap();
    let ee_frame = parse_c_str(ee_frame_ptr).unwrap();
    let arm_colliders = parse_c_str(arm_colliders_ptr).unwrap();
    let environment = parse_c_str(environment_ptr).unwrap();
    let config = parse_c_str(config_ptr).unwrap();
    
    match make_solver(urdf, ee_frame, arm_colliders, environment, config) {
        Some(solver) => {
            Box::into_raw(Box::new(solver))
        },
        None => {
            null()
        }
    }
}

fn try_solve(iksolver: *mut IKSolver, current_q_ptr: *mut f64, trans_ptr: *const [f64; 7]) -> Option<(Vec<f64>,[f64; 7])> {
    let iksolver = unsafe { iksolver.as_mut()? };
    let current_q;
    let trans;
    unsafe {
        current_q = Vec::from(std::slice::from_raw_parts(current_q_ptr, iksolver.arm.dof()));
        trans = std::ptr::read(trans_ptr);
    }
    

    let x = Vector3::new(trans[0], trans[1], trans[2]);
    let rot = UnitQuaternion::from_quaternion(Quaternion::new(trans[3], trans[4], trans[5], trans[6]));
    iksolver.arm.set_joint_positions_clamped(&current_q);
    let current_q = iksolver.arm.joint_positions();
    let mut lb = iksolver.lb.clone();
    let mut ub = iksolver.ub.clone();
    for i in 0..lb.len() {
        lb[i] = lb[i].max(current_q[i] - iksolver.config.solver.local_search_bound);
        ub[i] = ub[i].min(current_q[i] + iksolver.config.solver.local_search_bound);
    }
    let res = solver::solve(
        &iksolver.arm, &mut iksolver.cache, &iksolver.arm_colliders, &iksolver.environment, 
        &x, &rot, &lb, &ub,
        iksolver.config.solver.max_iter,
        iksolver.config.solver.max_time_ms, // Yea, this is way too may parameters. Ohh well.
    );
    res.and_then(|q| { 
        iksolver.arm.set_joint_positions_clamped(&current_q);
        if iksolver.config.planner.enabled {
            let q = planner::lerp(
                &iksolver.arm, &q, &iksolver.arm_colliders, 
                &iksolver.environment, iksolver.config.planner.dq,
                iksolver.config.planner.safety_radius
            );
            iksolver.arm.set_joint_positions_clamped(&q);
        }
        iksolver.arm.update_transforms();
        let iso = iksolver.arm.end_transform();
        let trans = iso.translation.vector;
        let rot = iso.rotation;
        let pose_return: [f64; 7]=[trans[0],trans[1],trans[2],rot[0],rot[1],rot[2],rot[3]];
        Some((q,pose_return))
    })
}

#[no_mangle]
pub extern "C" fn solve(iksolver: *mut IKSolver, current_q_ptr: *mut f64, trans_ptr: *const [f64; 7], q_ptr: *mut f64,t_ptr: *mut f64) -> bool {
    match try_solve(iksolver, current_q_ptr, trans_ptr) {
        Some((q,t)) => {
            let q_array;
            unsafe {
                q_array = std::slice::from_raw_parts_mut(q_ptr, q.len());
            };
            for i in 0..q_array.len() {
                q_array[i] = q[i];
            }
            let t_array;
            unsafe {
                t_array = std::slice::from_raw_parts_mut(t_ptr, t.len());
            };
            for i in 0..t_array.len() {
                t_array[i] = t[i];
            }
            true
        },
        None => {
            false
        }
    }
}


#[no_mangle]
pub extern "C" fn dof(iksolver: *mut IKSolver) -> i32 {
    match unsafe { iksolver.as_ref() } {
        Some(solver) => {
            solver.arm.dof() as i32
        },
        None => {
            -1
        }
    }
}


#[no_mangle]
pub extern "C" fn deallocate(ptr: *mut IKSolver) {
    if ptr.is_null() {
        return;
    }
    unsafe {
        Box::from_raw(ptr);
    }
}

fn try_set_ee(iksolver: *mut IKSolver, ee_frame_ptr: *const c_char) -> Option<bool> {
    let iksolver = unsafe { iksolver.as_mut()? };
    let ee_frame = parse_c_str(ee_frame_ptr).unwrap();

    Some(iksolver.set_ee(ee_frame))
}

#[no_mangle]
pub extern "C" fn set_ee(iksolver: *mut IKSolver, ee_frame_ptr: *const c_char) -> bool {
    match try_set_ee(iksolver, ee_frame_ptr) {
        Some(success) => {
            success
        },
        None => {
            false
        }
    }
}


#[cfg(test)]
mod tests {
    use std::fs::read_to_string;
    use crate::{make_solver};
    use crate::solver;
    use rapier3d_f64::na;
    use na::{Vector3, UnitQuaternion, Quaternion};
    #[test]
    fn collision_test() {
        let mut iksolver = make_solver(
            &read_to_string("config/panda.urdf").unwrap(),
            "panda_gripper_joint", 
            &read_to_string("config/panda_collision.json").unwrap(),
            &read_to_string("config/environment.json").unwrap(),
            &read_to_string("config/solver_config.yaml").unwrap(),
        ).unwrap();
        
        let x = Vector3::new(0.3, 0., 0.2);
        let rot = UnitQuaternion::from_quaternion(Quaternion::new(1., 0., 0., 0.));
        
        let res = solver::solve(
            &iksolver.arm, &mut iksolver.cache, &iksolver.arm_colliders, &iksolver.environment, 
            &x, &rot, &iksolver.lb, &iksolver.ub,
            iksolver.config.solver.max_iter,
            iksolver.config.solver.max_time_ms, // Yea, this is way too may parameters. Ohh well.
        ).unwrap();
        println!("{:?}", res);
        println!("{}", iksolver.arm.end_transform());

        assert!(false);
    }
}