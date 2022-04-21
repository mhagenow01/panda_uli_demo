
extern crate k;
use k::{nalgebra as na, SerialChain};
use na::{UnitQuaternion, Vector3};
use optimization_engine::{SolverError};

/// Groove loss function taken from "An analysis of RelaxedIK: an optimization-based framework for generating accurate and feasible robot arm motions"
/// x := Optimization variable
/// t := Target value
/// d := Exponent of the exponential (2 -> Gaussian)
/// c := Width of the groove
/// f := Scale of the long range basin
/// g := Exponent of the long range basin (2 -> Quadratic)
/// @article{article,
///     author = {Rakita, Daniel and Mutlu, Bilge and Gleicher, Michael},
///     year = {2020},
///     month = {09},
///     pages = {},
///     title = {An analysis of RelaxedIK: an optimization-based framework for generating accurate and feasible robot arm motions},
///     volume = {44},
///     journal = {Autonomous Robots},
///     doi = {10.1007/s10514-020-09918-9}
/// }
fn groove_loss(x: f64, t: f64, d: i32, c: f64, f: f64, g: i32) -> f64 {
    -( (-(x - t).powi(d)) / (2.0 * c.powi(2) ) ).exp() + f * (x - t).powi(g)
}

pub fn groove_loss_derivative(x_val: f64, t: f64, d: i32, c: f64, f: f64, g: i32) -> f64 {
    -( (-(x_val - t).powi(d)) / (2.0 * c.powi(2) ) ).exp() *  ((-d as f64 * (x_val - t)) /  (2.0 * c.powi(2))) + g as f64 * f * (x_val - t)
}

/// Simple finite scheme for functions that are optimizable in OpEn.
/// Uses a 0, -1, 1 forward difference stencil.
pub fn finite_difference(f: &dyn Fn(&[f64], &mut f64) -> Result<(), SolverError>, u: &[f64], grad: &mut [f64]) -> Result<(), SolverError> {
    let h = 1e-8;
    let mut f0 = 0.0;
    f(u, &mut f0)?;

    let mut x : Vec<f64> = u.iter().map(|&v| v).collect();
    for i in 0..x.len() {
        let mut fi = 0.0;
        x[i] += h;
        f(&x, &mut fi)?;
        grad[i] = (fi - f0) / h;
        x[i] -= h;
    }

    Ok(())
}

pub fn position_cost(current_position: &Vector3<f64>, desired_position: &Vector3<f64>) -> f64 {
    5.0 * groove_loss((current_position - desired_position).norm(), 0., 2, 0.1, 10.0, 2)
}

pub fn position_gradient(current_position: &Vector3<f64>, desired_position: &Vector3<f64>) -> Vector3<f64> {
    let dx = current_position - desired_position;
    5.0 * groove_loss_derivative(dx.norm(), 0., 2, 0.1, 10.0, 2) * dx
}

pub fn rotation_cost(current_rotation: &UnitQuaternion<f64>, desired_rotation: &UnitQuaternion<f64>) -> f64 {
    3.0 * groove_loss(current_rotation.angle_to(desired_rotation).powi(2), 0., 2, 0.1, 10.0, 2)
}

pub fn rotation_gradient(current_rotation: &UnitQuaternion<f64>, desired_rotation: &UnitQuaternion<f64>) -> Vector3<f64> {
    let dr = current_rotation.rotation_to(desired_rotation).scaled_axis();
    -2. * dr * groove_loss_derivative(dr.norm_squared(), 0., 2, 0.1, 10.0, 2)
}

pub fn gradient(arm: &SerialChain<f64>, u: &[f64], desired_position: &Vector3<f64>, desired_rotation: &UnitQuaternion<f64>, grad: &mut [f64]) {
    arm.set_joint_positions_clamped(u);
    arm.update_transforms();
    let transform = arm.end_transform();
    let jac = k::jacobian(arm);

    let pgrad = position_gradient(&transform.translation.vector, desired_position);
    let rgrad = rotation_gradient(&transform.rotation, desired_rotation);

    let mut cart_grad = na::Vector6::<f64>::zeros();
    for i in 0..3 {
        cart_grad[i] = pgrad[i];
        cart_grad[i + 3] = rgrad[i];
    }

    let grad_vec = cart_grad.transpose() * jac;
    for i in 0..grad_vec.len() {
        grad[i] = grad_vec[i];
    }
}
