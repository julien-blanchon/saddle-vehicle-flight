use bevy::prelude::*;

pub(crate) fn move_towards(current: f32, target: f32, max_delta: f32) -> f32 {
    let delta = target - current;
    if delta.abs() <= max_delta {
        target
    } else {
        current + delta.signum() * max_delta
    }
}

pub(crate) fn shape_axis(value: f32, exponent: f32) -> f32 {
    let value = value.clamp(-1.0, 1.0);
    let exponent = exponent.max(1.0);
    value.signum() * value.abs().powf(exponent)
}

pub(crate) fn smoothstep01(value: f32) -> f32 {
    let x = value.clamp(0.0, 1.0);
    x * x * (3.0 - 2.0 * x)
}

pub(crate) fn finite_or_zero(value: f32) -> f32 {
    if value.is_finite() { value } else { 0.0 }
}

pub(crate) fn sanitize_vec3(vec: Vec3) -> Vec3 {
    Vec3::new(
        finite_or_zero(vec.x),
        finite_or_zero(vec.y),
        finite_or_zero(vec.z),
    )
}
