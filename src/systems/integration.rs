use crate::{
    components::{FlightBody, FlightKinematics, LandingGearState},
    config::{FixedWingAircraft, HelicopterAircraft, SpacecraftConfig, VtolAircraft},
    math::sanitize_vec3,
    telemetry::FlightForces,
};
use bevy::prelude::*;

pub(crate) fn integrate_motion(
    time: Res<Time>,
    mut query: Query<(
        &FlightBody,
        &FlightForces,
        &mut FlightKinematics,
        &mut Transform,
    )>,
) {
    let dt = time.delta_secs();
    if dt <= 0.0 {
        return;
    }

    for (body, forces, mut kinematics, mut transform) in &mut query {
        if !body.use_internal_integration {
            continue;
        }

        let linear_accel = forces.total_force_world_newtons / body.mass_kg.max(1e-3);
        let angular_accel = Vec3::new(
            forces.total_torque_body_nm.x / body.inertia_kgm2.x.max(1e-3),
            forces.total_torque_body_nm.y / body.inertia_kgm2.y.max(1e-3),
            forces.total_torque_body_nm.z / body.inertia_kgm2.z.max(1e-3),
        );

        kinematics.linear_velocity_world_mps =
            sanitize_vec3(kinematics.linear_velocity_world_mps + linear_accel * dt);
        kinematics.angular_velocity_body_rps =
            sanitize_vec3(kinematics.angular_velocity_body_rps + angular_accel * dt);

        transform.translation =
            sanitize_vec3(transform.translation + kinematics.linear_velocity_world_mps * dt);

        let angular_velocity_world = transform.right() * kinematics.angular_velocity_body_rps.x
            + transform.up() * kinematics.angular_velocity_body_rps.y
            + transform.forward() * kinematics.angular_velocity_body_rps.z;
        let delta_rotation = Quat::from_scaled_axis(angular_velocity_world * dt);
        transform.rotation = (delta_rotation * transform.rotation).normalize();
    }
}

pub(crate) fn resolve_ground_contact(
    time: Res<Time>,
    mut query: Query<(
        &mut Transform,
        &mut FlightKinematics,
        &mut LandingGearState,
        Option<&FixedWingAircraft>,
        Option<&HelicopterAircraft>,
        Option<&VtolAircraft>,
        Option<&SpacecraftConfig>,
        &crate::components::FlightEnvironment,
    )>,
) {
    let dt = time.delta_secs();
    for (
        mut transform,
        mut kinematics,
        mut gear,
        fixed_wing,
        helicopter,
        vtol,
        _spacecraft,
        environment,
    ) in &mut query
    {
        let Some(surface_altitude) = environment.surface_altitude_msl_m else {
            gear.contact = false;
            continue;
        };
        let contact = if let Some(aircraft) = fixed_wing {
            aircraft.landing_contact
        } else if let Some(aircraft) = helicopter {
            aircraft.contact_geometry
        } else if let Some(aircraft) = vtol {
            aircraft.contact_geometry
        } else {
            gear.contact = false;
            continue;
        };

        let contact_offset = contact.effective_offset_m(gear.position);
        let lowest_point = transform.translation.y - contact_offset;
        if lowest_point <= surface_altitude {
            gear.contact = true;
            transform.translation.y = surface_altitude + contact_offset;
            if kinematics.linear_velocity_world_mps.y < 0.0 {
                kinematics.linear_velocity_world_mps.y = 0.0;
            }
            let horizontal_damping =
                (1.0 - contact.surface_damping_per_second * dt).clamp(0.0, 1.0);
            if fixed_wing.is_some() || vtol.is_some() {
                let body_forward = Vec3::new(transform.forward().x, 0.0, transform.forward().z)
                    .normalize_or_zero();
                let body_right =
                    Vec3::new(transform.right().x, 0.0, transform.right().z).normalize_or_zero();
                let horizontal_velocity = Vec3::new(
                    kinematics.linear_velocity_world_mps.x,
                    0.0,
                    kinematics.linear_velocity_world_mps.z,
                );
                let forward_speed = horizontal_velocity.dot(body_forward);
                let side_speed = horizontal_velocity.dot(body_right);
                let forward_damping =
                    (1.0 - contact.surface_damping_per_second * 0.1 * dt).clamp(0.0, 1.0);
                let resolved_horizontal = body_forward * (forward_speed * forward_damping)
                    + body_right * (side_speed * horizontal_damping);
                kinematics.linear_velocity_world_mps.x = resolved_horizontal.x;
                kinematics.linear_velocity_world_mps.z = resolved_horizontal.z;
            } else {
                kinematics.linear_velocity_world_mps.x *= horizontal_damping;
                kinematics.linear_velocity_world_mps.z *= horizontal_damping;
            }
            kinematics.angular_velocity_body_rps *= horizontal_damping;
        } else {
            gear.contact = false;
        }
    }
}

pub(crate) fn sanitize_state(
    mut query: Query<(&mut Transform, &mut FlightKinematics, &mut FlightForces)>,
) {
    for (mut transform, mut kinematics, mut forces) in &mut query {
        transform.translation = sanitize_vec3(transform.translation);
        if !transform.rotation.is_finite() {
            transform.rotation = Quat::IDENTITY;
        }
        kinematics.linear_velocity_world_mps = sanitize_vec3(kinematics.linear_velocity_world_mps);
        kinematics.angular_velocity_body_rps = sanitize_vec3(kinematics.angular_velocity_body_rps);
        forces.thrust_world_newtons = sanitize_vec3(forces.thrust_world_newtons);
        forces.lift_world_newtons = sanitize_vec3(forces.lift_world_newtons);
        forces.drag_world_newtons = sanitize_vec3(forces.drag_world_newtons);
        forces.side_force_world_newtons = sanitize_vec3(forces.side_force_world_newtons);
        forces.gravity_world_newtons = sanitize_vec3(forces.gravity_world_newtons);
        forces.total_force_world_newtons = sanitize_vec3(forces.total_force_world_newtons);
        forces.control_torque_body_nm = sanitize_vec3(forces.control_torque_body_nm);
        forces.damping_torque_body_nm = sanitize_vec3(forces.damping_torque_body_nm);
        forces.assist_torque_body_nm = sanitize_vec3(forces.assist_torque_body_nm);
        forces.total_torque_body_nm = sanitize_vec3(forces.total_torque_body_nm);
    }
}
