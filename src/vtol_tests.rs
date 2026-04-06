use bevy::prelude::*;

use crate::{
    AtmosphereSample, FlightAeroState, FlightControlChannels, FlightControlInput, FlightKinematics,
    FlightPlugin, FlightTelemetry, LandingGearState,
    config::{
        ChannelResponse, ContactGeometry, ControlChannelBinding, ControlInputSource,
        GroundHandling, HybridActuators, HybridVehicleModel, TrimInputSource, VehicleActuators,
        VehicleControlMap, VehicleModel,
    },
    model::{common::sample_motion, vtol::evaluate_hybrid},
};

fn tiltrotor_transport_setup() -> (
    VehicleModel,
    VehicleActuators,
    VehicleControlMap,
    GroundHandling,
    crate::FlightBody,
    crate::FlightAssist,
) {
    (
        VehicleModel::hybrid(HybridVehicleModel::default()),
        VehicleActuators::hybrid(HybridActuators::default()),
        VehicleControlMap {
            pitch: ControlChannelBinding {
                source: ControlInputSource::Pitch,
                trim: Some(TrimInputSource::Pitch),
                trim_scale: 1.0,
                response: ChannelResponse {
                    rise_per_second: 5.4,
                    fall_per_second: 5.4,
                    exponent: 1.25,
                },
                ..default()
            },
            roll: ControlChannelBinding {
                source: ControlInputSource::Roll,
                trim: Some(TrimInputSource::Roll),
                trim_scale: 1.0,
                response: ChannelResponse {
                    rise_per_second: 5.8,
                    fall_per_second: 5.8,
                    exponent: 1.15,
                },
                ..default()
            },
            yaw: ControlChannelBinding {
                source: ControlInputSource::Yaw,
                trim: Some(TrimInputSource::Yaw),
                trim_scale: 1.0,
                response: ChannelResponse {
                    rise_per_second: 4.4,
                    fall_per_second: 4.4,
                    exponent: 1.0,
                },
                ..default()
            },
            forward_thrust: ControlChannelBinding {
                source: ControlInputSource::Throttle,
                clamp_min: 0.0,
                clamp_max: 1.0,
                response: ChannelResponse {
                    rise_per_second: 0.85,
                    fall_per_second: 1.25,
                    exponent: 1.0,
                },
                ..default()
            },
            vertical_thrust: ControlChannelBinding {
                source: ControlInputSource::Collective,
                clamp_min: 0.0,
                clamp_max: 1.0,
                response: ChannelResponse {
                    rise_per_second: 1.15,
                    fall_per_second: 1.35,
                    exponent: 1.0,
                },
                ..default()
            },
            transition: ControlChannelBinding {
                source: ControlInputSource::Transition,
                clamp_min: 0.0,
                clamp_max: 1.0,
                response: ChannelResponse {
                    rise_per_second: 0.35,
                    fall_per_second: 0.35,
                    exponent: 1.0,
                },
                ..default()
            },
            ..default()
        },
        GroundHandling {
            enabled: true,
            contact_geometry: ContactGeometry {
                contact_offset_below_origin_m: 1.95,
                retractable: true,
                extension_rate_per_second: 0.85,
                ground_effect_height_m: 5.5,
                ground_effect_boost: 0.16,
            },
            longitudinal_damping_per_second: 0.16,
            lateral_damping_per_second: 1.6,
            angular_damping_per_second: 1.6,
        },
        crate::FlightBody::new(2_950.0, Vec3::new(6_400.0, 7_200.0, 8_400.0)),
        crate::FlightAssist {
            wings_leveling: 0.18,
            coordinated_turn: 0.16,
            hover_leveling: 0.44,
        },
    )
}

fn sample_transform() -> Transform {
    Transform::IDENTITY
}

#[test]
fn vtol_hover_mode_produces_upward_thrust() {
    let (model_component, actuator_component, _map, ground, body, assist) =
        tiltrotor_transport_setup();
    let transform = sample_transform();
    let motion = sample_motion(
        &transform,
        &FlightKinematics::default(),
        &crate::FlightEnvironment::default(),
    );
    let model = match model_component.kind {
        crate::VehicleModelKind::Hybrid(model) => model,
        _ => unreachable!(),
    };
    let actuators = match actuator_component.kind {
        crate::VehicleActuatorKind::Hybrid(actuators) => actuators,
        _ => unreachable!(),
    };
    let evaluated = evaluate_hybrid(
        motion,
        &transform,
        body,
        model,
        actuators,
        FlightControlChannels {
            vertical_thrust: 0.78,
            transition: 0.0,
            ..default()
        },
        FlightControlInput {
            collective: 0.78,
            vtol_transition: 0.0,
            ..default()
        },
        assist,
        FlightAeroState {
            atmosphere: AtmosphereSample::default(),
            ..default()
        },
        0.0,
        LandingGearState::default(),
        Some(ground),
    );

    assert!(evaluated.forces.thrust_world_newtons.y > 0.0);
}

#[test]
fn vtol_forward_mode_points_thrust_along_forward_axis() {
    let (model_component, actuator_component, _map, ground, body, assist) =
        tiltrotor_transport_setup();
    let transform = sample_transform();
    let motion = sample_motion(
        &transform,
        &FlightKinematics {
            linear_velocity_world_mps: Vec3::new(0.0, 0.0, -55.0),
            ..default()
        },
        &crate::FlightEnvironment::default(),
    );
    let model = match model_component.kind {
        crate::VehicleModelKind::Hybrid(model) => model,
        _ => unreachable!(),
    };
    let actuators = match actuator_component.kind {
        crate::VehicleActuatorKind::Hybrid(actuators) => actuators,
        _ => unreachable!(),
    };
    let evaluated = evaluate_hybrid(
        motion,
        &transform,
        body,
        model,
        actuators,
        FlightControlChannels {
            vertical_thrust: 0.82,
            transition: 1.0,
            ..default()
        },
        FlightControlInput {
            collective: 0.82,
            vtol_transition: 1.0,
            ..default()
        },
        assist,
        FlightAeroState {
            atmosphere: AtmosphereSample::default(),
            ..default()
        },
        0.0,
        LandingGearState::default(),
        Some(ground),
    );

    assert!(evaluated.forces.thrust_world_newtons.z < 0.0);
    assert!(
        evaluated.forces.thrust_world_newtons.y.abs()
            < evaluated.forces.thrust_world_newtons.z.abs()
    );
}

#[test]
fn telemetry_exposes_vtol_transition_state() {
    let (model, actuators, control_map, ground_handling, body, assist) =
        tiltrotor_transport_setup();
    let mut app = App::new();
    app.add_plugins(MinimalPlugins);
    app.add_plugins(FlightPlugin::always_on(Update));
    app.world_mut().spawn((
        model,
        actuators,
        control_map,
        ground_handling,
        crate::FlightBody {
            use_internal_integration: false,
            ..body
        },
        assist,
        FlightControlInput {
            throttle: 0.7,
            collective: 0.7,
            vtol_transition: 0.6,
            ..default()
        },
        Transform::from_xyz(0.0, 20.0, 0.0),
    ));

    for _ in 0..20 {
        app.update();
    }

    let telemetry = app
        .world_mut()
        .query::<&FlightTelemetry>()
        .single(app.world())
        .expect("telemetry should exist");
    assert!(telemetry.vtol_transition > 0.0);
}
