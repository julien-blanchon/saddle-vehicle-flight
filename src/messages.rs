use bevy::prelude::*;

#[derive(Message, Debug, Clone, Copy)]
pub struct StallEntered {
    pub entity: Entity,
    pub angle_of_attack_rad: f32,
    pub stall_amount: f32,
}

#[derive(Message, Debug, Clone, Copy)]
pub struct StallRecovered {
    pub entity: Entity,
    pub angle_of_attack_rad: f32,
}

#[derive(Message, Debug, Clone, Copy)]
pub struct GearStateChanged {
    pub entity: Entity,
    pub deployed: bool,
    pub position: f32,
}
