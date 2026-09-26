use crate::network::graph::Network;
use crate::simulation::vehicle::Vehicle;
use glam::{Quat, Vec2, Vec3};
use serde::{Deserialize, Serialize};

/// 3D World Transform for Bevy integration.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct BevyTransform {
    pub translation: Vec3,
    pub rotation: Quat,
    pub scale: Vec3,
}

impl BevyTransform {
    pub fn new(translation: Vec3, rotation: Quat) -> Self {
        Self {
            translation,
            rotation,
            scale: Vec3::ONE,
        }
    }

    /// Convert from 2D plane (x, y) with heading angle (radians) to 3D Bevy coordinates.
    ///
    /// In Bevy RPG conventions (XZ ground plane with Y-up):
    /// - 2D x -> 3D X
    /// - 2D y -> 3D -Z
    /// - Elevation -> 3D Y
    pub fn from_2d_heading_xz(pos_2d: Vec2, heading_rad: f32, elevation_y: f32) -> Self {
        let translation = Vec3::new(pos_2d.x, elevation_y, -pos_2d.y);
        let rotation = Quat::from_rotation_y(heading_rad);
        Self::new(translation, rotation)
    }

    /// Convert from 2D plane (x, y) with heading angle (radians) to 2D/2.5D top-down Bevy coordinates (XY plane, Z-up).
    pub fn from_2d_heading_xy(pos_2d: Vec2, heading_rad: f32, z: f32) -> Self {
        let translation = Vec3::new(pos_2d.x, pos_2d.y, z);
        let rotation = Quat::from_rotation_z(heading_rad);
        Self::new(translation, rotation)
    }
}

/// Helper function to extract the 3D Bevy transform of a vehicle in an open-world map.
pub fn vehicle_to_bevy_transform(
    vehicle: &Vehicle,
    network: &Network,
    elevation_y: f32,
) -> BevyTransform {
    let pos_2d = vehicle.world_position(network);
    let heading = vehicle.world_heading(network);
    BevyTransform::from_2d_heading_xz(pos_2d, heading, elevation_y)
}
