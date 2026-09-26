pub mod idm;
pub mod krauss;
pub mod model;

pub use idm::{IdmModel, IdmParameters};
pub use krauss::{KraussModel, KraussParameters};
pub use model::{CarFollowingModel, LeaderInfo};

use serde::{Deserialize, Serialize};

/// Enum encapsulating available car-following models.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum DriverModel {
    Krauss(KraussModel),
    Idm(IdmModel),
}

impl DriverModel {
    pub fn krauss_default() -> Self {
        Self::Krauss(KraussModel::new(KraussParameters::default()))
    }

    pub fn idm_default() -> Self {
        Self::Idm(IdmModel::new(IdmParameters::default()))
    }

    pub fn calculate_acceleration(&self, speed: f32, leader: Option<&LeaderInfo>, dt: f32) -> f32 {
        CarFollowingModel::calculate_acceleration(self, speed, leader, dt)
    }

    pub fn calculate_speed(&self, current_speed: f32, leader: Option<&LeaderInfo>, dt: f32) -> f32 {
        CarFollowingModel::calculate_speed(self, current_speed, leader, dt)
    }

    pub fn desired_speed(&self) -> f32 {
        CarFollowingModel::desired_speed(self)
    }

    pub fn max_acceleration(&self) -> f32 {
        CarFollowingModel::max_acceleration(self)
    }

    pub fn max_deceleration(&self) -> f32 {
        CarFollowingModel::max_deceleration(self)
    }

    pub fn min_gap(&self) -> f32 {
        CarFollowingModel::min_gap(self)
    }
}

impl CarFollowingModel for DriverModel {
    fn name(&self) -> &'static str {
        match self {
            Self::Krauss(m) => m.name(),
            Self::Idm(m) => m.name(),
        }
    }

    fn calculate_acceleration(&self, speed: f32, leader: Option<&LeaderInfo>, dt: f32) -> f32 {
        match self {
            Self::Krauss(m) => m.calculate_acceleration(speed, leader, dt),
            Self::Idm(m) => m.calculate_acceleration(speed, leader, dt),
        }
    }

    fn calculate_speed(&self, current_speed: f32, leader: Option<&LeaderInfo>, dt: f32) -> f32 {
        match self {
            Self::Krauss(m) => m.calculate_speed(current_speed, leader, dt),
            Self::Idm(m) => m.calculate_speed(current_speed, leader, dt),
        }
    }

    fn max_acceleration(&self) -> f32 {
        match self {
            Self::Krauss(m) => m.max_acceleration(),
            Self::Idm(m) => m.max_acceleration(),
        }
    }

    fn max_deceleration(&self) -> f32 {
        match self {
            Self::Krauss(m) => m.max_deceleration(),
            Self::Idm(m) => m.max_deceleration(),
        }
    }

    fn desired_speed(&self) -> f32 {
        match self {
            Self::Krauss(m) => m.desired_speed(),
            Self::Idm(m) => m.desired_speed(),
        }
    }

    fn min_gap(&self) -> f32 {
        match self {
            Self::Krauss(m) => m.min_gap(),
            Self::Idm(m) => m.min_gap(),
        }
    }
}
