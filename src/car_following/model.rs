use serde::{Deserialize, Serialize};

/// Information about a leading vehicle on the road.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct LeaderInfo {
    /// Net gap between the front bumper of the ego vehicle and the rear bumper of the leader (in meters).
    pub gap: f32,
    /// Current speed of the leader vehicle (in m/s).
    pub speed: f32,
    /// Length of the leader vehicle (in meters).
    pub length: f32,
    /// Acceleration of the leader vehicle (in m/s²), if known.
    pub acceleration: f32,
}

impl LeaderInfo {
    pub fn new(gap: f32, speed: f32) -> Self {
        Self {
            gap,
            speed,
            length: 5.0,
            acceleration: 0.0,
        }
    }

    pub fn with_length(mut self, length: f32) -> Self {
        self.length = length;
        self
    }

    pub fn with_acceleration(mut self, acceleration: f32) -> Self {
        self.acceleration = acceleration;
        self
    }
}

/// Common trait for microscopic car-following models.
pub trait CarFollowingModel: Send + Sync {
    /// Name of the car-following model.
    fn name(&self) -> &'static str;

    /// Calculate the acceleration (in m/s²) for the ego vehicle.
    ///
    /// * `speed` - current speed of the ego vehicle (m/s).
    /// * `leader` - optional leading vehicle info.
    /// * `dt` - simulation timestep (seconds).
    fn calculate_acceleration(&self, speed: f32, leader: Option<&LeaderInfo>, dt: f32) -> f32;

    /// Calculate the resulting speed (in m/s) after timestep `dt`.
    fn calculate_speed(&self, current_speed: f32, leader: Option<&LeaderInfo>, dt: f32) -> f32 {
        let accel = self.calculate_acceleration(current_speed, leader, dt);
        (current_speed + accel * dt).max(0.0)
    }

    /// Maximum acceleration capability (m/s²).
    fn max_acceleration(&self) -> f32;

    /// Maximum deceleration capability (positive value, m/s²).
    fn max_deceleration(&self) -> f32;

    /// Desired free-flow speed (m/s).
    fn desired_speed(&self) -> f32;

    /// Minimum standstill gap (meters).
    fn min_gap(&self) -> f32;
}
