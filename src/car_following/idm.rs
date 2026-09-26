use crate::car_following::model::{CarFollowingModel, LeaderInfo};
use serde::{Deserialize, Serialize};

/// Parameters for the Intelligent Driver Model (IDM).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IdmParameters {
    /// Desired velocity $v_0$ in free traffic (m/s).
    pub desired_speed: f32,
    /// Safe time headway $T$ (seconds).
    pub time_headway: f32,
    /// Minimum bumper-to-bumper distance at standstill $s_0$ (meters).
    pub min_gap: f32,
    /// Maximum acceleration $a$ (m/s²).
    pub max_accel: f32,
    /// Comfortable deceleration $b$ (m/s², positive).
    pub comfortable_decel: f32,
    /// Acceleration exponent $\delta$ (typically 4.0).
    pub delta: f32,
    /// Maximum emergency deceleration limit $b_{max}$ (m/s², positive).
    pub max_decel: f32,
}

impl Default for IdmParameters {
    fn default() -> Self {
        Self {
            desired_speed: 13.89,   // ~50 km/h
            time_headway: 1.5,      // 1.5 s
            min_gap: 2.0,           // 2.0 m
            max_accel: 1.4,         // m/s²
            comfortable_decel: 2.0, // m/s²
            delta: 4.0,             // standard IDM exponent
            max_decel: 9.0,         // emergency braking threshold
        }
    }
}

/// Intelligent Driver Model (Treiber, Hennecke, and Helbing, 2000).
///
/// Implements acceleration:
/// $$\dot{v} = a \left[ 1 - \left(\frac{v}{v_0}\right)^\delta - \left(\frac{s^*(v, \Delta v)}{s}\right)^2 \right]$$
/// with dynamic desired distance:
/// $$s^*(v, \Delta v) = s_0 + v \cdot T + \frac{v \cdot \Delta v}{2 \sqrt{a \cdot b}}$$
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IdmModel {
    pub params: IdmParameters,
}

impl IdmModel {
    pub fn new(params: IdmParameters) -> Self {
        Self { params }
    }

    /// Calculate the dynamic desired gap $s^*(v, \Delta v)$.
    ///
    /// * `v` - ego vehicle speed (m/s).
    /// * `delta_v` - approach rate $v - v_l$ (m/s).
    pub fn desired_gap(&self, v: f32, delta_v: f32) -> f32 {
        let v_pos = v.max(0.0);
        let s0 = self.params.min_gap;
        let t = self.params.time_headway;
        let a = self.params.max_accel;
        let b = self.params.comfortable_decel;

        let dynamic_term = (v_pos * delta_v) / (2.0 * (a * b).sqrt());
        (s0 + v_pos * t + dynamic_term).max(s0)
    }

    /// Calculate instantaneous acceleration $\dot{v}$.
    pub fn acceleration_continuous(&self, v: f32, leader: Option<&LeaderInfo>) -> f32 {
        let v_pos = v.max(0.0);
        let v0 = self.params.desired_speed.max(0.1);
        let a = self.params.max_accel;
        let delta = self.params.delta;

        // Free-road term: 1 - (v / v0)^delta
        let free_road_term = 1.0 - (v_pos / v0).powf(delta);

        // Interaction term: (s* / s)^2
        let interaction_term = match leader {
            Some(l) => {
                let delta_v = v_pos - l.speed;
                let s_star = self.desired_gap(v_pos, delta_v);
                let actual_gap = l.gap.max(0.01);
                (s_star / actual_gap).powi(2)
            }
            None => 0.0,
        };

        let raw_accel = a * (free_road_term - interaction_term);
        raw_accel.clamp(-self.params.max_decel, self.params.max_accel)
    }
}

impl CarFollowingModel for IdmModel {
    fn name(&self) -> &'static str {
        "IDM"
    }

    fn calculate_acceleration(&self, speed: f32, leader: Option<&LeaderInfo>, _dt: f32) -> f32 {
        self.acceleration_continuous(speed, leader)
    }

    fn calculate_speed(&self, current_speed: f32, leader: Option<&LeaderInfo>, dt: f32) -> f32 {
        let accel = self.calculate_acceleration(current_speed, leader, dt);
        (current_speed + accel * dt).max(0.0)
    }

    fn max_acceleration(&self) -> f32 {
        self.params.max_accel
    }

    fn max_deceleration(&self) -> f32 {
        self.params.max_decel
    }

    fn desired_speed(&self) -> f32 {
        self.params.desired_speed
    }

    fn min_gap(&self) -> f32 {
        self.params.min_gap
    }
}
