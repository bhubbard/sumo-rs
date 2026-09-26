use crate::car_following::model::{CarFollowingModel, LeaderInfo};
use rand::Rng;
use serde::{Deserialize, Serialize};

/// Parameters for the Krauss Car-Following Model (from Eclipse SUMO).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct KraussParameters {
    /// Desired free-flow speed $v_0$ (m/s).
    pub max_speed: f32,
    /// Maximum acceleration $a$ (m/s²).
    pub accel: f32,
    /// Comfortable / maximum deceleration $b$ (m/s², positive).
    pub decel: f32,
    /// Driver reaction time / headway $\tau$ (seconds).
    pub tau: f32,
    /// Imperfection / dawdling parameter $\sigma \in [0.0, 1.0]$.
    /// 0.0 corresponds to a perfect deterministic driver.
    pub sigma: f32,
    /// Standstill minimum gap $s_0$ (meters).
    pub min_gap: f32,
}

impl Default for KraussParameters {
    fn default() -> Self {
        Self {
            max_speed: 13.89, // ~50 km/h (urban speed limit)
            accel: 2.6,       // m/s²
            decel: 4.5,       // m/s²
            tau: 1.0,         // seconds
            sigma: 0.5,       // standard driver dawdling
            min_gap: 2.5,     // meters
        }
    }
}

/// Krauss Car-Following Model.
///
/// Implements safe speed:
/// $$v_{safe} = -g \cdot \tau + \sqrt{(g \cdot \tau)^2 + v_l^2 + 2 \cdot b \cdot g}$$
/// Desired speed:
/// $$v_{des} = \min(v + a \cdot \Delta t, v_0, v_{safe})$$
/// Acceleration with stochastic dawdling:
/// $$v_{next} = \max(0.0, v_{des} - \sigma \cdot \min(a \cdot \Delta t, v_{des}) \cdot \eta)$$
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct KraussModel {
    pub params: KraussParameters,
}

impl KraussModel {
    pub fn new(params: KraussParameters) -> Self {
        Self { params }
    }

    /// Calculate safe velocity according to Krauss formula.
    ///
    /// $v_{safe} = -g \cdot \tau + \sqrt{(g \cdot \tau)^2 + v_l^2 + 2 \cdot b \cdot g}$
    ///
    /// where $g = \max(0.0, \text{gap} - \text{min\_gap})$.
    pub fn calculate_safe_speed(&self, net_gap: f32, leader_speed: f32) -> f32 {
        let g = (net_gap - self.params.min_gap).max(0.0);
        if g <= 0.0 {
            // Gap is at or below minimum standstill gap
            return (leader_speed * 0.5).min(leader_speed);
        }

        let tau = self.params.tau;
        let b = self.params.decel;
        let g_tau = g * tau;
        let radicand = (g_tau * g_tau) + (leader_speed * leader_speed) + (2.0 * b * g);

        if radicand <= 0.0 {
            0.0
        } else {
            (-g_tau + radicand.sqrt()).max(0.0)
        }
    }

    /// Calculate next speed deterministically with an explicit dawdle factor $\eta \in [0.0, 1.0]$.
    pub fn calculate_speed_with_dawdle(
        &self,
        current_speed: f32,
        leader: Option<&LeaderInfo>,
        dt: f32,
        dawdle_factor: f32,
    ) -> f32 {
        let v_safe = match leader {
            Some(l) => self.calculate_safe_speed(l.gap, l.speed),
            None => f32::INFINITY,
        };

        // v_des = min(v + a * dt, v_0, v_safe)
        let v_accel = current_speed + self.params.accel * dt;
        let v_des = v_accel.min(self.params.max_speed).min(v_safe);

        // Stochastic dawdling: reduction by sigma * min(a * dt, v_des) * eta
        let max_dawdle = (self.params.accel * dt).min(v_des);
        let dawdle = self.params.sigma * max_dawdle * dawdle_factor.clamp(0.0, 1.0);

        (v_des - dawdle).max(0.0)
    }

    /// Calculate next acceleration with an explicit dawdle factor $\eta \in [0.0, 1.0]$.
    pub fn calculate_acceleration_with_dawdle(
        &self,
        current_speed: f32,
        leader: Option<&LeaderInfo>,
        dt: f32,
        dawdle_factor: f32,
    ) -> f32 {
        let v_next = self.calculate_speed_with_dawdle(current_speed, leader, dt, dawdle_factor);
        let raw_accel = (v_next - current_speed) / dt;
        raw_accel.clamp(-self.params.decel, self.params.accel)
    }
}

impl CarFollowingModel for KraussModel {
    fn name(&self) -> &'static str {
        "Krauss"
    }

    fn calculate_acceleration(&self, speed: f32, leader: Option<&LeaderInfo>, dt: f32) -> f32 {
        let mut rng = rand::thread_rng();
        let dawdle_factor: f32 = if self.params.sigma > 0.0 {
            rng.gen_range(0.0..1.0)
        } else {
            0.0
        };
        self.calculate_acceleration_with_dawdle(speed, leader, dt, dawdle_factor)
    }

    fn calculate_speed(&self, current_speed: f32, leader: Option<&LeaderInfo>, dt: f32) -> f32 {
        let mut rng = rand::thread_rng();
        let dawdle_factor: f32 = if self.params.sigma > 0.0 {
            rng.gen_range(0.0..1.0)
        } else {
            0.0
        };
        self.calculate_speed_with_dawdle(current_speed, leader, dt, dawdle_factor)
    }

    fn max_acceleration(&self) -> f32 {
        self.params.accel
    }

    fn max_deceleration(&self) -> f32 {
        self.params.decel
    }

    fn desired_speed(&self) -> f32 {
        self.params.max_speed
    }

    fn min_gap(&self) -> f32 {
        self.params.min_gap
    }
}
