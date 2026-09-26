use crate::types::LaneChangeDirection;
use serde::{Deserialize, Serialize};

/// MOBIL (Minimizing Overall Braking Induced by Lane Changes) configuration parameters.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MobilParameters {
    /// Politeness factor $p \in [0.0, 1.0]$.
    /// $p = 0.0$ is completely egoistic.
    /// $p = 1.0$ is fully altruistic.
    /// Typical values range from 0.2 to 0.5.
    pub politeness: f32,

    /// Acceleration threshold $\Delta a_{th}$ (m/s²) required to justify a lane change.
    /// Prevents oscillating or unnecessary lane changes.
    pub switching_threshold: f32,

    /// Maximum safe deceleration $b_{safe}$ (m/s², positive value).
    /// Follower deceleration must satisfy $\tilde{a}_{follower} \ge -b_{safe}$.
    pub safe_decel: f32,

    /// Asymmetric lane bias $a_{bias}$ (m/s²).
    /// Positive bias favors moving right (keep-right rule).
    pub bias_right: f32,

    /// Minimum clearance distance (front and back) required to consider changing lanes.
    pub min_clearance: f32,
}

impl Default for MobilParameters {
    fn default() -> Self {
        Self {
            politeness: 0.3,
            switching_threshold: 0.2, // m/s²
            safe_decel: 4.0,          // m/s²
            bias_right: 0.1,          // slight incentive to keep right
            min_clearance: 2.0,       // 2 meters
        }
    }
}

/// Context for evaluating a MOBIL lane change decision.
#[derive(Debug, Clone)]
pub struct MobilContext {
    /// Current acceleration of ego vehicle in current lane: $a_{curr}$
    pub ego_accel_curr: f32,
    /// Projected acceleration of ego vehicle in target lane: $a_{new}$
    pub ego_accel_new: f32,

    /// Current acceleration of new follower in target lane before ego changes: $a_{curr,follower}$
    pub new_follower_accel_curr: f32,
    /// Projected acceleration of new follower in target lane after ego changes: $a_{new,follower}$
    pub new_follower_accel_new: f32,

    /// Current acceleration of current follower in ego's lane before ego leaves: $a_{curr,old\_follower}$
    pub old_follower_accel_curr: f32,
    /// Projected acceleration of current follower in ego's lane after ego leaves: $a_{new,old\_follower}$
    pub old_follower_accel_new: f32,

    /// Optional change in leader acceleration in target lane ($a_{new,leader} - a_{curr,leader}$)
    pub leader_accel_delta: f32,

    /// Gap between ego and new follower in target lane (meters). None if no follower.
    pub gap_to_new_follower: Option<f32>,
    /// Gap between ego and new leader in target lane (meters). None if no leader.
    pub gap_to_new_leader: Option<f32>,

    /// Direction of proposed change.
    pub direction: LaneChangeDirection,
}

/// Result of evaluating a MOBIL lane change query.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MobilDecision {
    /// Recommended direction.
    pub direction: LaneChangeDirection,
    /// Incentive value (overall acceleration advantage).
    pub incentive: f32,
    /// Whether the safety criterion is satisfied.
    pub is_safe: bool,
    /// Whether both safety and incentive criteria are satisfied.
    pub should_change: bool,
    /// Detailed reason for the decision.
    pub reason: String,
}

/// MOBIL lane changing evaluator.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MobilModel {
    pub params: MobilParameters,
}

impl MobilModel {
    pub fn new(params: MobilParameters) -> Self {
        Self { params }
    }

    /// Evaluate the MOBIL incentive and safety criteria.
    ///
    /// # Incentive Criterion:
    /// $$a_{new} - a_{curr} + p \cdot (a_{new,follower} - a_{curr,follower} + a_{new,leader} - a_{curr,leader} + a_{new,old\_follower} - a_{curr,old\_follower}) > \Delta a_{th} \pm a_{bias}$$
    ///
    /// # Safety Criterion:
    /// $$a_{new,follower} \ge -b_{safe} \quad \text{and} \quad a_{new} \ge -b_{safe}$$
    pub fn evaluate(&self, ctx: &MobilContext) -> MobilDecision {
        if ctx.direction == LaneChangeDirection::Stay {
            return MobilDecision {
                direction: LaneChangeDirection::Stay,
                incentive: 0.0,
                is_safe: true,
                should_change: false,
                reason: "Staying in current lane".to_string(),
            };
        }

        // 1. Check physical clearances
        if let Some(gap_leader) = ctx.gap_to_new_leader
            && gap_leader < self.params.min_clearance
        {
            return MobilDecision {
                direction: ctx.direction,
                incentive: 0.0,
                is_safe: false,
                should_change: false,
                reason: format!("Insufficient gap to target leader: {:.2}m", gap_leader),
            };
        }

        if let Some(gap_follower) = ctx.gap_to_new_follower
            && gap_follower < self.params.min_clearance
        {
            return MobilDecision {
                direction: ctx.direction,
                incentive: 0.0,
                is_safe: false,
                should_change: false,
                reason: format!("Insufficient gap to target follower: {:.2}m", gap_follower),
            };
        }

        // 2. Safety criterion:
        // Deceleration of new immediate follower does not exceed maximum safe braking b_safe
        let new_follower_safe = ctx.new_follower_accel_new >= -self.params.safe_decel;
        let ego_safe = ctx.ego_accel_new >= -self.params.safe_decel;
        let is_safe = new_follower_safe && ego_safe;

        if !is_safe {
            let reason = if !new_follower_safe {
                format!(
                    "Follower braking exceeds safe limit: {:.2} m/s² < -{:.2}",
                    ctx.new_follower_accel_new, self.params.safe_decel
                )
            } else {
                format!(
                    "Ego braking in target lane exceeds safe limit: {:.2} m/s² < -{:.2}",
                    ctx.ego_accel_new, self.params.safe_decel
                )
            };
            return MobilDecision {
                direction: ctx.direction,
                incentive: 0.0,
                is_safe: false,
                should_change: false,
                reason,
            };
        }

        // 3. Incentive criterion:
        let ego_advantage = ctx.ego_accel_new - ctx.ego_accel_curr;
        let new_follower_diff = ctx.new_follower_accel_new - ctx.new_follower_accel_curr;
        let old_follower_diff = ctx.old_follower_accel_new - ctx.old_follower_accel_curr;
        let leader_diff = ctx.leader_accel_delta;

        let other_advantage = new_follower_diff + old_follower_diff + leader_diff;
        let mut incentive = ego_advantage + self.params.politeness * other_advantage;

        // Apply lane bias:
        // Moving right gains bias_right; moving left loses bias_right (keep-right rule)
        match ctx.direction {
            LaneChangeDirection::Right => incentive += self.params.bias_right,
            LaneChangeDirection::Left => incentive -= self.params.bias_right,
            LaneChangeDirection::Stay => {}
        }

        let should_change = incentive > self.params.switching_threshold;
        let reason = if should_change {
            format!(
                "Incentive {:.2} exceeds threshold {:.2}",
                incentive, self.params.switching_threshold
            )
        } else {
            format!(
                "Incentive {:.2} does not exceed threshold {:.2}",
                incentive, self.params.switching_threshold
            )
        };

        MobilDecision {
            direction: ctx.direction,
            incentive,
            is_safe,
            should_change,
            reason,
        }
    }
}
