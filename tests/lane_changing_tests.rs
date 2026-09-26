use sumo_rs::lane_changing::{MobilContext, MobilModel, MobilParameters};
use sumo_rs::types::LaneChangeDirection;

#[test]
fn test_mobil_incentive_overtake_slow_leader() {
    let params = MobilParameters {
        politeness: 0.2,
        switching_threshold: 0.2,
        safe_decel: 4.0,
        bias_right: 0.0,
        min_clearance: 2.0,
    };
    let mobil = MobilModel::new(params);

    // Current lane: stuck behind slow truck, ego_accel = -1.0
    // Target lane: empty, ego can accelerate at +1.5
    // Target follower: far enough, decelerates only slightly from +0.5 to +0.3
    // Old follower: speeds up from +0.0 to +1.0
    let ctx = MobilContext {
        ego_accel_curr: -1.0,
        ego_accel_new: 1.5,
        new_follower_accel_curr: 0.5,
        new_follower_accel_new: 0.3,
        old_follower_accel_curr: 0.0,
        old_follower_accel_new: 1.0,
        leader_accel_delta: 0.0,
        gap_to_new_follower: Some(25.0),
        gap_to_new_leader: Some(40.0),
        direction: LaneChangeDirection::Left,
    };

    let decision = mobil.evaluate(&ctx);
    assert!(decision.is_safe);
    assert!(decision.should_change);
    // ego_advantage = 2.5
    // other_advantage = (0.3 - 0.5) + (1.0 - 0.0) = -0.2 + 1.0 = 0.8
    // total incentive = 2.5 + 0.2 * 0.8 = 2.66 > 0.2
    assert!(decision.incentive > 2.0);
}

#[test]
fn test_mobil_safety_criterion_blocks_dangerous_cut_in() {
    let params = MobilParameters {
        politeness: 0.1,
        switching_threshold: 0.2,
        safe_decel: 4.0, // max safe braking is 4 m/s^2
        bias_right: 0.0,
        min_clearance: 2.0,
    };
    let mobil = MobilModel::new(params);

    // Cutting in right in front of a fast car, forcing them to brake at -6.0 m/s^2
    let ctx = MobilContext {
        ego_accel_curr: 0.0,
        ego_accel_new: 1.0,
        new_follower_accel_curr: 0.0,
        new_follower_accel_new: -6.0, // Exceeds safe decel limit!
        old_follower_accel_curr: 0.0,
        old_follower_accel_new: 0.0,
        leader_accel_delta: 0.0,
        gap_to_new_follower: Some(5.0),
        gap_to_new_leader: Some(30.0),
        direction: LaneChangeDirection::Right,
    };

    let decision = mobil.evaluate(&ctx);
    assert!(!decision.is_safe);
    assert!(!decision.should_change);
    assert!(
        decision
            .reason
            .contains("Follower braking exceeds safe limit")
    );
}

#[test]
fn test_mobil_politeness_prevents_inconvenience() {
    let params = MobilParameters {
        politeness: 0.8, // highly polite driver
        switching_threshold: 0.2,
        safe_decel: 4.0,
        bias_right: 0.0,
        min_clearance: 2.0,
    };
    let mobil = MobilModel::new(params);

    // Ego gains slight advantage (+0.3 m/s^2), but causes new follower to brake by -1.5 m/s^2
    let ctx = MobilContext {
        ego_accel_curr: 0.0,
        ego_accel_new: 0.3,
        new_follower_accel_curr: 0.5,
        new_follower_accel_new: -1.0, // diff = -1.5
        old_follower_accel_curr: 0.0,
        old_follower_accel_new: 0.0,
        leader_accel_delta: 0.0,
        gap_to_new_follower: Some(15.0),
        gap_to_new_leader: Some(25.0),
        direction: LaneChangeDirection::Left,
    };

    let decision = mobil.evaluate(&ctx);
    // ego_gain = 0.3
    // others = -1.5
    // incentive = 0.3 + 0.8 * (-1.5) = 0.3 - 1.2 = -0.9 < threshold
    assert!(decision.is_safe);
    assert!(!decision.should_change);
}
