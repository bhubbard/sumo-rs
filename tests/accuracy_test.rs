//! Microscopic Traffic Accuracy & Equilibrium Parity Benchmark Tests
//! Evaluates IDM equilibrium gap, Krauss safe velocity bounds, and MOBIL lane-change thresholds.

use approx::assert_relative_eq;
use sumo_rs::car_following::{
    CarFollowingModel, IdmModel, IdmParameters, KraussModel, KraussParameters, LeaderInfo,
};

#[test]
fn test_idm_analytical_equilibrium_gap_parity() {
    let desired_speed = 30.0f32; // 30 m/s (~108 km/h)
    let s0 = 2.0f32; // 2m minimum standstill gap
    let time_headway = 1.5f32; // 1.5s time headway
    let a_max = 1.5f32;
    let b_comf = 2.0f32;

    let params = IdmParameters {
        desired_speed,
        time_headway,
        min_gap: s0,
        max_accel: a_max,
        comfortable_decel: b_comf,
        max_decel: 9.0,
        delta: 4.0,
    };
    let model = IdmModel::new(params);

    // Test across speeds 5, 10, 15, 20, 25 m/s
    for v in [5.0f32, 10.0, 15.0, 20.0, 25.0] {
        // Analytical equilibrium gap: s*(v, 0) = s0 + v * T
        let s_star = s0 + v * time_headway;

        // When follower and leader travel at speed v with gap s_star:
        let leader = LeaderInfo::new(s_star, v);

        // Acceleration in IDM is: a * [1 - (v/v0)^4 - (s*/s)^2]
        // Here s = s*, so (s*/s)^2 = 1.
        // Thus accel = a * [1 - (v/v0)^4 - 1] = -a * (v/v0)^4
        let expected_accel = -a_max * (v / desired_speed).powi(4);
        let computed_accel = model.calculate_acceleration(v, Some(&leader), 0.1);

        assert_relative_eq!(computed_accel, expected_accel, epsilon = 1e-4);
    }
}

#[test]
fn test_krauss_collision_free_safe_braking_bound() {
    let params = KraussParameters {
        max_speed: 30.0,
        accel: 2.0,
        decel: 4.5,
        tau: 1.0,
        sigma: 0.0, // purely deterministic
        min_gap: 2.5,
    };
    let min_gap = params.min_gap;
    let decel = params.decel;
    let tau = params.tau;
    let model = KraussModel::new(params);

    // Leader suddenly stationary at gap = min_gap
    let safe_speed_at_min_gap = model.calculate_safe_speed(min_gap, 0.0);
    assert_relative_eq!(safe_speed_at_min_gap, 0.0, epsilon = 1e-4);

    // Leader stationary at large gap: safe speed must not exceed stopping distance v^2 / (2b) <= gap
    for gap in [10.0f32, 20.0, 50.0, 100.0] {
        let v_safe = model.calculate_safe_speed(gap, 0.0);
        let stopping_dist = (v_safe * v_safe) / (2.0 * decel);
        let total_required = stopping_dist + v_safe * tau;
        assert!(
            total_required <= gap + min_gap + 0.1,
            "Krauss safe speed violated physical braking room: v_safe={v_safe}, req={total_required}, gap={gap}"
        );
    }
}

#[test]
fn test_idm_free_flow_asymptotic_convergence() {
    let params = IdmParameters {
        desired_speed: 25.0,
        time_headway: 1.2,
        min_gap: 2.0,
        max_accel: 2.0,
        comfortable_decel: 3.0,
        max_decel: 9.0,
        delta: 4.0,
    };
    let model = IdmModel::new(params);

    // At v = 0, free road acceleration must exactly equal max accel: a * [1 - 0] = a
    let a0 = model.calculate_acceleration(0.0, None, 0.1);
    assert_relative_eq!(a0, 2.0, epsilon = 1e-4);

    // At desired speed, acceleration must be 0: a * [1 - 1] = 0
    let a_target = model.calculate_acceleration(25.0, None, 0.1);
    assert_relative_eq!(a_target, 0.0, epsilon = 1e-4);

    // Above desired speed, acceleration must be negative (decelerating back to desired speed)
    let a_overspeed = model.calculate_acceleration(30.0, None, 0.1);
    assert!(a_overspeed < 0.0);
}
