use approx::assert_relative_eq;
use sumo_rs::car_following::{
    CarFollowingModel, IdmModel, IdmParameters, KraussModel, KraussParameters, LeaderInfo,
};

#[test]
fn test_krauss_safe_speed_calculation() {
    let params = KraussParameters {
        max_speed: 30.0,
        accel: 2.0,
        decel: 4.0,
        tau: 1.0,
        sigma: 0.0, // deterministic
        min_gap: 2.0,
    };
    let model = KraussModel::new(params);

    // net_gap = 12.0m, min_gap = 2.0m -> g = 10.0m
    // v_l = 10.0 m/s, tau = 1.0 s, b = 4.0 m/s^2
    // g * tau = 10.0
    // radicand = (10)^2 + (10)^2 + 2 * 4 * 10 = 100 + 100 + 80 = 280
    // v_safe = -10 + sqrt(280) = -10 + 16.7332 = 6.7332 m/s
    let safe_speed = model.calculate_safe_speed(12.0, 10.0);
    assert_relative_eq!(safe_speed, -10.0 + (280.0f32).sqrt(), epsilon = 1e-4);

    // If gap <= min_gap (e.g. gap = 1.5m), safe speed is reduced to half leader speed or less
    let stopped_safe = model.calculate_safe_speed(1.5, 0.0);
    assert_relative_eq!(stopped_safe, 0.0, epsilon = 1e-4);
}

#[test]
fn test_krauss_free_flow_acceleration() {
    let params = KraussParameters {
        max_speed: 20.0,
        accel: 2.5,
        decel: 4.0,
        tau: 1.0,
        sigma: 0.0,
        min_gap: 2.5,
    };
    let model = KraussModel::new(params);

    // Without leader, accelerate by a * dt
    let dt = 1.0;
    let v0 = 0.0;
    let v1 = model.calculate_speed(v0, None, dt);
    assert_relative_eq!(v1, 2.5, epsilon = 1e-4);

    let v2 = model.calculate_speed(v1, None, dt);
    assert_relative_eq!(v2, 5.0, epsilon = 1e-4);

    // Should cap at max_speed
    let mut v = 0.0;
    for _ in 0..20 {
        v = model.calculate_speed(v, None, dt);
    }
    assert_relative_eq!(v, 20.0, epsilon = 1e-4);
}

#[test]
fn test_krauss_dawdling_imperfection() {
    let params = KraussParameters {
        max_speed: 20.0,
        accel: 2.0,
        decel: 4.0,
        tau: 1.0,
        sigma: 0.5, // 50% dawdling
        min_gap: 2.5,
    };
    let model = KraussModel::new(params);
    let dt = 1.0;

    // With dawdle_factor = 0 (perfect), v_des = 2.0
    let v_perf = model.calculate_speed_with_dawdle(0.0, None, dt, 0.0);
    assert_relative_eq!(v_perf, 2.0, epsilon = 1e-4);

    // With dawdle_factor = 1.0 (max dawdle), reduction is sigma * min(a * dt, v_des) = 0.5 * 2.0 = 1.0
    // so v = 2.0 - 1.0 = 1.0
    let v_dawdle = model.calculate_speed_with_dawdle(0.0, None, dt, 1.0);
    assert_relative_eq!(v_dawdle, 1.0, epsilon = 1e-4);
}

#[test]
fn test_idm_desired_gap() {
    let params = IdmParameters {
        desired_speed: 30.0,
        time_headway: 1.5,
        min_gap: 2.0,
        max_accel: 1.4,
        comfortable_decel: 2.0,
        delta: 4.0,
        max_decel: 9.0,
    };
    let model = IdmModel::new(params);

    // At standstill v = 0, delta_v = 0: desired gap = s0 = 2.0m
    let s_star_zero = model.desired_gap(0.0, 0.0);
    assert_relative_eq!(s_star_zero, 2.0, epsilon = 1e-4);

    // At v = 10 m/s with equal leader speed delta_v = 0:
    // s* = s0 + v * T = 2.0 + 10.0 * 1.5 = 17.0m
    let s_star_cruise = model.desired_gap(10.0, 0.0);
    assert_relative_eq!(s_star_cruise, 17.0, epsilon = 1e-4);

    // Closing in: v = 10 m/s, delta_v = 5 m/s (ego 10 m/s, leader 5 m/s)
    // dynamic term = (10 * 5) / (2 * sqrt(1.4 * 2.0)) = 50 / (2 * sqrt(2.8)) = 50 / 3.3466 = 14.94m
    // s* = 2.0 + 15.0 + 14.94 = 31.94m
    let s_star_closing = model.desired_gap(10.0, 5.0);
    assert!(s_star_closing > 31.0 && s_star_closing < 33.0);
}

#[test]
fn test_idm_acceleration_free_road() {
    let params = IdmParameters {
        desired_speed: 20.0,
        time_headway: 1.5,
        min_gap: 2.0,
        max_accel: 2.0,
        comfortable_decel: 2.0,
        delta: 4.0,
        max_decel: 9.0,
    };
    let model = IdmModel::new(params);

    // At v = 0 on free road: a * [1 - (0)^4] = a = 2.0 m/s^2
    let a_start = model.calculate_acceleration(0.0, None, 0.1);
    assert_relative_eq!(a_start, 2.0, epsilon = 1e-4);

    // At v = 20 on free road: a * [1 - (1)^4] = 0.0 m/s^2
    let a_top = model.calculate_acceleration(20.0, None, 0.1);
    assert_relative_eq!(a_top, 0.0, epsilon = 1e-4);

    // At v = 25 (overspeed): negative acceleration
    let a_over = model.calculate_acceleration(25.0, None, 0.1);
    assert!(a_over < 0.0);
}

#[test]
fn test_idm_car_following_braking() {
    let params = IdmParameters::default();
    let model = IdmModel::new(params);

    // Ego at 15 m/s, closing in on stopped leader at 0 m/s with gap 10m
    let leader = LeaderInfo::new(10.0, 0.0);
    let accel = model.calculate_acceleration(15.0, Some(&leader), 0.1);

    // Must be braking hard
    assert!(accel < -1.5);
}
