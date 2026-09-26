use glam::Vec2;
use sumo_rs::intersection::{
    AllWayStopManager, ConflictZoneManager, RightOfWayEvaluator, SignalColor,
    TrafficLightController,
};
use sumo_rs::network::builder::NetworkBuilder;
use sumo_rs::network::graph::Network;
use sumo_rs::network::node::JunctionType;
use sumo_rs::types::{ConnectionId, NodeId, VehicleId};

#[test]
fn test_traffic_light_controller_cycle() {
    let ns_conns = vec![
        ConnectionId::new("conn_S_Straight"),
        ConnectionId::new("conn_N_Straight"),
    ];
    let ew_conns = vec![
        ConnectionId::new("conn_E_Straight"),
        ConnectionId::new("conn_W_Straight"),
    ];

    let mut tl = TrafficLightController::standard_4way(
        "TL_Center".into(),
        &ns_conns,
        &ew_conns,
        20.0, // green
        3.0,  // yellow
        2.0,  // all-red
    );

    assert_eq!(tl.cycle_length(), 20.0 + 3.0 + 2.0 + 20.0 + 3.0 + 2.0); // 50.0s

    // Initially NS Green
    assert_eq!(tl.get_connection_signal(&ns_conns[0]), SignalColor::Green);
    assert_eq!(tl.get_connection_signal(&ew_conns[0]), SignalColor::Red);

    // Step 20s -> NS Yellow
    tl.step(20.0);
    assert_eq!(tl.get_connection_signal(&ns_conns[0]), SignalColor::Yellow);

    // Step 3s -> Clearance NS (all red)
    tl.step(3.0);
    assert_eq!(tl.get_connection_signal(&ns_conns[0]), SignalColor::Red);
    assert_eq!(tl.get_connection_signal(&ew_conns[0]), SignalColor::Red);

    // Step 2s -> EW Green
    tl.step(2.0);
    assert_eq!(tl.get_connection_signal(&ns_conns[0]), SignalColor::Red);
    assert_eq!(tl.get_connection_signal(&ew_conns[0]), SignalColor::Green);
}

#[test]
fn test_left_turn_yield_to_oncoming() {
    let mut network = Network::new();
    NetworkBuilder::build_four_way_intersection(
        &mut network,
        NodeId::new("J0"),
        Vec2::ZERO,
        50.0,
        13.89,
        JunctionType::Priority,
    );

    let left_conn_id = ConnectionId::new("conn_S_TurnLeft");
    let oncoming_straight_conn_id = ConnectionId::new("conn_N_Straight");

    // Ego turning left, 10m from intersection at 8 m/s
    // Oncoming straight vehicle is 15m away at 10 m/s
    let approaching_conflicts = vec![(
        VehicleId(2),
        oncoming_straight_conn_id.clone(),
        15.0, // dist
        10.0, // speed
    )];

    let must_yield = RightOfWayEvaluator::should_yield(
        &network,
        &left_conn_id,
        10.0,
        8.0,
        &[oncoming_straight_conn_id],
        &approaching_conflicts,
    );

    assert!(
        must_yield,
        "Left-turning vehicle must yield to approaching oncoming straight vehicle"
    );
}

#[test]
fn test_all_way_stop_fifo_order() {
    let mut stop_mgr = AllWayStopManager::new();
    let veh_1 = VehicleId(1);
    let veh_2 = VehicleId(2);

    // Vehicle 1 arrives at stop line and stops
    let auth1 = stop_mgr.update_vehicle(veh_1, 0.0, 1.0, 0.5);
    assert!(!auth1, "Vehicle 1 cannot immediately go without dwell time");

    // Vehicle 2 arrives at stop line 0.5s later
    let auth2 = stop_mgr.update_vehicle(veh_2, 0.0, 1.0, 0.5);
    assert!(!auth2);

    // Vehicle 1 completes required stop time (1.2s total)
    let auth1_ready = stop_mgr.update_vehicle(veh_1, 0.0, 1.0, 0.8);
    assert!(
        auth1_ready,
        "Vehicle 1 is first in FIFO and satisfied dwell time"
    );

    // Vehicle 2 also satisfied dwell time, but Vehicle 1 hasn't cleared yet
    let _ = stop_mgr.update_vehicle(veh_2, 0.0, 1.0, 1.0);
    assert_eq!(stop_mgr.stopped_queue.front(), Some(&veh_1));

    // Vehicle 1 enters and clears intersection
    stop_mgr.on_vehicle_entered(&veh_1);

    // Now Vehicle 2 is authorized to go!
    let auth2_ready = stop_mgr.update_vehicle(veh_2, 0.0, 1.0, 0.1);
    assert!(
        auth2_ready,
        "Vehicle 2 is now authorized after Vehicle 1 cleared"
    );
}

#[test]
fn test_conflict_zone_reservation() {
    let mut conflict_mgr = ConflictZoneManager::new();
    let v1 = VehicleId(1);
    let v2 = VehicleId(2);

    let conn_a = ConnectionId::new("conn_NS");
    let conn_b = ConnectionId::new("conn_EW");
    let conflicts = vec![conn_b.clone()];

    // Vehicle 1 reserves conn_a from t=5.0 to t=8.0
    assert!(conflict_mgr.can_reserve(v1, &conn_a, &conflicts, 5.0, 8.0));
    conflict_mgr.make_reservation(v1, conn_a.clone(), 5.0, 8.0);

    // Vehicle 2 wants to cross intersecting conn_b at t=6.0 to t=9.0 -> Must be blocked!
    assert!(!conflict_mgr.can_reserve(v2, &conn_b, std::slice::from_ref(&conn_a), 6.0, 9.0));

    // Vehicle 2 after Vehicle 1 cleared: t=12.0 to t=15.0 -> Allowed!
    assert!(conflict_mgr.can_reserve(v2, &conn_b, std::slice::from_ref(&conn_a), 12.0, 15.0));
}
