use glam::Vec2;
use sumo_rs::car_following::{DriverModel, IdmModel, IdmParameters, KraussModel, KraussParameters};
use sumo_rs::lane_changing::MobilParameters;
use sumo_rs::network::builder::NetworkBuilder;
use sumo_rs::network::graph::Network;
use sumo_rs::network::node::JunctionType;
use sumo_rs::simulation::{TrafficSimulation, Vehicle};
use sumo_rs::types::{ConnectionId, EdgeId, LaneId, NodeId, VehicleId};

#[test]
fn test_platoon_car_following_no_collisions() {
    let mut network = Network::new();
    NetworkBuilder::build_straight_road(
        &mut network,
        NodeId::new("N0"),
        NodeId::new("N1"),
        EdgeId::new("E0"),
        Vec2::new(0.0, 0.0),
        Vec2::new(1000.0, 0.0),
        1,
        25.0,
    );

    let mut sim = TrafficSimulation::new(network);
    let lane_id = LaneId::new("E0_0");

    // Spawn 5 vehicles in platoon with IDM
    for i in 0..5 {
        let pos = (4 - i) as f32 * 30.0; // 120m, 90m, 60m, 30m, 0m
        let idm_params = IdmParameters {
            desired_speed: 20.0,
            ..Default::default()
        };
        let model = DriverModel::Idm(IdmModel::new(idm_params));
        let veh = Vehicle::new(VehicleId(i as u64), lane_id.clone(), pos, 15.0, model);
        sim.add_vehicle(veh).unwrap();
    }

    // Step simulation for 100 seconds (1000 steps at dt = 0.1)
    let dt = 0.1;
    for _ in 0..1000 {
        sim.step(dt);
        let collisions = sim.check_collisions();
        assert!(
            collisions.is_empty(),
            "Collision detected in platoon: {:?}",
            collisions
        );
    }

    assert!(sim.vehicle_count() == 5);
}

#[test]
fn test_krauss_simulation_no_collisions() {
    let mut network = Network::new();
    NetworkBuilder::build_straight_road(
        &mut network,
        NodeId::new("N0"),
        NodeId::new("N1"),
        EdgeId::new("E0"),
        Vec2::new(0.0, 0.0),
        Vec2::new(1000.0, 0.0),
        1,
        25.0,
    );

    let mut sim = TrafficSimulation::new(network);
    let lane_id = LaneId::new("E0_0");

    for i in 0..4 {
        let pos = (3 - i) as f32 * 35.0;
        let krauss_params = KraussParameters {
            max_speed: 20.0,
            sigma: 0.3, // stochastic imperfection
            ..Default::default()
        };
        let model = DriverModel::Krauss(KraussModel::new(krauss_params));
        let veh = Vehicle::new(VehicleId(i as u64), lane_id.clone(), pos, 10.0, model);
        sim.add_vehicle(veh).unwrap();
    }

    let dt = 0.1;
    for _ in 0..500 {
        sim.step(dt);
        let collisions = sim.check_collisions();
        assert!(
            collisions.is_empty(),
            "Collision detected in Krauss platoon: {:?}",
            collisions
        );
    }
}

#[test]
fn test_highway_mobil_lane_change_execution() {
    let mut network = Network::new();
    // 2-lane road: lane 0 (right), lane 1 (left)
    NetworkBuilder::build_straight_road(
        &mut network,
        NodeId::new("N0"),
        NodeId::new("N1"),
        EdgeId::new("E0"),
        Vec2::new(0.0, 0.0),
        Vec2::new(1000.0, 0.0),
        2,
        30.0,
    );

    let mut sim = TrafficSimulation::new(network);
    let lane_right = LaneId::new("E0_0");

    // Vehicle 0: Slow truck in right lane at pos 80m, speed 8 m/s
    let truck_params = IdmParameters {
        desired_speed: 8.0,
        ..Default::default()
    };
    let truck = Vehicle::new(
        VehicleId(0),
        lane_right.clone(),
        80.0,
        8.0,
        DriverModel::Idm(IdmModel::new(truck_params)),
    );
    sim.add_vehicle(truck).unwrap();

    // Vehicle 1: Fast car in right lane behind truck at pos 20m, speed 25 m/s
    let car_params = IdmParameters {
        desired_speed: 25.0,
        ..Default::default()
    };
    let mut car = Vehicle::new(
        VehicleId(1),
        lane_right.clone(),
        20.0,
        20.0,
        DriverModel::Idm(IdmModel::new(car_params)),
    );
    car.mobil = MobilParameters {
        politeness: 0.1,
        switching_threshold: 0.1,
        safe_decel: 4.0,
        bias_right: 0.05,
        min_clearance: 2.0,
    };
    sim.add_vehicle(car).unwrap();

    let mut did_change_lane = false;
    let dt = 0.1;

    for _ in 0..300 {
        sim.step(dt);
        let collisions = sim.check_collisions();
        assert!(
            collisions.is_empty(),
            "Collision detected: {:?}",
            collisions
        );

        let car_ref = sim.vehicle(&VehicleId(1)).unwrap();
        if car_ref.lane_id == LaneId::new("E0_1") {
            did_change_lane = true;
            break;
        }
    }

    assert!(
        did_change_lane,
        "Fast vehicle should have executed MOBIL lane change to pass slow truck"
    );
}

#[test]
fn test_intersection_signal_stopping() {
    let mut network = Network::new();
    NetworkBuilder::build_four_way_intersection(
        &mut network,
        NodeId::new("J0"),
        Vec2::ZERO,
        100.0,
        13.89,
        JunctionType::TrafficLight,
    );

    let ns_conns = vec![
        ConnectionId::new("conn_S_Straight"),
        ConnectionId::new("conn_N_Straight"),
    ];
    let ew_conns = vec![
        ConnectionId::new("conn_E_Straight"),
        ConnectionId::new("conn_W_Straight"),
    ];

    let tl = sumo_rs::intersection::TrafficLightController::standard_4way(
        "TL_J0".into(),
        &ns_conns,
        &ew_conns,
        15.0, // green
        3.0,  // yellow
        2.0,  // all-red
    );

    let mut sim = TrafficSimulation::new(network);
    sim.add_traffic_light(tl);

    // Vehicle on EW road approaching intersection (starts at red light)
    let ew_lane = LaneId::new("edge_E_in_0");
    let model = DriverModel::idm_default();
    let veh = Vehicle::new(VehicleId(1), ew_lane, 20.0, 10.0, model);
    sim.add_vehicle(veh).unwrap();

    let dt = 0.1;
    // Step during red phase for EW
    for _ in 0..100 {
        sim.step(dt);
        let collisions = sim.check_collisions();
        assert!(collisions.is_empty());
    }

    let veh = sim.vehicle(&VehicleId(1)).unwrap();
    // Vehicle should have decelerated or stopped before the stop line (~100m)
    assert!(
        veh.position < 100.0,
        "Vehicle must not cross red light stop line, position: {}",
        veh.position
    );
}
