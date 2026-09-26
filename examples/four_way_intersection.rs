use glam::Vec2;
use sumo_rs::bevy_adapter::vehicle_to_bevy_transform;
use sumo_rs::car_following::{DriverModel, IdmModel, IdmParameters};
use sumo_rs::intersection::traffic_light::TrafficLightController;
use sumo_rs::network::builder::NetworkBuilder;
use sumo_rs::network::graph::Network;
use sumo_rs::network::node::JunctionType;
use sumo_rs::simulation::{TrafficSimulation, Vehicle};
use sumo_rs::types::{ConnectionId, LaneId, NodeId, VehicleId};

fn main() {
    println!("=== sumo-rs: 4-Way Signalized Intersection ===");

    let mut network = Network::new();
    let center = Vec2::new(0.0, 0.0);
    let arm_len = 150.0;

    NetworkBuilder::build_four_way_intersection(
        &mut network,
        NodeId::new("intersection_main"),
        center,
        arm_len,
        13.89, // 50 km/h
        JunctionType::TrafficLight,
    );

    let ns_conns = vec![
        ConnectionId::new("conn_S_Straight"),
        ConnectionId::new("conn_S_TurnRight"),
        ConnectionId::new("conn_S_TurnLeft"),
        ConnectionId::new("conn_N_Straight"),
        ConnectionId::new("conn_N_TurnRight"),
        ConnectionId::new("conn_N_TurnLeft"),
    ];

    let ew_conns = vec![
        ConnectionId::new("conn_E_Straight"),
        ConnectionId::new("conn_E_TurnRight"),
        ConnectionId::new("conn_E_TurnLeft"),
        ConnectionId::new("conn_W_Straight"),
        ConnectionId::new("conn_W_TurnRight"),
        ConnectionId::new("conn_W_TurnLeft"),
    ];

    let tl = TrafficLightController::standard_4way(
        "TL_Main".into(),
        &ns_conns,
        &ew_conns,
        20.0, // green
        3.0,  // yellow
        2.0,  // all-red
    );

    let mut sim = TrafficSimulation::new(network);
    sim.add_traffic_light(tl);

    // Spawn vehicles on each approach
    let approaches = [
        ("edge_S_in_0", 10.0),
        ("edge_S_in_0", 40.0),
        ("edge_N_in_0", 20.0),
        ("edge_E_in_0", 15.0),
        ("edge_W_in_0", 30.0),
    ];

    for (i, (lane_str, pos)) in approaches.iter().enumerate() {
        let idm = IdmParameters {
            desired_speed: 13.89,
            ..Default::default()
        };
        let veh = Vehicle::new(
            VehicleId(i as u64),
            LaneId::new(*lane_str),
            *pos,
            10.0,
            DriverModel::Idm(IdmModel::new(idm)),
        );
        sim.add_vehicle(veh).unwrap();
    }

    println!("Total vehicles spawned: {}", sim.vehicle_count());

    // Run 40 seconds (400 steps)
    let dt = 0.1;
    for step in 0..400 {
        sim.step(dt);

        if step % 100 == 0 {
            println!(
                "Time: {:>4.1}s | Phase: {:<12} | Active: {}",
                sim.sim_time,
                sim.traffic_lights[&sumo_rs::types::TrafficLightId::new("TL_Main")]
                    .current_phase()
                    .unwrap()
                    .name,
                sim.active_vehicles().count()
            );

            // Demonstrate Bevy transform conversion for first vehicle
            if let Ok(v0) = sim.vehicle(&VehicleId(0)) {
                let transform = vehicle_to_bevy_transform(v0, &sim.network, 0.0);
                println!(
                    "  [Bevy] Veh 0 Pos (3D): ({:.1}, {:.1}, {:.1}) | Rot: {:?}",
                    transform.translation.x,
                    transform.translation.y,
                    transform.translation.z,
                    transform.rotation
                );
            }
        }
    }

    let collisions = sim.check_collisions();
    println!("Collisions detected: {}", collisions.len());
    assert!(collisions.is_empty());

    println!("4-way intersection simulation completed without incidents!");
}
