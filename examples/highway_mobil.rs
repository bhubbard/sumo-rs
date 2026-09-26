use glam::Vec2;
use sumo_rs::car_following::{DriverModel, IdmModel, IdmParameters, KraussModel, KraussParameters};
use sumo_rs::lane_changing::MobilParameters;
use sumo_rs::network::builder::NetworkBuilder;
use sumo_rs::network::graph::Network;
use sumo_rs::simulation::{TrafficSimulation, Vehicle};
use sumo_rs::types::{EdgeId, LaneId, NodeId, VehicleId};

fn main() {
    println!("=== sumo-rs: Highway MOBIL Simulation ===");

    let mut network = Network::new();
    let num_lanes = 3;
    let highway_length = 2000.0; // 2 km

    NetworkBuilder::build_straight_road(
        &mut network,
        NodeId::new("highway_start"),
        NodeId::new("highway_end"),
        EdgeId::new("highway"),
        Vec2::new(0.0, 0.0),
        Vec2::new(highway_length, 0.0),
        num_lanes,
        33.33, // 120 km/h
    );

    let mut sim = TrafficSimulation::new(network);

    // Spawn slow trucks in right lane (index 0)
    for i in 0..3 {
        let pos = 200.0 + (i as f32) * 250.0;
        let truck_params = IdmParameters {
            desired_speed: 18.0, // ~65 km/h
            ..Default::default()
        };
        let truck = Vehicle::new(
            VehicleId(100 + i),
            LaneId::new("highway_0"),
            pos,
            18.0,
            DriverModel::Idm(IdmModel::new(truck_params)),
        );
        sim.add_vehicle(truck).unwrap();
    }

    // Spawn fast commuter cars in right/middle lanes with MOBIL enabled
    for i in 0..6 {
        let pos = (i as f32) * 50.0;
        let car_params = KraussParameters {
            max_speed: 30.0, // ~108 km/h
            sigma: 0.2,
            ..Default::default()
        };

        let mut car = Vehicle::new(
            VehicleId(i),
            LaneId::new("highway_0"),
            pos,
            22.0,
            DriverModel::Krauss(KraussModel::new(car_params)),
        );
        car.mobil = MobilParameters {
            politeness: 0.25,
            switching_threshold: 0.15,
            safe_decel: 4.0,
            bias_right: 0.1, // Keep-right rule
            min_clearance: 3.0,
        };
        sim.add_vehicle(car).unwrap();
    }

    println!("Initial vehicle count: {}", sim.vehicle_count());

    // Step simulation for 60 seconds (600 steps at dt = 0.1)
    let dt = 0.1;
    for step in 0..600 {
        sim.step(dt);

        if step % 100 == 0 {
            println!(
                "Time: {:>5.1}s | Active: {:>2} | Avg Speed: {:>5.1} m/s ({:>5.1} km/h)",
                sim.sim_time,
                sim.active_vehicles().count(),
                sim.average_speed(),
                sim.average_speed() * 3.6
            );
        }
    }

    let collisions = sim.check_collisions();
    println!("Collisions detected: {}", collisions.len());
    assert!(collisions.is_empty());

    println!("Simulation finished successfully!");
}
