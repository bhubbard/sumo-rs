use std::time::Instant;
use glam::Vec2;
use sumo_rs::car_following::{DriverModel, IdmModel, IdmParameters, KraussModel, KraussParameters};
use sumo_rs::lane_changing::{MobilContext, MobilModel, MobilParameters};
use sumo_rs::network::builder::NetworkBuilder;
use sumo_rs::network::edge::Edge;
use sumo_rs::network::graph::Network;
use sumo_rs::network::node::{JunctionType, Node};
use sumo_rs::simulation::{TrafficSimulation, Vehicle};
use sumo_rs::types::{EdgeId, LaneChangeDirection, LaneId, NodeId, VehicleId};

fn main() {
    println!("============================================================");
    println!("     sumo-rs (Rust) vs Eclipse SUMO (C++) Benchmark Suite   ");
    println!("============================================================");

    // Benchmark 1: Krauss & IDM Car-Following Step Latency
    println!("\n--- 1. Microscopic Traffic Step (Krauss + IDM Car-Following) ---");
    for &vehicle_count in &[100, 500, 1000] {
        let mut network = Network::new();
        let num_lanes = 4;
        let highway_length = 60000.0; // 60 km to fit 1000 cars comfortably

        NetworkBuilder::build_straight_road(
            &mut network,
            NodeId::new("start"),
            NodeId::new("end"),
            EdgeId::new("highway"),
            Vec2::new(0.0, 0.0),
            Vec2::new(highway_length, 0.0),
            num_lanes,
            33.33,
        );

        let mut sim = TrafficSimulation::new(network);

        for i in 0..vehicle_count {
            let lane_idx = i % num_lanes;
            let lane_id = LaneId::new(format!("highway_{}", lane_idx));
            let pos = (i / num_lanes) as f32 * 40.0;
            let speed = 25.0 + (i % 10) as f32 * 0.5;

            let model = if i % 2 == 0 {
                DriverModel::Krauss(KraussModel::new(KraussParameters {
                    max_speed: 33.33,
                    accel: 2.6,
                    decel: 4.5,
                    sigma: 0.3,
                    tau: 1.0,
                    min_gap: 2.5,
                }))
            } else {
                DriverModel::Idm(IdmModel::new(IdmParameters {
                    desired_speed: 33.33,
                    max_accel: 2.0,
                    comfortable_decel: 3.0,
                    min_gap: 2.0,
                    time_headway: 1.2,
                    delta: 4.0,
                    max_decel: 9.0,
                }))
            };

            let mut car = Vehicle::new(VehicleId(i as u64), lane_id, pos, speed, model);
            car.mobil = MobilParameters {
                politeness: 0.2,
                switching_threshold: 0.1,
                safe_decel: 4.0,
                bias_right: 0.1,
                min_clearance: 2.5,
            };
            sim.add_vehicle(car).unwrap();
        }

        // Warm up
        for _ in 0..20 {
            sim.step(0.1);
        }

        let steps = 500;
        let start = Instant::now();
        for _ in 0..steps {
            sim.step(0.1);
        }
        let elapsed = start.elapsed();
        let step_latency = elapsed / steps as u32;
        let steps_per_sec = steps as f64 / elapsed.as_secs_f64();
        let vehicle_updates_per_sec = (steps * vehicle_count) as f64 / elapsed.as_secs_f64();

        println!(
            "Vehicles: {:>4} | Step Latency: {:>8.2?} | {:>8.0} steps/s | {:>10.0} veh-updates/s",
            vehicle_count, step_latency, steps_per_sec, vehicle_updates_per_sec
        );
    }

    // Benchmark 2: MOBIL Lane-Changing Calculation Throughput
    println!("\n--- 2. MOBIL Lane-Changing Decisions Throughput ---");
    {
        let mobil = MobilModel::new(MobilParameters {
            politeness: 0.25,
            switching_threshold: 0.15,
            safe_decel: 4.0,
            bias_right: 0.1,
            min_clearance: 3.0,
        });

        let ctx = MobilContext {
            ego_accel_curr: 1.2,
            ego_accel_new: 2.1,
            new_follower_accel_curr: 0.0,
            new_follower_accel_new: -1.2,
            old_follower_accel_curr: -0.5,
            old_follower_accel_new: 0.2,
            leader_accel_delta: 0.0,
            gap_to_new_follower: Some(14.0),
            gap_to_new_leader: Some(18.0),
            direction: LaneChangeDirection::Right,
        };

        let iterations = 2_000_000;
        let start = Instant::now();
        let mut changes = 0;
        for _ in 0..iterations {
            if mobil.evaluate(&ctx).should_change {
                changes += 1;
            }
        }
        let elapsed = start.elapsed();
        let per_sec = iterations as f64 / elapsed.as_secs_f64();
        let ns_per_eval = elapsed.as_nanos() as f64 / iterations as f64;

        println!(
            "MOBIL Evaluations: {} | Time: {:.2?} | {:>10.0} evals/sec ({:.2} ns/eval) | Changed: {}",
            iterations, elapsed, per_sec, ns_per_eval, changes
        );
    }

    // Benchmark 3: Shortest-Path Dijkstra Routing Throughput
    println!("\n--- 3. Network Shortest-Path Routing Throughput ---");
    {
        let mut network = Network::new();
        for i in 0..20 {
            network.add_node(Node::new(
                NodeId::new(format!("n{}", i)),
                Vec2::new((i % 5) as f32 * 100.0, (i / 5) as f32 * 100.0),
                JunctionType::Priority,
            ));
        }
        for i in 0..19 {
            network.add_edge(Edge::new(
                EdgeId::new(format!("e{}", i)),
                NodeId::new(format!("n{}", i)),
                NodeId::new(format!("n{}", i + 1)),
                100.0,
            ));
        }

        let from_edge = EdgeId::new("e0");
        let to_edge = EdgeId::new("e18");

        let routes_to_find = 50_000;
        let start = Instant::now();
        let mut found = 0;
        for _ in 0..routes_to_find {
            if let Some(path) = network.find_route(&from_edge, &to_edge) {
                found += path.len();
            }
        }
        let elapsed = start.elapsed();
        let per_sec = routes_to_find as f64 / elapsed.as_secs_f64();
        let us_per_route = elapsed.as_micros() as f64 / routes_to_find as f64;

        println!(
            "Dijkstra Queries: {} | Time: {:.2?} | {:>10.0} routes/sec ({:.2} µs/route) | Total Segs: {}",
            routes_to_find, elapsed, per_sec, us_per_route, found
        );
    }

    println!("\n============================================================");
    println!("                      Benchmark Complete                    ");
    println!("============================================================");
}
