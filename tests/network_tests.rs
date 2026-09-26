use approx::assert_relative_eq;
use glam::Vec2;
use sumo_rs::network::builder::NetworkBuilder;
use sumo_rs::network::graph::Network;
use sumo_rs::network::lane::Lane;
use sumo_rs::types::{EdgeId, LaneId, NodeId};

#[test]
fn test_lane_geometry_and_interpolation() {
    let lane_id = LaneId::new("lane_0");
    let edge_id = EdgeId::new("edge_0");
    let shape = vec![Vec2::new(0.0, 0.0), Vec2::new(100.0, 0.0)];
    let lane = Lane::new(lane_id, edge_id, 0, shape, 13.89);

    assert_relative_eq!(lane.length, 100.0, epsilon = 1e-4);

    // Position at offset 25m
    let pos_25 = lane.position_at_offset(25.0);
    assert_relative_eq!(pos_25.x, 25.0, epsilon = 1e-4);
    assert_relative_eq!(pos_25.y, 0.0, epsilon = 1e-4);

    // Heading at offset 50m
    let heading = lane.heading_at_offset(50.0);
    assert_relative_eq!(heading, 0.0, epsilon = 1e-4);

    // Point projection
    let (s, dist) = lane.project_point(Vec2::new(30.0, 5.0));
    assert_relative_eq!(s, 30.0, epsilon = 1e-4);
    assert_relative_eq!(dist, 5.0, epsilon = 1e-4);
}

#[test]
fn test_multi_lane_road_builder() {
    let mut network = Network::new();
    NetworkBuilder::build_straight_road(
        &mut network,
        NodeId::new("N0"),
        NodeId::new("N1"),
        EdgeId::new("E0"),
        Vec2::new(0.0, 0.0),
        Vec2::new(200.0, 0.0),
        3, // 3 lanes
        25.0,
    );

    let edge = network.get_edge(&EdgeId::new("E0")).unwrap();
    assert_eq!(edge.lane_count(), 3);

    let lane0 = network.get_lane(&edge.lanes[0]).unwrap();
    let lane1 = network.get_lane(&edge.lanes[1]).unwrap();
    let lane2 = network.get_lane(&edge.lanes[2]).unwrap();

    // Check neighbor connectivity
    assert_eq!(lane0.right_lane, None);
    assert_eq!(lane0.left_lane, Some(lane1.id.clone()));

    assert_eq!(lane1.right_lane, Some(lane0.id.clone()));
    assert_eq!(lane1.left_lane, Some(lane2.id.clone()));

    assert_eq!(lane2.right_lane, Some(lane1.id.clone()));
    assert_eq!(lane2.left_lane, None);
}

#[test]
fn test_network_dijkstra_routing() {
    let mut network = Network::new();
    // Segment 1: N0 -> N1
    NetworkBuilder::build_straight_road(
        &mut network,
        NodeId::new("N0"),
        NodeId::new("N1"),
        EdgeId::new("E0"),
        Vec2::new(0.0, 0.0),
        Vec2::new(100.0, 0.0),
        1,
        15.0,
    );
    // Segment 2: N1 -> N2
    NetworkBuilder::build_straight_road(
        &mut network,
        NodeId::new("N1"),
        NodeId::new("N2"),
        EdgeId::new("E1"),
        Vec2::new(100.0, 0.0),
        Vec2::new(200.0, 0.0),
        1,
        15.0,
    );
    // Segment 3: N2 -> N3
    NetworkBuilder::build_straight_road(
        &mut network,
        NodeId::new("N2"),
        NodeId::new("N3"),
        EdgeId::new("E2"),
        Vec2::new(200.0, 0.0),
        Vec2::new(300.0, 0.0),
        1,
        15.0,
    );

    let route = network.find_route(&EdgeId::new("E0"), &EdgeId::new("E2"));
    assert!(route.is_some());
    let path = route.unwrap();
    assert_eq!(
        path,
        vec![EdgeId::new("E0"), EdgeId::new("E1"), EdgeId::new("E2")]
    );
}
