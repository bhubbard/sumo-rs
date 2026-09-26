use crate::network::connection::Connection;
use crate::network::edge::Edge;
use crate::network::graph::Network;
use crate::network::lane::Lane;
use crate::network::node::{JunctionType, Node};
use crate::types::{ConnectionId, EdgeId, LaneId, NodeId, TurnDirection};
use glam::Vec2;

pub struct NetworkBuilder;

impl NetworkBuilder {
    /// Creates a straight multi-lane road segment between two points.
    /// Lanes are indexed 0 (rightmost) to num_lanes - 1 (leftmost).
    #[allow(clippy::too_many_arguments)]
    pub fn build_straight_road(
        network: &mut Network,
        from_node_id: NodeId,
        to_node_id: NodeId,
        edge_id: EdgeId,
        from_pos: Vec2,
        to_pos: Vec2,
        num_lanes: usize,
        speed_limit: f32,
    ) {
        if !network.nodes.contains_key(&from_node_id) {
            network.add_node(Node::new(
                from_node_id.clone(),
                from_pos,
                JunctionType::Uncontrolled,
            ));
        }
        if !network.nodes.contains_key(&to_node_id) {
            network.add_node(Node::new(
                to_node_id.clone(),
                to_pos,
                JunctionType::Uncontrolled,
            ));
        }

        let road_vec = to_pos - from_pos;
        let length = road_vec.length();
        let forward = road_vec.normalize();
        // Normal vector pointing to the left of the driving direction
        let left_normal = Vec2::new(-forward.y, forward.x);
        let lane_width = 3.5;

        let mut edge = Edge::new(edge_id.clone(), from_node_id, to_node_id, length);
        let mut lane_ids = Vec::new();

        for i in 0..num_lanes {
            let lane_id = LaneId::from_edge(&edge_id, i);
            lane_ids.push(lane_id.clone());

            // Lateral offset: lane 0 is rightmost
            let lateral_offset = (i as f32 + 0.5) * lane_width;
            let start = from_pos + left_normal * lateral_offset;
            let end = to_pos + left_normal * lateral_offset;

            let mut lane = Lane::new(lane_id, edge_id.clone(), i, vec![start, end], speed_limit);
            lane.width = lane_width;
            network.add_lane(lane);
        }

        // Configure left/right neighbor pointers
        for i in 0..num_lanes {
            let id = &lane_ids[i];
            let right = if i > 0 {
                Some(lane_ids[i - 1].clone())
            } else {
                None
            };
            let left = if i + 1 < num_lanes {
                Some(lane_ids[i + 1].clone())
            } else {
                None
            };

            if let Some(lane) = network.lanes.get_mut(id) {
                lane.right_lane = right;
                lane.left_lane = left;
            }
        }

        edge.lanes = lane_ids;
        network.add_edge(edge);
    }

    /// Creates a 4-way cross intersection at `center` with incoming and outgoing legs in 4 cardinal directions.
    ///
    /// Cardinal directions:
    /// - South: coming from (center.x, center.y - arm_len) towards center
    /// - North: coming from (center.x, center.y + arm_len) towards center
    /// - West: coming from (center.x - arm_len, center.y) towards center
    /// - East: coming from (center.x + arm_len, center.y) towards center
    pub fn build_four_way_intersection(
        network: &mut Network,
        center_node_id: NodeId,
        center: Vec2,
        arm_len: f32,
        speed_limit: f32,
        junction_type: JunctionType,
    ) {
        let center_node = Node::new(center_node_id.clone(), center, junction_type);
        network.add_node(center_node);

        // Exterior boundary nodes
        let n_node = NodeId::new(format!("{}_N", center_node_id));
        let s_node = NodeId::new(format!("{}_S", center_node_id));
        let e_node = NodeId::new(format!("{}_E", center_node_id));
        let w_node = NodeId::new(format!("{}_W", center_node_id));

        network.add_node(Node::new(
            n_node.clone(),
            center + Vec2::new(0.0, arm_len),
            JunctionType::Uncontrolled,
        ));
        network.add_node(Node::new(
            s_node.clone(),
            center + Vec2::new(0.0, -arm_len),
            JunctionType::Uncontrolled,
        ));
        network.add_node(Node::new(
            e_node.clone(),
            center + Vec2::new(arm_len, 0.0),
            JunctionType::Uncontrolled,
        ));
        network.add_node(Node::new(
            w_node.clone(),
            center + Vec2::new(-arm_len, 0.0),
            JunctionType::Uncontrolled,
        ));

        let offset = 2.0; // distance from centerline of intersection

        // 1. South to Center (Inflow) & Center to South (Outflow)
        Self::create_directed_leg(
            network,
            &s_node,
            &center_node_id,
            &EdgeId::new("edge_S_in"),
            center + Vec2::new(offset, -arm_len),
            center + Vec2::new(offset, -offset),
            speed_limit,
        );
        Self::create_directed_leg(
            network,
            &center_node_id,
            &s_node,
            &EdgeId::new("edge_S_out"),
            center + Vec2::new(-offset, -offset),
            center + Vec2::new(-offset, -arm_len),
            speed_limit,
        );

        // 2. North to Center (Inflow) & Center to North (Outflow)
        Self::create_directed_leg(
            network,
            &n_node,
            &center_node_id,
            &EdgeId::new("edge_N_in"),
            center + Vec2::new(-offset, arm_len),
            center + Vec2::new(-offset, offset),
            speed_limit,
        );
        Self::create_directed_leg(
            network,
            &center_node_id,
            &n_node,
            &EdgeId::new("edge_N_out"),
            center + Vec2::new(offset, offset),
            center + Vec2::new(offset, arm_len),
            speed_limit,
        );

        // 3. West to Center (Inflow) & Center to West (Outflow)
        Self::create_directed_leg(
            network,
            &w_node,
            &center_node_id,
            &EdgeId::new("edge_W_in"),
            center + Vec2::new(-arm_len, -offset),
            center + Vec2::new(-offset, -offset),
            speed_limit,
        );
        Self::create_directed_leg(
            network,
            &center_node_id,
            &w_node,
            &EdgeId::new("edge_W_out"),
            center + Vec2::new(-offset, offset),
            center + Vec2::new(-arm_len, offset),
            speed_limit,
        );

        // 4. East to Center (Inflow) & Center to East (Outflow)
        Self::create_directed_leg(
            network,
            &e_node,
            &center_node_id,
            &EdgeId::new("edge_E_in"),
            center + Vec2::new(arm_len, offset),
            center + Vec2::new(offset, offset),
            speed_limit,
        );
        Self::create_directed_leg(
            network,
            &center_node_id,
            &e_node,
            &EdgeId::new("edge_E_out"),
            center + Vec2::new(offset, -offset),
            center + Vec2::new(arm_len, -offset),
            speed_limit,
        );

        // Now link connections across intersection (Straight, Left turn, Right turn)
        let directions = [
            ("S", "edge_S_in_0", "edge_N_out_0", TurnDirection::Straight),
            ("S", "edge_S_in_0", "edge_E_out_0", TurnDirection::TurnRight),
            ("S", "edge_S_in_0", "edge_W_out_0", TurnDirection::TurnLeft),
            ("N", "edge_N_in_0", "edge_S_out_0", TurnDirection::Straight),
            ("N", "edge_N_in_0", "edge_W_out_0", TurnDirection::TurnRight),
            ("N", "edge_N_in_0", "edge_E_out_0", TurnDirection::TurnLeft),
            ("W", "edge_W_in_0", "edge_E_out_0", TurnDirection::Straight),
            ("W", "edge_W_in_0", "edge_S_out_0", TurnDirection::TurnRight),
            ("W", "edge_W_in_0", "edge_N_out_0", TurnDirection::TurnLeft),
            ("E", "edge_E_in_0", "edge_W_out_0", TurnDirection::Straight),
            ("E", "edge_E_in_0", "edge_N_out_0", TurnDirection::TurnRight),
            ("E", "edge_E_in_0", "edge_S_out_0", TurnDirection::TurnLeft),
        ];

        for (from_dir, from_lane_str, to_lane_str, turn) in directions {
            let conn_id = ConnectionId::new(format!("conn_{}_{:?}", from_dir, turn));
            let from_l = LaneId::new(from_lane_str);
            let to_l = LaneId::new(to_lane_str);

            let p_start = network
                .lanes
                .get(&from_l)
                .map(|l| *l.shape.last().unwrap())
                .unwrap_or(center);
            let p_end = network
                .lanes
                .get(&to_l)
                .map(|l| l.shape[0])
                .unwrap_or(center);

            // Shape through intersection
            let shape = match turn {
                TurnDirection::Straight => vec![p_start, p_end],
                TurnDirection::TurnLeft => {
                    let control = center;
                    vec![p_start, control, p_end]
                }
                TurnDirection::TurnRight => {
                    let corner = (p_start + p_end) * 0.5;
                    vec![p_start, corner, p_end]
                }
                TurnDirection::UTurn => vec![p_start, center, p_end],
            };

            let mut conn = Connection::new(conn_id.clone(), from_l, to_l, turn, shape);
            if turn == TurnDirection::Straight {
                conn.priority = true;
            }
            network.add_connection(conn);
        }

        Self::compute_conflicts_and_yields(network);
    }

    fn create_directed_leg(
        network: &mut Network,
        from_n: &NodeId,
        to_n: &NodeId,
        edge_id: &EdgeId,
        start_pt: Vec2,
        end_pt: Vec2,
        speed: f32,
    ) {
        let length = (end_pt - start_pt).length();
        let mut edge = Edge::new(edge_id.clone(), from_n.clone(), to_n.clone(), length);
        let lane_id = LaneId::from_edge(edge_id, 0);
        edge.lanes.push(lane_id.clone());

        let lane = Lane::new(lane_id, edge_id.clone(), 0, vec![start_pt, end_pt], speed);
        network.add_lane(lane);
        network.add_edge(edge);
    }

    /// Automatically compute intersecting conflict paths and yield rules between connections.
    pub fn compute_conflicts_and_yields(network: &mut Network) {
        let conn_ids: Vec<ConnectionId> = network.connections.keys().cloned().collect();

        for i in 0..conn_ids.len() {
            for j in (i + 1)..conn_ids.len() {
                let id_a = &conn_ids[i];
                let id_b = &conn_ids[j];

                let shape_a = network.connections[id_a].shape.clone();
                let shape_b = network.connections[id_b].shape.clone();

                if polylines_intersect(&shape_a, &shape_b) {
                    network
                        .connections
                        .get_mut(id_a)
                        .unwrap()
                        .conflict_connections
                        .push(id_b.clone());
                    network
                        .connections
                        .get_mut(id_b)
                        .unwrap()
                        .conflict_connections
                        .push(id_a.clone());

                    // Left turn yield rule:
                    // If A is Left turn and B is Straight or Right turn, A yields to B.
                    let dir_a = network.connections[id_a].direction;
                    let dir_b = network.connections[id_b].direction;

                    if dir_a == TurnDirection::TurnLeft
                        && (dir_b == TurnDirection::Straight || dir_b == TurnDirection::TurnRight)
                    {
                        network
                            .connections
                            .get_mut(id_a)
                            .unwrap()
                            .yield_to
                            .push(id_b.clone());
                    } else if dir_b == TurnDirection::TurnLeft
                        && (dir_a == TurnDirection::Straight || dir_a == TurnDirection::TurnRight)
                    {
                        network
                            .connections
                            .get_mut(id_b)
                            .unwrap()
                            .yield_to
                            .push(id_a.clone());
                    }
                }
            }
        }
    }
}

fn polylines_intersect(p1: &[Vec2], p2: &[Vec2]) -> bool {
    for w1 in p1.windows(2) {
        for w2 in p2.windows(2) {
            if line_segments_intersect(w1[0], w1[1], w2[0], w2[1]) {
                return true;
            }
        }
    }
    false
}

fn line_segments_intersect(a: Vec2, b: Vec2, c: Vec2, d: Vec2) -> bool {
    fn ccw(p1: Vec2, p2: Vec2, p3: Vec2) -> bool {
        (p3.y - p1.y) * (p2.x - p1.x) > (p2.y - p1.y) * (p3.x - p1.x)
    }

    let ccw1 = ccw(a, c, d);
    let ccw2 = ccw(b, c, d);
    let ccw3 = ccw(a, b, c);
    let ccw4 = ccw(a, b, d);

    ccw1 != ccw2 && ccw3 != ccw4
}
