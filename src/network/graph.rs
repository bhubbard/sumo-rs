use crate::error::{Result, SumoError};
use crate::network::connection::Connection;
use crate::network::edge::Edge;
use crate::network::lane::Lane;
use crate::network::node::Node;
use crate::types::{ConnectionId, EdgeId, LaneId, NodeId};
use serde::{Deserialize, Serialize};
use std::collections::{BinaryHeap, HashMap};

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Network {
    pub nodes: HashMap<NodeId, Node>,
    pub edges: HashMap<EdgeId, Edge>,
    pub lanes: HashMap<LaneId, Lane>,
    pub connections: HashMap<ConnectionId, Connection>,
}

impl Network {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn add_node(&mut self, node: Node) {
        self.nodes.insert(node.id.clone(), node);
    }

    pub fn add_edge(&mut self, edge: Edge) {
        if let Some(from_node) = self.nodes.get_mut(&edge.from_node)
            && !from_node.outgoing_edges.contains(&edge.id)
        {
            from_node.outgoing_edges.push(edge.id.clone());
        }
        if let Some(to_node) = self.nodes.get_mut(&edge.to_node)
            && !to_node.incoming_edges.contains(&edge.id)
        {
            to_node.incoming_edges.push(edge.id.clone());
        }
        self.edges.insert(edge.id.clone(), edge);
    }

    pub fn add_lane(&mut self, lane: Lane) {
        if let Some(edge) = self.edges.get_mut(&lane.edge_id)
            && !edge.lanes.contains(&lane.id)
        {
            edge.lanes.push(lane.id.clone());
        }
        self.lanes.insert(lane.id.clone(), lane);
    }

    pub fn add_connection(&mut self, connection: Connection) {
        if let Some(from_lane) = self.lanes.get_mut(&connection.from_lane)
            && !from_lane.outgoing_connections.contains(&connection.id)
        {
            from_lane.outgoing_connections.push(connection.id.clone());
        }
        self.connections.insert(connection.id.clone(), connection);
    }

    pub fn get_node(&self, id: &NodeId) -> Result<&Node> {
        self.nodes
            .get(id)
            .ok_or_else(|| SumoError::NodeNotFound(id.clone()))
    }

    pub fn get_edge(&self, id: &EdgeId) -> Result<&Edge> {
        self.edges
            .get(id)
            .ok_or_else(|| SumoError::EdgeNotFound(id.clone()))
    }

    pub fn get_lane(&self, id: &LaneId) -> Result<&Lane> {
        self.lanes
            .get(id)
            .ok_or_else(|| SumoError::LaneNotFound(id.clone()))
    }

    pub fn get_connection(&self, id: &ConnectionId) -> Result<&Connection> {
        self.connections
            .get(id)
            .ok_or_else(|| SumoError::ConnectionNotFound(id.clone()))
    }

    /// Computes shortest path route between two edges using Dijkstra's algorithm.
    pub fn find_route(&self, from_edge: &EdgeId, to_edge: &EdgeId) -> Option<Vec<EdgeId>> {
        if !self.edges.contains_key(from_edge) || !self.edges.contains_key(to_edge) {
            return None;
        }

        if from_edge == to_edge {
            return Some(vec![from_edge.clone()]);
        }

        #[derive(Copy, Clone, PartialEq)]
        struct State {
            cost: f32,
            edge_idx: usize,
        }

        impl Eq for State {}

        impl Ord for State {
            fn cmp(&self, other: &Self) -> std::cmp::Ordering {
                other
                    .cost
                    .partial_cmp(&self.cost)
                    .unwrap_or(std::cmp::Ordering::Equal)
            }
        }

        impl PartialOrd for State {
            fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
                Some(self.cmp(other))
            }
        }

        // Map edge IDs to compact indices
        let edge_list: Vec<&EdgeId> = self.edges.keys().collect();
        let edge_to_idx: HashMap<&EdgeId, usize> = edge_list
            .iter()
            .enumerate()
            .map(|(i, &id)| (id, i))
            .collect();

        let start_idx = *edge_to_idx.get(from_edge)?;
        let target_idx = *edge_to_idx.get(to_edge)?;

        let mut dist: Vec<f32> = vec![f32::INFINITY; edge_list.len()];
        let mut prev: Vec<Option<usize>> = vec![None; edge_list.len()];
        let mut heap = BinaryHeap::new();

        dist[start_idx] = self.edges.get(from_edge)?.length;
        heap.push(State {
            cost: dist[start_idx],
            edge_idx: start_idx,
        });

        while let Some(State { cost, edge_idx }) = heap.pop() {
            if edge_idx == target_idx {
                let mut path = Vec::new();
                let mut curr = Some(target_idx);
                while let Some(idx) = curr {
                    path.push(edge_list[idx].clone());
                    curr = prev[idx];
                }
                path.reverse();
                return Some(path);
            }

            if cost > dist[edge_idx] {
                continue;
            }

            let curr_edge_id = edge_list[edge_idx];
            let curr_edge = match self.edges.get(curr_edge_id) {
                Some(e) => e,
                None => continue,
            };

            // Explore outgoing edges connected at to_node
            if let Some(target_node) = self.nodes.get(&curr_edge.to_node) {
                for next_edge_id in &target_node.outgoing_edges {
                    if let Some(&next_idx) = edge_to_idx.get(next_edge_id) {
                        let next_edge = &self.edges[next_edge_id];
                        let next_cost = cost + next_edge.length;
                        if next_cost < dist[next_idx] {
                            dist[next_idx] = next_cost;
                            prev[next_idx] = Some(edge_idx);
                            heap.push(State {
                                cost: next_cost,
                                edge_idx: next_idx,
                            });
                        }
                    }
                }
            }
        }

        None
    }
}
