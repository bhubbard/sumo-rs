use crate::types::{ConnectionId, EdgeId, NodeId, TrafficLightId};
use glam::Vec2;
use serde::{Deserialize, Serialize};

/// Type of intersection / junction.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum JunctionType {
    /// Major road priority; incoming minor roads yield.
    Priority,
    /// Uncontrolled intersection where drivers yield to traffic coming from the right.
    RightBeforeLeft,
    /// All-way stop intersection (e.g. 4-way stop).
    AllWayStop,
    /// Controlled by traffic lights.
    TrafficLight,
    /// Uncontrolled node (e.g., road shape waypoint, simple merge/split).
    Uncontrolled,
}

/// Represents an intersection or junction in the road network graph.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Node {
    pub id: NodeId,
    pub position: Vec2,
    pub junction_type: JunctionType,
    pub incoming_edges: Vec<EdgeId>,
    pub outgoing_edges: Vec<EdgeId>,
    pub internal_connections: Vec<ConnectionId>,
    pub traffic_light_id: Option<TrafficLightId>,
}

impl Node {
    pub fn new(id: NodeId, position: Vec2, junction_type: JunctionType) -> Self {
        Self {
            id,
            position,
            junction_type,
            incoming_edges: Vec::new(),
            outgoing_edges: Vec::new(),
            internal_connections: Vec::new(),
            traffic_light_id: None,
        }
    }

    pub fn with_traffic_light(mut self, tl_id: TrafficLightId) -> Self {
        self.junction_type = JunctionType::TrafficLight;
        self.traffic_light_id = Some(tl_id);
        self
    }
}
