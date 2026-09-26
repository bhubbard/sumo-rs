use crate::types::{EdgeId, LaneId, NodeId};
use serde::{Deserialize, Serialize};

/// Represents a directed road segment connecting two nodes (junctions).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Edge {
    pub id: EdgeId,
    pub from_node: NodeId,
    pub to_node: NodeId,
    pub length: f32,
    /// Ordered lanes within this edge (index 0 is the rightmost lane).
    pub lanes: Vec<LaneId>,
    /// Priority level (higher priority roads have right-of-way over lower priority roads).
    pub priority: i32,
    /// Human-readable street name.
    pub road_name: String,
}

impl Edge {
    pub fn new(id: EdgeId, from_node: NodeId, to_node: NodeId, length: f32) -> Self {
        Self {
            id,
            from_node,
            to_node,
            length,
            lanes: Vec::new(),
            priority: 0,
            road_name: String::new(),
        }
    }

    pub fn with_lanes(mut self, lanes: Vec<LaneId>) -> Self {
        self.lanes = lanes;
        self
    }

    pub fn with_priority(mut self, priority: i32) -> Self {
        self.priority = priority;
        self
    }

    pub fn with_road_name(mut self, name: impl Into<String>) -> Self {
        self.road_name = name.into();
        self
    }

    pub fn lane_count(&self) -> usize {
        self.lanes.len()
    }
}
