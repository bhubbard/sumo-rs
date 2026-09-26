use crate::network::lane::calculate_polyline_length;
use crate::types::{ConnectionId, LaneId, TurnDirection};
use glam::Vec2;
use serde::{Deserialize, Serialize};

/// Connection linking an incoming lane to an outgoing lane across an intersection.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Connection {
    pub id: ConnectionId,
    pub from_lane: LaneId,
    pub to_lane: LaneId,
    pub direction: TurnDirection,
    pub shape: Vec<Vec2>,
    pub length: f32,
    /// Whether this connection has unconditional right-of-way over intersecting non-priority connections.
    pub priority: bool,
    /// Explicit list of conflicting connection IDs to which vehicles on this connection must yield.
    pub yield_to: Vec<ConnectionId>,
    /// Conflicting connection IDs whose physical trajectory crosses or overlaps with this connection.
    pub conflict_connections: Vec<ConnectionId>,
    /// Optional internal lane representation for microscopic tracking through the intersection.
    pub internal_lane: Option<LaneId>,
}

impl Connection {
    pub fn new(
        id: ConnectionId,
        from_lane: LaneId,
        to_lane: LaneId,
        direction: TurnDirection,
        shape: Vec<Vec2>,
    ) -> Self {
        let length = calculate_polyline_length(&shape);
        Self {
            id,
            from_lane,
            to_lane,
            direction,
            shape,
            length,
            priority: false,
            yield_to: Vec::new(),
            conflict_connections: Vec::new(),
            internal_lane: None,
        }
    }

    pub fn with_priority(mut self, priority: bool) -> Self {
        self.priority = priority;
        self
    }

    pub fn with_yield_to(mut self, yield_to: Vec<ConnectionId>) -> Self {
        self.yield_to = yield_to;
        self
    }

    pub fn with_conflicts(mut self, conflicts: Vec<ConnectionId>) -> Self {
        self.conflict_connections = conflicts;
        self
    }

    pub fn with_internal_lane(mut self, internal_lane: LaneId) -> Self {
        self.internal_lane = Some(internal_lane);
        self
    }
}
