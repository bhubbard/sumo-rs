use crate::types::{ConnectionId, EdgeId, LaneId, NodeId, VehicleId};
use thiserror::Error;

#[derive(Error, Debug)]
pub enum SumoError {
    #[error("Node not found: {0}")]
    NodeNotFound(NodeId),

    #[error("Edge not found: {0}")]
    EdgeNotFound(EdgeId),

    #[error("Lane not found: {0}")]
    LaneNotFound(LaneId),

    #[error("Connection not found: {0}")]
    ConnectionNotFound(ConnectionId),

    #[error("Vehicle not found: {0}")]
    VehicleNotFound(VehicleId),

    #[error("Vehicle {0} already exists in simulation")]
    VehicleAlreadyExists(VehicleId),

    #[error("Invalid route: {0}")]
    InvalidRoute(String),

    #[error("Network topology error: {0}")]
    TopologyError(String),

    #[error("Simulation error: {0}")]
    SimulationError(String),
}

pub type Result<T> = std::result::Result<T, SumoError>;
