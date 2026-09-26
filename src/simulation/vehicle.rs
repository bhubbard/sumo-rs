use crate::car_following::DriverModel;
use crate::lane_changing::MobilParameters;
use crate::network::graph::Network;
use crate::types::{ConnectionId, EdgeId, LaneId, VehicleId};
use glam::Vec2;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum VehicleState {
    Active,
    StoppedAtSign,
    WaitingForSignal,
    Yielding,
    Arrived,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Vehicle {
    pub id: VehicleId,
    pub lane_id: LaneId,
    /// Longitudinal position along the lane (meters from start).
    pub position: f32,
    /// Current velocity (m/s).
    pub speed: f32,
    /// Current acceleration (m/s²).
    pub acceleration: f32,
    /// Vehicle length (meters, bumper-to-bumper).
    pub length: f32,
    /// Vehicle width (meters).
    pub width: f32,
    /// Maximum allowed speed for this vehicle (m/s).
    pub max_speed: f32,

    /// Planned route as a sequence of EdgeIds.
    pub route: Vec<EdgeId>,
    /// Index into the route sequence.
    pub route_index: usize,

    /// Car-following model (Krauss or IDM).
    pub driver_model: DriverModel,
    /// MOBIL lane changing parameters.
    pub mobil: MobilParameters,

    /// Active intersection connection if traversing through an intersection.
    pub current_connection: Option<ConnectionId>,
    /// State of the vehicle.
    pub state: VehicleState,

    /// Total distance traveled (meters).
    pub total_distance_traveled: f32,
    /// Total time spent waiting at standstill (seconds).
    pub total_waiting_time: f32,
}

impl Vehicle {
    pub fn new(
        id: VehicleId,
        lane_id: LaneId,
        position: f32,
        initial_speed: f32,
        driver_model: DriverModel,
    ) -> Self {
        let max_speed = driver_model.desired_speed();
        Self {
            id,
            lane_id,
            position,
            speed: initial_speed,
            acceleration: 0.0,
            length: 5.0,
            width: 2.0,
            max_speed,
            route: Vec::new(),
            route_index: 0,
            driver_model,
            mobil: MobilParameters::default(),
            current_connection: None,
            state: VehicleState::Active,
            total_distance_traveled: 0.0,
            total_waiting_time: 0.0,
        }
    }

    pub fn with_route(mut self, route: Vec<EdgeId>) -> Self {
        self.route = route;
        self
    }

    pub fn with_length(mut self, length: f32) -> Self {
        self.length = length;
        self
    }

    pub fn with_width(mut self, width: f32) -> Self {
        self.width = width;
        self
    }

    pub fn with_mobil(mut self, mobil: MobilParameters) -> Self {
        self.mobil = mobil;
        self
    }

    /// Front bumper longitudinal position along lane.
    pub fn front_bumper_pos(&self) -> f32 {
        self.position
    }

    /// Rear bumper longitudinal position along lane.
    pub fn rear_bumper_pos(&self) -> f32 {
        self.position - self.length
    }

    /// Computes world 2D position by interpolating along the current lane geometry.
    pub fn world_position(&self, network: &Network) -> Vec2 {
        if let Some(conn_id) = &self.current_connection
            && let Ok(conn) = network.get_connection(conn_id)
        {
            // If connection has internal lane or shape
            let len = conn.length.max(0.1);
            let factor = (self.position / len).clamp(0.0, 1.0);
            if conn.shape.len() >= 2 {
                if conn.shape.len() == 2 {
                    return conn.shape[0].lerp(conn.shape[1], factor);
                }
                // Quadratic or polyline curve
                let p0 = conn.shape[0];
                let p1 = conn.shape[1];
                let p2 = *conn.shape.last().unwrap();
                let one_minus_t = 1.0 - factor;
                return p0 * (one_minus_t * one_minus_t)
                    + p1 * (2.0 * one_minus_t * factor)
                    + p2 * (factor * factor);
            }
        }

        if let Ok(lane) = network.get_lane(&self.lane_id) {
            lane.position_at_offset(self.position)
        } else {
            Vec2::ZERO
        }
    }

    /// Computes heading angle (radians) along the current lane or intersection path.
    pub fn world_heading(&self, network: &Network) -> f32 {
        if let Some(conn_id) = &self.current_connection
            && let Ok(conn) = network.get_connection(conn_id)
            && conn.shape.len() >= 2
        {
            let dir = (conn.shape.last().unwrap() - conn.shape[0]).normalize_or_zero();
            return dir.y.atan2(dir.x);
        }

        if let Ok(lane) = network.get_lane(&self.lane_id) {
            lane.heading_at_offset(self.position)
        } else {
            0.0
        }
    }
}
