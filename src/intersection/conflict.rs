use crate::types::{ConnectionId, VehicleId};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Time window reservation for a vehicle traversing an intersection conflict area.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConflictReservation {
    pub vehicle_id: VehicleId,
    pub connection_id: ConnectionId,
    pub time_enter: f32,
    pub time_exit: f32,
}

/// Manages collision prevention and reservations across intersection conflict zones.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ConflictZoneManager {
    /// Active reservations per connection.
    pub reservations: Vec<ConflictReservation>,
    /// Set of currently occupied connections inside the junction.
    pub active_occupants: HashMap<ConnectionId, VehicleId>,
    /// Safety buffer time (seconds) between successive vehicles in conflicting paths.
    pub safety_buffer: f32,
}

impl ConflictZoneManager {
    pub fn new() -> Self {
        Self {
            reservations: Vec::new(),
            active_occupants: HashMap::new(),
            safety_buffer: 1.5, // 1.5 seconds safety buffer
        }
    }

    /// Advance time and prune expired reservations.
    pub fn clean_expired(&mut self, current_time: f32) {
        self.reservations.retain(|r| r.time_exit > current_time);
    }

    /// Check if a vehicle can reserve entry onto `connection_id` without colliding with conflicting reservations.
    ///
    /// * `conflicting_conns`: list of connection IDs that physically cross or merge with `connection_id`.
    /// * `time_enter`: projected entry time (seconds).
    /// * `time_exit`: projected exit time (seconds).
    pub fn can_reserve(
        &self,
        vehicle_id: VehicleId,
        connection_id: &ConnectionId,
        conflicting_conns: &[ConnectionId],
        time_enter: f32,
        time_exit: f32,
    ) -> bool {
        // If connection or any conflicting connection is actively occupied by another vehicle:
        for c in std::iter::once(connection_id).chain(conflicting_conns.iter()) {
            if let Some(&occupant) = self.active_occupants.get(c)
                && occupant != vehicle_id
            {
                return false;
            }
        }

        let start = time_enter - self.safety_buffer;
        let end = time_exit + self.safety_buffer;

        // Check overlapping reservations on any conflicting connection or same connection
        for r in &self.reservations {
            if r.vehicle_id == vehicle_id {
                continue;
            }

            let is_conflict =
                r.connection_id == *connection_id || conflicting_conns.contains(&r.connection_id);
            if is_conflict {
                // Check time window overlap: max(start1, start2) < min(end1, end2)
                if start < r.time_exit && end > r.time_enter {
                    return false;
                }
            }
        }

        true
    }

    /// Reserve the conflict zone for a vehicle.
    pub fn make_reservation(
        &mut self,
        vehicle_id: VehicleId,
        connection_id: ConnectionId,
        time_enter: f32,
        time_exit: f32,
    ) {
        // Remove prior reservation for this vehicle if present
        self.reservations.retain(|r| r.vehicle_id != vehicle_id);
        self.reservations.push(ConflictReservation {
            vehicle_id,
            connection_id,
            time_enter,
            time_exit,
        });
    }

    /// Mark that vehicle has physically entered the intersection connection.
    pub fn enter_junction(&mut self, vehicle_id: VehicleId, connection_id: ConnectionId) {
        self.active_occupants.insert(connection_id, vehicle_id);
    }

    /// Mark that vehicle has cleared the intersection connection.
    pub fn exit_junction(&mut self, vehicle_id: VehicleId, connection_id: &ConnectionId) {
        if self.active_occupants.get(connection_id) == Some(&vehicle_id) {
            self.active_occupants.remove(connection_id);
        }
        self.reservations.retain(|r| r.vehicle_id != vehicle_id);
    }
}
