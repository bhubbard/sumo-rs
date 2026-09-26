use crate::network::graph::Network;
use crate::types::{ConnectionId, TurnDirection, VehicleId};
use serde::{Deserialize, Serialize};
use std::collections::{HashMap, VecDeque};

/// Stop sign state tracking for a vehicle approaching an intersection.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StopSignTracker {
    /// Has the vehicle come to a complete standstill at the stop line?
    pub has_stopped: bool,
    /// Time spent at complete stop (seconds).
    pub stopped_duration: f32,
    /// Required wait duration at stop sign before proceeding (seconds).
    pub required_stop_time: f32,
}

impl Default for StopSignTracker {
    fn default() -> Self {
        Self {
            has_stopped: false,
            stopped_duration: 0.0,
            required_stop_time: 1.2, // 1.2s complete stop rule
        }
    }
}

/// Manages All-Way Stop (e.g. 4-way stop) arrival queues (FIFO right-of-way).
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct AllWayStopManager {
    /// Ordered queue of vehicles that have achieved a full stop at the intersection.
    pub stopped_queue: VecDeque<VehicleId>,
    /// Per-vehicle stop tracking state.
    pub vehicle_trackers: HashMap<VehicleId, StopSignTracker>,
}

impl AllWayStopManager {
    pub fn new() -> Self {
        Self::default()
    }

    /// Update vehicle stop status.
    /// Returns true if vehicle has satisfied stop sign conditions and is authorized to enter intersection.
    pub fn update_vehicle(
        &mut self,
        veh_id: VehicleId,
        speed: f32,
        dist_to_stop_line: f32,
        dt: f32,
    ) -> bool {
        let tracker = self.vehicle_trackers.entry(veh_id).or_default();

        // Check if vehicle has reached stop line and brought speed below standstill threshold
        if dist_to_stop_line <= 3.0 && speed < 0.2 {
            if !tracker.has_stopped {
                tracker.has_stopped = true;
                tracker.stopped_duration = dt;
                if !self.stopped_queue.contains(&veh_id) {
                    self.stopped_queue.push_back(veh_id);
                }
            } else {
                tracker.stopped_duration += dt;
            }
        }

        // Vehicle can proceed if it is at the front of the FIFO stopped queue
        // AND has completed its required stop duration.
        if tracker.has_stopped
            && tracker.stopped_duration >= tracker.required_stop_time
            && self.stopped_queue.front() == Some(&veh_id)
        {
            return true;
        }

        false
    }

    /// Notify that a vehicle has cleared the intersection, freeing the FIFO queue.
    pub fn on_vehicle_entered(&mut self, veh_id: &VehicleId) {
        if self.stopped_queue.front() == Some(veh_id) {
            self.stopped_queue.pop_front();
        } else {
            self.stopped_queue.retain(|v| v != veh_id);
        }
        self.vehicle_trackers.remove(veh_id);
    }
}

/// Evaluates Right-of-Way priority rules:
/// 1. Priority roads have right-of-way over minor roads.
/// 2. Left-turn yielding to oncoming straight/right-turn traffic.
/// 3. Yield to right (RightBeforeLeft rule).
#[derive(Debug, Clone)]
pub struct RightOfWayEvaluator;

impl RightOfWayEvaluator {
    /// Critical time gap (seconds) required to safely execute a maneuver in front of an approaching vehicle.
    pub const CRITICAL_TIME_GAP: f32 = 4.0;

    /// Evaluates if vehicle `ego_id` planning to take `conn_id` must yield to any approaching vehicle `other`.
    pub fn should_yield(
        network: &Network,
        conn_id: &ConnectionId,
        ego_dist_to_intersection: f32,
        ego_speed: f32,
        conflicting_conns: &[ConnectionId],
        approaching_conflicts: &[(VehicleId, ConnectionId, f32, f32)], // (id, conn, dist, speed)
    ) -> bool {
        let conn = match network.connections.get(conn_id) {
            Some(c) => c,
            None => return false,
        };

        // 1. Explicit Yield Rules: Check if conn explicitly yields to any conflicting connection
        for (other_veh, other_conn_id, other_dist, other_speed) in approaching_conflicts {
            if *other_veh == VehicleId(0) {
                continue;
            }

            let is_explicit_yield = conn.yield_to.contains(other_conn_id);
            let is_crossing_conflict = conn.conflict_connections.contains(other_conn_id)
                || conflicting_conns.contains(other_conn_id);

            // Left-turn yield to oncoming traffic:
            let is_left_turn_yield = conn.direction == TurnDirection::TurnLeft
                && network
                    .connections
                    .get(other_conn_id)
                    .map(|oc| {
                        oc.direction == TurnDirection::Straight
                            || oc.direction == TurnDirection::TurnRight
                    })
                    .unwrap_or(false);

            if is_explicit_yield || is_left_turn_yield || (is_crossing_conflict && !conn.priority) {
                // Check if other vehicle will arrive at the conflict point within critical time horizon
                let other_speed_pos = other_speed.max(0.5);
                let time_to_arrival = other_dist / other_speed_pos;

                let ego_speed_pos = ego_speed.max(0.5);
                let ego_time_to_arrival = ego_dist_to_intersection / ego_speed_pos;

                // If other vehicle arrives before ego clears, ego must yield!
                if time_to_arrival < ego_time_to_arrival + Self::CRITICAL_TIME_GAP
                    && *other_dist > -5.0
                {
                    return true;
                }
            }
        }

        false
    }
}
