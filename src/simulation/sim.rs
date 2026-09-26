use crate::car_following::LeaderInfo;
use crate::error::{Result, SumoError};
use crate::intersection::conflict::ConflictZoneManager;
use crate::intersection::right_of_way::{AllWayStopManager, RightOfWayEvaluator};
use crate::intersection::traffic_light::TrafficLightController;
use crate::lane_changing::{MobilContext, MobilModel};
use crate::network::graph::Network;
use crate::network::node::JunctionType;
use crate::simulation::vehicle::{Vehicle, VehicleState};
use crate::types::{LaneChangeDirection, LaneId, TrafficLightId, VehicleId};
use std::collections::HashMap;

/// Microscopic Traffic Simulation Engine.
#[derive(Debug, Clone, Default)]
pub struct TrafficSimulation {
    pub network: Network,
    pub vehicles: HashMap<VehicleId, Vehicle>,
    pub traffic_lights: HashMap<TrafficLightId, TrafficLightController>,
    pub conflict_manager: ConflictZoneManager,
    pub stop_manager: AllWayStopManager,
    pub sim_time: f32,
    pub step_count: u64,
}

impl TrafficSimulation {
    pub fn new(network: Network) -> Self {
        Self {
            network,
            vehicles: HashMap::new(),
            traffic_lights: HashMap::new(),
            conflict_manager: ConflictZoneManager::new(),
            stop_manager: AllWayStopManager::new(),
            sim_time: 0.0,
            step_count: 0,
        }
    }

    pub fn add_traffic_light(&mut self, controller: TrafficLightController) {
        self.traffic_lights
            .insert(controller.id.clone(), controller);
    }

    pub fn add_vehicle(&mut self, vehicle: Vehicle) -> Result<()> {
        if self.vehicles.contains_key(&vehicle.id) {
            return Err(SumoError::VehicleAlreadyExists(vehicle.id));
        }
        self.vehicles.insert(vehicle.id, vehicle);
        Ok(())
    }

    pub fn remove_vehicle(&mut self, id: &VehicleId) -> Result<Vehicle> {
        self.vehicles
            .remove(id)
            .ok_or(SumoError::VehicleNotFound(*id))
    }

    pub fn vehicle(&self, id: &VehicleId) -> Result<&Vehicle> {
        self.vehicles.get(id).ok_or(SumoError::VehicleNotFound(*id))
    }

    pub fn vehicle_mut(&mut self, id: &VehicleId) -> Result<&mut Vehicle> {
        self.vehicles
            .get_mut(id)
            .ok_or(SumoError::VehicleNotFound(*id))
    }

    pub fn vehicle_count(&self) -> usize {
        self.vehicles.len()
    }

    pub fn active_vehicles(&self) -> impl Iterator<Item = &Vehicle> {
        self.vehicles
            .values()
            .filter(|v| v.state != VehicleState::Arrived)
    }

    pub fn average_speed(&self) -> f32 {
        let active: Vec<&Vehicle> = self.active_vehicles().collect();
        if active.is_empty() {
            return 0.0;
        }
        let total: f32 = active.iter().map(|v| v.speed).sum();
        total / active.len() as f32
    }

    /// Advances the simulation by timestep `dt` (in seconds).
    pub fn step(&mut self, dt: f32) {
        self.sim_time += dt;
        self.step_count += 1;

        // 1. Advance traffic lights
        for tl in self.traffic_lights.values_mut() {
            tl.step(dt);
        }

        // 2. Prune expired conflict reservations
        self.conflict_manager.clean_expired(self.sim_time);

        // 3. Evaluate MOBIL lane changes
        self.evaluate_lane_changes(dt);

        // 4. Compute car-following accelerations for all vehicles
        let mut new_accelerations = HashMap::new();
        let vehicle_ids: Vec<VehicleId> = self.vehicles.keys().cloned().collect();

        for veh_id in &vehicle_ids {
            if self.vehicles[veh_id].state == VehicleState::Arrived {
                continue;
            }

            let leader_info = self.determine_effective_leader(veh_id);
            let veh = &self.vehicles[veh_id];
            let accel =
                veh.driver_model
                    .calculate_acceleration(veh.speed, leader_info.as_ref(), dt);
            new_accelerations.insert(*veh_id, accel);
        }

        // Apply accelerations
        for (veh_id, accel) in new_accelerations {
            if let Some(veh) = self.vehicles.get_mut(&veh_id) {
                veh.acceleration = accel;
            }
        }

        // 5. Numerical integration: update velocities and positions
        for veh_id in &vehicle_ids {
            let veh = self.vehicles.get_mut(veh_id).unwrap();
            if veh.state == VehicleState::Arrived {
                continue;
            }

            let new_speed = (veh.speed + veh.acceleration * dt)
                .max(0.0)
                .min(veh.max_speed);
            veh.speed = new_speed;
            let delta_s = new_speed * dt;
            veh.position += delta_s;
            veh.total_distance_traveled += delta_s;

            if new_speed < 0.1 {
                veh.total_waiting_time += dt;
            }
        }

        // 6. Handle lane / edge transitions and arrivals
        self.handle_transitions();
    }

    /// Evaluates MOBIL lane changing for vehicles on multi-lane edges.
    fn evaluate_lane_changes(&mut self, dt: f32) {
        let vehicle_ids: Vec<VehicleId> = self.vehicles.keys().cloned().collect();

        for veh_id in vehicle_ids {
            let (lane_id, pos, speed, mobil_params, current_accel) = {
                let v = &self.vehicles[&veh_id];
                if v.state == VehicleState::Arrived || v.current_connection.is_some() {
                    continue;
                }
                (
                    v.lane_id.clone(),
                    v.position,
                    v.speed,
                    v.mobil.clone(),
                    v.acceleration,
                )
            };

            let lane = match self.network.get_lane(&lane_id) {
                Ok(l) => l,
                Err(_) => continue,
            };

            let mobil = MobilModel::new(mobil_params);

            // Consider changing to left lane or right lane
            for dir in [LaneChangeDirection::Left, LaneChangeDirection::Right] {
                let target_lane_id = match dir {
                    LaneChangeDirection::Left => lane.left_lane.as_ref(),
                    LaneChangeDirection::Right => lane.right_lane.as_ref(),
                    _ => None,
                };

                let target_lane_id = match target_lane_id {
                    Some(id) => id,
                    None => continue,
                };

                // Find leader and follower in target lane
                let (target_leader, target_follower) =
                    self.find_surrounding_in_lane(target_lane_id, pos);
                let (curr_leader, curr_follower) = self.find_surrounding_in_lane(&lane_id, pos);

                let gap_to_new_leader = target_leader.map(|(_, l_pos, l_len)| l_pos - l_len - pos);
                let gap_to_new_follower = target_follower
                    .map(|(_, f_pos, _)| pos - self.vehicles[&veh_id].length - f_pos);

                // Ego projected acceleration in target lane
                let ego_leader_info = target_leader.map(|(lid, l_pos, l_len)| {
                    let l_speed = self.vehicles[&lid].speed;
                    LeaderInfo::new((l_pos - l_len - pos).max(0.0), l_speed)
                });
                let ego_accel_new = self.vehicles[&veh_id].driver_model.calculate_acceleration(
                    speed,
                    ego_leader_info.as_ref(),
                    dt,
                );

                // Target follower acceleration before & after
                let (new_f_accel_curr, new_f_accel_new) =
                    if let Some((fid, f_pos, _)) = target_follower {
                        let f_veh = &self.vehicles[&fid];
                        let f_accel_curr = f_veh.acceleration;
                        let f_new_gap = (pos - self.vehicles[&veh_id].length - f_pos).max(0.0);
                        let f_leader_info = LeaderInfo::new(f_new_gap, speed);
                        let f_accel_new = f_veh.driver_model.calculate_acceleration(
                            f_veh.speed,
                            Some(&f_leader_info),
                            dt,
                        );
                        (f_accel_curr, f_accel_new)
                    } else {
                        (0.0, 0.0)
                    };

                // Current follower acceleration before & after ego leaves
                let (old_f_accel_curr, old_f_accel_new) = if let Some((fid, f_pos, _)) =
                    curr_follower
                {
                    let f_veh = &self.vehicles[&fid];
                    let f_accel_curr = f_veh.acceleration;
                    let f_accel_new = if let Some((lid, l_pos, l_len)) = curr_leader {
                        let l_veh = &self.vehicles[&lid];
                        let gap = (l_pos - l_len - f_pos).max(0.0);
                        let l_info = LeaderInfo::new(gap, l_veh.speed);
                        f_veh
                            .driver_model
                            .calculate_acceleration(f_veh.speed, Some(&l_info), dt)
                    } else {
                        f_veh
                            .driver_model
                            .calculate_acceleration(f_veh.speed, None, dt)
                    };
                    (f_accel_curr, f_accel_new)
                } else {
                    (0.0, 0.0)
                };

                let ctx = MobilContext {
                    ego_accel_curr: current_accel,
                    ego_accel_new,
                    new_follower_accel_curr: new_f_accel_curr,
                    new_follower_accel_new: new_f_accel_new,
                    old_follower_accel_curr: old_f_accel_curr,
                    old_follower_accel_new: old_f_accel_new,
                    leader_accel_delta: 0.0,
                    gap_to_new_follower,
                    gap_to_new_leader,
                    direction: dir,
                };

                let decision = mobil.evaluate(&ctx);
                if decision.should_change {
                    // Execute lane change
                    if let Some(veh) = self.vehicles.get_mut(&veh_id) {
                        veh.lane_id = target_lane_id.clone();
                    }
                    break;
                }
            }
        }
    }

    /// Finds immediate leader and follower around `ref_pos` in `lane_id`.
    /// Returns `(Option<(leader_id, leader_pos, leader_len)>, Option<(follower_id, follower_pos, follower_len)>)`
    #[allow(clippy::type_complexity)]
    fn find_surrounding_in_lane(
        &self,
        lane_id: &LaneId,
        ref_pos: f32,
    ) -> (Option<(VehicleId, f32, f32)>, Option<(VehicleId, f32, f32)>) {
        let mut closest_leader: Option<(VehicleId, f32, f32)> = None;
        let mut min_leader_dist = f32::INFINITY;

        let mut closest_follower: Option<(VehicleId, f32, f32)> = None;
        let mut min_follower_dist = f32::INFINITY;

        for (id, v) in &self.vehicles {
            if v.state == VehicleState::Arrived || v.lane_id != *lane_id {
                continue;
            }

            if v.position > ref_pos {
                let dist = v.position - ref_pos;
                if dist < min_leader_dist {
                    min_leader_dist = dist;
                    closest_leader = Some((*id, v.position, v.length));
                }
            } else if v.position < ref_pos {
                let dist = ref_pos - v.position;
                if dist < min_follower_dist {
                    min_follower_dist = dist;
                    closest_follower = Some((*id, v.position, v.length));
                }
            }
        }

        (closest_leader, closest_follower)
    }

    /// Determines the effective leader for car-following:
    /// In-lane vehicle leader, or virtual leader at red lights / stop signs / yielding conflict zones.
    pub fn determine_effective_leader(&mut self, veh_id: &VehicleId) -> Option<LeaderInfo> {
        let veh = &self.vehicles[veh_id];
        let lane = self.network.get_lane(&veh.lane_id).ok()?;
        let remaining_in_lane = lane.length - veh.position;

        // 1. Check physical vehicle leader on current lane
        let mut closest_veh_gap = f32::INFINITY;
        let mut closest_veh_speed = 0.0;

        for (other_id, other) in &self.vehicles {
            if other_id == veh_id
                || other.state == VehicleState::Arrived
                || other.lane_id != veh.lane_id
            {
                continue;
            }
            if other.position > veh.position {
                let gap = (other.rear_bumper_pos() - veh.front_bumper_pos()).max(0.0);
                if gap < closest_veh_gap {
                    closest_veh_gap = gap;
                    closest_veh_speed = other.speed;
                }
            }
        }

        // If there is an immediate vehicle ahead in this lane, return that leader
        if closest_veh_gap < remaining_in_lane {
            return Some(LeaderInfo::new(closest_veh_gap, closest_veh_speed));
        }

        // 2. Check intersection constraints (Virtual Leader at Stop Line)
        let dist_to_stop_line = remaining_in_lane.max(0.0);
        let edge = self.network.get_edge(&lane.edge_id).ok()?;
        let to_node = self.network.get_node(&edge.to_node).ok()?;

        // Look up candidate outgoing connection from current lane
        let outgoing_conn_id = lane.outgoing_connections.first().cloned();

        if let Some(conn_id) = &outgoing_conn_id {
            // Check Traffic Light
            if let Some(tl_id) = &to_node.traffic_light_id
                && let Some(tl) = self.traffic_lights.get(tl_id)
            {
                let signal = tl.get_connection_signal(conn_id);
                if signal.is_stop() || (signal.is_caution() && dist_to_stop_line > 5.0) {
                    return Some(LeaderInfo::new(dist_to_stop_line, 0.0));
                }
            }

            // Check All-Way Stop or Priority Stop
            if to_node.junction_type == JunctionType::AllWayStop {
                let authorized = self.stop_manager.update_vehicle(
                    *veh_id,
                    veh.speed,
                    dist_to_stop_line,
                    0.1, // nominal check dt
                );
                if !authorized {
                    return Some(LeaderInfo::new(dist_to_stop_line, 0.0));
                }
            }

            // Check Right-of-Way & Left-turn yields
            if let Ok(conn) = self.network.get_connection(conn_id) {
                let mut approaching_conflicts = Vec::new();
                for (other_id, other) in &self.vehicles {
                    if other_id == veh_id || other.state == VehicleState::Arrived {
                        continue;
                    }
                    if let Ok(other_lane) = self.network.get_lane(&other.lane_id)
                        && let Some(other_conn) = other_lane.outgoing_connections.first()
                    {
                        let other_dist = other_lane.length - other.position;
                        approaching_conflicts.push((
                            *other_id,
                            other_conn.clone(),
                            other_dist,
                            other.speed,
                        ));
                    }
                }

                let must_yield = RightOfWayEvaluator::should_yield(
                    &self.network,
                    conn_id,
                    dist_to_stop_line,
                    veh.speed,
                    &conn.conflict_connections,
                    &approaching_conflicts,
                );

                if must_yield {
                    return Some(LeaderInfo::new(dist_to_stop_line, 0.0));
                }

                // Check Conflict Zone Reservation
                let speed_est = veh.speed.max(2.0);
                let time_enter = self.sim_time + dist_to_stop_line / speed_est;
                let time_exit = time_enter + conn.length / speed_est;

                if !self.conflict_manager.can_reserve(
                    *veh_id,
                    conn_id,
                    &conn.conflict_connections,
                    time_enter,
                    time_exit,
                ) {
                    return Some(LeaderInfo::new(dist_to_stop_line, 0.0));
                }
            }
        }

        // If physical vehicle was found further downstream
        if closest_veh_gap.is_finite() {
            Some(LeaderInfo::new(closest_veh_gap, closest_veh_speed))
        } else {
            None
        }
    }

    /// Handles vehicle transitions across lane boundaries, intersection connections, and routes.
    fn handle_transitions(&mut self) {
        let vehicle_ids: Vec<VehicleId> = self.vehicles.keys().cloned().collect();

        for veh_id in vehicle_ids {
            let (pos, lane_id, conn_opt) = {
                let v = &self.vehicles[&veh_id];
                (v.position, v.lane_id.clone(), v.current_connection.clone())
            };

            // If vehicle is currently crossing an intersection
            if let Some(conn_id) = conn_opt {
                if let Ok(conn) = self.network.get_connection(&conn_id)
                    && pos >= conn.length
                {
                    // Vehicle has cleared the intersection!
                    self.conflict_manager.exit_junction(veh_id, &conn_id);
                    self.stop_manager.on_vehicle_entered(&veh_id);

                    let surplus = pos - conn.length;
                    let veh = self.vehicles.get_mut(&veh_id).unwrap();
                    veh.lane_id = conn.to_lane.clone();
                    veh.position = surplus;
                    veh.current_connection = None;
                    veh.state = VehicleState::Active;

                    // Advance route index if applicable
                    if let Ok(new_lane) = self.network.get_lane(&veh.lane_id)
                        && veh.route_index + 1 < veh.route.len()
                        && veh.route[veh.route_index + 1] == new_lane.edge_id
                    {
                        veh.route_index += 1;
                    }
                }
                continue;
            }

            // Normal lane progression
            let lane_len = match self.network.get_lane(&lane_id) {
                Ok(l) => l.length,
                Err(_) => continue,
            };

            if pos >= lane_len {
                let surplus = pos - lane_len;
                let lane = &self.network.lanes[&lane_id];

                if let Some(conn_id) = lane.outgoing_connections.first().cloned() {
                    let conn = &self.network.connections[&conn_id];
                    let speed_est = self.vehicles[&veh_id].speed.max(2.0);
                    let time_exit = self.sim_time + conn.length / speed_est;

                    self.conflict_manager
                        .enter_junction(veh_id, conn_id.clone());
                    self.conflict_manager.make_reservation(
                        veh_id,
                        conn_id.clone(),
                        self.sim_time,
                        time_exit,
                    );

                    let veh = self.vehicles.get_mut(&veh_id).unwrap();
                    veh.current_connection = Some(conn_id);
                    veh.position = surplus;
                } else {
                    // End of route / road reached
                    let veh = self.vehicles.get_mut(&veh_id).unwrap();
                    veh.state = VehicleState::Arrived;
                    veh.speed = 0.0;
                }
            }
        }
    }

    /// Verifies that no two vehicles are in physical collision (overlapping bumper intervals on same lane/connection).
    /// Returns any collision pairs detected.
    pub fn check_collisions(&self) -> Vec<(VehicleId, VehicleId, f32)> {
        let mut collisions = Vec::new();
        let vehicles: Vec<&Vehicle> = self.active_vehicles().collect();

        for i in 0..vehicles.len() {
            for j in (i + 1)..vehicles.len() {
                let v1 = vehicles[i];
                let v2 = vehicles[j];

                // If on same lane and not arrived
                if v1.lane_id == v2.lane_id && v1.current_connection == v2.current_connection {
                    let front1 = v1.front_bumper_pos();
                    let rear1 = v1.rear_bumper_pos();
                    let front2 = v2.front_bumper_pos();
                    let rear2 = v2.rear_bumper_pos();

                    // Check 1D interval overlap
                    if front1 > rear2 && front2 > rear1 {
                        let overlap = (front1.min(front2) - rear1.max(rear2)).abs();
                        collisions.push((v1.id, v2.id, overlap));
                    }
                }
            }
        }

        collisions
    }
}
