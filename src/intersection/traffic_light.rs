use crate::types::{ConnectionId, LaneId, TrafficLightId};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Signal display color for a lane or intersection connection.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum SignalColor {
    Green,
    Yellow,
    Red,
    RedYellow, // Optional European preparation phase
    Off,
}

impl SignalColor {
    pub fn is_go(&self) -> bool {
        matches!(self, SignalColor::Green)
    }

    pub fn is_stop(&self) -> bool {
        matches!(self, SignalColor::Red | SignalColor::RedYellow)
    }

    pub fn is_caution(&self) -> bool {
        matches!(self, SignalColor::Yellow)
    }
}

/// A single phase within a traffic light cycle.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SignalPhase {
    pub name: String,
    /// Duration of this phase in seconds.
    pub duration: f32,
    /// Signal color assigned to connections during this phase.
    pub connection_signals: HashMap<ConnectionId, SignalColor>,
    /// Signal color assigned to specific incoming lanes.
    pub lane_signals: HashMap<LaneId, SignalColor>,
}

impl SignalPhase {
    pub fn new(name: impl Into<String>, duration: f32) -> Self {
        Self {
            name: name.into(),
            duration,
            connection_signals: HashMap::new(),
            lane_signals: HashMap::new(),
        }
    }

    pub fn with_connection_signal(mut self, conn_id: ConnectionId, color: SignalColor) -> Self {
        self.connection_signals.insert(conn_id, color);
        self
    }

    pub fn with_lane_signal(mut self, lane_id: LaneId, color: SignalColor) -> Self {
        self.lane_signals.insert(lane_id, color);
        self
    }
}

/// Traffic Light Controller managing phases, timing, and transitions.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TrafficLightController {
    pub id: TrafficLightId,
    pub phases: Vec<SignalPhase>,
    pub current_phase_index: usize,
    pub time_in_current_phase: f32,
    pub is_enabled: bool,
}

impl TrafficLightController {
    pub fn new(id: TrafficLightId, phases: Vec<SignalPhase>) -> Self {
        Self {
            id,
            phases,
            current_phase_index: 0,
            time_in_current_phase: 0.0,
            is_enabled: true,
        }
    }

    /// Advance the traffic light timer by `dt` seconds.
    pub fn step(&mut self, dt: f32) {
        if !self.is_enabled || self.phases.is_empty() {
            return;
        }

        self.time_in_current_phase += dt;
        let current_duration = self.phases[self.current_phase_index].duration;

        if self.time_in_current_phase >= current_duration {
            self.time_in_current_phase -= current_duration;
            self.current_phase_index = (self.current_phase_index + 1) % self.phases.len();
        }
    }

    /// Returns the active phase.
    pub fn current_phase(&self) -> Option<&SignalPhase> {
        self.phases.get(self.current_phase_index)
    }

    /// Total cycle length in seconds.
    pub fn cycle_length(&self) -> f32 {
        self.phases.iter().map(|p| p.duration).sum()
    }

    /// Query signal color for a specific connection.
    pub fn get_connection_signal(&self, conn_id: &ConnectionId) -> SignalColor {
        if !self.is_enabled {
            return SignalColor::Off;
        }
        self.current_phase()
            .and_then(|p| p.connection_signals.get(conn_id).copied())
            .unwrap_or(SignalColor::Red)
    }

    /// Query signal color for an incoming lane.
    pub fn get_lane_signal(&self, lane_id: &LaneId) -> SignalColor {
        if !self.is_enabled {
            return SignalColor::Off;
        }
        self.current_phase()
            .and_then(|p| p.lane_signals.get(lane_id).copied())
            .unwrap_or(SignalColor::Red)
    }

    /// Creates a standard 4-way intersection signal program:
    /// Phase 1: North-South Green (e.g. 25s)
    /// Phase 2: North-South Yellow (e.g. 3s)
    /// Phase 3: All-Red clearance interval (e.g. 2s)
    /// Phase 4: East-West Green (e.g. 25s)
    /// Phase 5: East-West Yellow (e.g. 3s)
    /// Phase 6: All-Red clearance interval (e.g. 2s)
    pub fn standard_4way(
        id: TrafficLightId,
        ns_conns: &[ConnectionId],
        ew_conns: &[ConnectionId],
        green_time: f32,
        yellow_time: f32,
        all_red_time: f32,
    ) -> Self {
        let mut p1 = SignalPhase::new("NS_Green", green_time);
        for c in ns_conns {
            p1 = p1.with_connection_signal(c.clone(), SignalColor::Green);
        }
        for c in ew_conns {
            p1 = p1.with_connection_signal(c.clone(), SignalColor::Red);
        }

        let mut p2 = SignalPhase::new("NS_Yellow", yellow_time);
        for c in ns_conns {
            p2 = p2.with_connection_signal(c.clone(), SignalColor::Yellow);
        }
        for c in ew_conns {
            p2 = p2.with_connection_signal(c.clone(), SignalColor::Red);
        }

        let mut p3 = SignalPhase::new("Clearance_NS", all_red_time);
        for c in ns_conns.iter().chain(ew_conns.iter()) {
            p3 = p3.with_connection_signal(c.clone(), SignalColor::Red);
        }

        let mut p4 = SignalPhase::new("EW_Green", green_time);
        for c in ew_conns {
            p4 = p4.with_connection_signal(c.clone(), SignalColor::Green);
        }
        for c in ns_conns {
            p4 = p4.with_connection_signal(c.clone(), SignalColor::Red);
        }

        let mut p5 = SignalPhase::new("EW_Yellow", yellow_time);
        for c in ew_conns {
            p5 = p5.with_connection_signal(c.clone(), SignalColor::Yellow);
        }
        for c in ns_conns {
            p5 = p5.with_connection_signal(c.clone(), SignalColor::Red);
        }

        let mut p6 = SignalPhase::new("Clearance_EW", all_red_time);
        for c in ns_conns.iter().chain(ew_conns.iter()) {
            p6 = p6.with_connection_signal(c.clone(), SignalColor::Red);
        }

        Self::new(id, vec![p1, p2, p3, p4, p5, p6])
    }
}
