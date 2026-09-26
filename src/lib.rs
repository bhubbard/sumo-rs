//! # sumo-rs
//!
//! Pure Rust microscopic traffic simulation engine featuring IDM, Krauss car-following,
//! MOBIL lane changing, and intersection right-of-way translated from Eclipse SUMO (C++).
//!
//! Designed for real-time microscopic traffic simulation in autonomous vehicle research,
//! transportation engineering, and open-world video games (such as Bevy RPGs).
//!
//! ## Core Features
//! - **Car-Following Models**:
//!   - **Krauss Model**: Safe speed $v_{safe} = -g \cdot \tau + \sqrt{(g \cdot \tau)^2 + v_l^2 + 2 \cdot b \cdot g}$,
//!     desired speed calculation, acceleration with stochastic imperfection (driver dawdling).
//!   - **Intelligent Driver Model (IDM)**:
//!     $\dot{v} = a \left[ 1 - \left(\frac{v}{v_0}\right)^\delta - \left(\frac{s^*(v, \Delta v)}{s}\right)^2 \right]$
//!     with dynamic desired gap $s^*(v, \Delta v) = s_0 + v \cdot T + \frac{v \cdot \Delta v}{2 \sqrt{a \cdot b}}$.
//! - **Lane Changing Model (MOBIL)**:
//!   - Minimizing Overall Braking Induced by Lane changes (Kesting, Treiber, Helbing).
//!   - Incentive criterion: $a_{new} - a_{curr} + p \cdot (a_{new,follower} - a_{curr,follower} + a_{new,leader} - a_{curr,leader}) > \Delta a_{th} \pm a_{bias}$.
//!   - Safety criterion: follower braking in target lane does not exceed $b_{safe}$.
//! - **Intersection Right-of-Way & Traffic Lights**:
//!   - Priority rules (priority roads, yield to right, yield to oncoming traffic on left turns, stop signs).
//!   - Signal phase and timing (green, yellow, all-red clearance intervals).
//!   - Junction reservation and conflict zone collision prevention.
//! - **Road Network Topology**:
//!   - Nodes (intersections), Edges (road segments), Lanes with speed limits, lengths, and connectivity.
//!   - Graph shortest-path routing (Dijkstra).
//! - **Bevy Compatibility**:
//!   - Transforms directly exportable to `glam::Vec3` and `glam::Quat` for 3D open-world rendering.

pub mod bevy_adapter;
pub mod car_following;
pub mod error;
pub mod intersection;
pub mod lane_changing;
pub mod network;
pub mod prelude;
pub mod simulation;
pub mod types;

// Re-exports of top-level types for ergonomic usage
pub use bevy_adapter::{BevyTransform, vehicle_to_bevy_transform};
pub use car_following::{
    CarFollowingModel, DriverModel, IdmModel, IdmParameters, KraussModel, KraussParameters,
    LeaderInfo,
};
pub use error::{Result, SumoError};
pub use intersection::{
    AllWayStopManager, ConflictReservation, ConflictZoneManager, RightOfWayEvaluator, SignalColor,
    SignalPhase, StopSignTracker, TrafficLightController,
};
pub use lane_changing::{MobilContext, MobilDecision, MobilModel, MobilParameters};
pub use network::{Connection, Edge, JunctionType, Lane, Network, NetworkBuilder, Node};
pub use simulation::{TrafficSimulation, Vehicle, VehicleState};
pub use types::{
    ConnectionId, EdgeId, LaneChangeDirection, LaneId, NodeId, TrafficLightId, TurnDirection,
    VehicleId,
};
