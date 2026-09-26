# sumo-rs

[![CI](https://github.com/bhubbard/sumo-rs/actions/workflows/ci.yml/badge.svg)](https://github.com/bhubbard/sumo-rs/actions)
[![Website](https://img.shields.io/badge/website-GitHub%20Pages-10B981?style=flat&logo=github)](https://bhubbard.github.io/sumo-rs/)
[![Crates.io](https://img.shields.io/badge/crates.io-v0.1.0-orange.svg)](https://crates.io/crates/sumo-rs)
[![Documentation](https://docs.rs/sumo-rs/badge.svg)](https://docs.rs/sumo-rs)
[![License](https://img.shields.io/badge/license-MIT%2FApache--2.0-blue.svg)](LICENSE)

A pure Rust microscopic traffic simulation engine featuring **Intelligent Driver Model (IDM)**, **Krauss car-following**, **MOBIL lane-changing**, and **intersection right-of-way** translated directly from Eclipse SUMO (C++).

Built with deterministic simulation stepping, high performance, and zero runtime dependencies on C++ or Python—specifically architected for **autonomous vehicle simulation**, **traffic engineering**, and **open-world game engines like Bevy**.

---

## Features

- **Car-Following Models**:
  - **Krauss Model**: Safe speed $v_{safe}$ calculation, desired velocity bounds, and stochastic driver imperfection (dawdling).
  - **Intelligent Driver Model (IDM)**: Continuous acceleration dynamics $\dot{v}$ with dynamic desired gap $s^*(v, \Delta v)$ and comfortable deceleration.
- **Lane-Changing Model (MOBIL)**:
  - *Minimizing Overall Braking Induced by Lane changes* (Kesting, Treiber, Helbing).
  - Incentive criterion incorporating driver politeness $p$, switching threshold $\Delta a_{th}$, and asymmetric lane preference $a_{bias}$ (keep-right rule).
  - Safety criterion enforcing maximum safe follower braking $b_{safe}$.
- **Intersection Right-of-Way & Traffic Lights**:
  - **Priority Rules**: Priority roads vs. minor roads, uncontrolled yield-to-right (right-before-left), left-turn yield to oncoming straight traffic, and 4-way stop signs with FIFO arrival arbitration.
  - **Signal Phase & Timing (SPaT)**: Fully configurable multi-phase traffic light controllers (Green, Yellow, All-Red clearance intervals).
  - **Conflict Zone Reservation**: Time-window reservation system that prevents intersecting vehicle collisions inside junctions.
- **Road Network Topology**:
  - Graph representation with `Node` (intersections), `Edge` (directed segments), `Lane` (with polyline geometries, speed limits, and neighbor connectivity), and `Connection` (internal intersection links).
  - Built-in Dijkstra shortest-path routing across the road network.
- **Game Engine & Bevy Integration**:
  - Pure Rust, zero unsafe code.
  - Built-in adapter for `glam` (`Vec2`, `Vec3`, `Quat`) converting 2D road lane coordinates directly into 3D world transforms for Bevy entities.

---

## Algorithmic Details

### 1. Krauss Car-Following Model

The Krauss model (Krauß 1998, as implemented in Eclipse SUMO) ensures collision-free car-following by calculating a safe speed $v_{safe}$:

$$v_{safe} = -g \cdot \tau + \sqrt{(g \cdot \tau)^2 + v_l^2 + 2 \cdot b \cdot g}$$

where:
- $g = \max(0, \text{gap} - s_0)$: Net bumper-to-bumper distance to the leading vehicle minus standstill minimum gap.
- $\tau$: Driver reaction time / headway time (seconds).
- $v_l$: Speed of the leading vehicle (m/s).
- $b$: Comfortable/maximum deceleration (m/s²).

The driver computes desired speed:

$$v_{des} = \min(v + a \cdot \Delta t, v_0, v_{safe})$$

To simulate human imperfection, stochastic dawdling is applied using parameter $\sigma \in [0, 1]$:

$$v(t + \Delta t) = \max\left(0, v_{des} - \sigma \cdot \min(a \cdot \Delta t, v_{des}) \cdot \eta\right), \quad \eta \sim \mathcal{U}(0, 1)$$

---

### 2. Intelligent Driver Model (IDM)

The Intelligent Driver Model (Treiber, Hennecke, and Helbing 2000) provides smooth, realistic acceleration and deceleration curves:

$$\dot{v} = a \left[ 1 - \left(\frac{v}{v_0}\right)^\delta - \left(\frac{s^*(v, \Delta v)}{s}\right)^2 \right]$$

with the dynamic desired gap $s^*(v, \Delta v)$:

$$s^*(v, \Delta v) = s_0 + v \cdot T + \frac{v \cdot \Delta v}{2 \sqrt{a \cdot b}}$$

where:
- $v$: Current vehicle speed.
- $v_0$: Desired free-flow speed.
- $\Delta v = v - v_l$: Approach rate / closing speed (positive when closing in).
- $s$: Actual bumper-to-bumper distance to leader.
- $s_0$: Jam distance / minimum standstill gap.
- $T$: Safe time headway.
- $a$: Maximum acceleration.
- $b$: Comfortable deceleration.
- $\delta$: Acceleration exponent (typically 4.0).

---

### 3. MOBIL Lane-Changing Model

MOBIL evaluates whether changing lanes benefits the ego vehicle while not penalizing surrounding traffic excessively:

#### Incentive Criterion
$$a_{new} - a_{curr} + p \cdot \left( a_{new,f} - a_{curr,f} + a_{new,l} - a_{curr,l} + a_{new,old\_f} - a_{curr,old\_f} \right) > \Delta a_{th} \pm a_{bias}$$

where:
- $a_{curr}, a_{new}$: Ego vehicle acceleration before and after prospective lane change.
- $a_{curr,f}, a_{new,f}$: Target lane follower acceleration before and after change.
- $a_{curr,old\_f}, a_{new,old\_f}$: Trailing vehicle acceleration in the original lane after ego departs.
- $p \in [0, 1]$: Politeness factor ($p=0$: selfish, $p=1$: altruistic).
- $\Delta a_{th}$: Switching threshold to prevent lane oscillation.
- $a_{bias}$: Asymmetric bias favoring the right lane (keep-right rule).

#### Safety Criterion
The prospective follower in the target lane must not exceed maximum safe braking:

$$a_{new,f} \ge -b_{safe} \quad \text{and} \quad a_{new} \ge -b_{safe}$$

---

### 4. Intersection Right-of-Way & Collision Prevention

1. **Traffic Light Controllers (SPaT)**:
   - Configurable cycle programs supporting Green, Yellow, All-Red clearance intervals, and Red phases per connection.
2. **Left-Turn Yielding**:
   - Vehicles executing unprotected left turns yield to oncoming straight and right-turning vehicles if their projected arrival time falls within the critical gap $\tau_{crit} = 4.0\text{s}$.
3. **All-Way Stop (FIFO)**:
   - Evaluates full stop criteria ($v < 0.2\text{ m/s}$, $s_{stop} \le 3.0\text{m}$) and enforces mandatory dwell time before granting entry in strict First-In, First-Out sequence.
4. **Junction Conflict Zone Reservation**:
   - Spatially intersects crossing connection trajectories. Vehicles project time windows $[t_{enter} - t_{buffer}, t_{exit} + t_{buffer}]$ and dynamically reserve conflict areas, mathematically guaranteeing collision-free intersection traversals.

---

## Installation

Add `sumo-rs` to your `Cargo.toml`:

```toml
[dependencies]
sumo-rs = "0.1"
glam = "0.29"
```

---

## Quickstart

### Basic Multi-Lane Highway with MOBIL

```rust
use glam::Vec2;
use sumo_rs::prelude::*;

fn main() -> sumo_rs::Result<()> {
    let mut network = Network::new();

    // Build a 3-lane 1km highway
    NetworkBuilder::build_straight_road(
        &mut network,
        NodeId::new("start"),
        NodeId::new("end"),
        EdgeId::new("highway"),
        Vec2::new(0.0, 0.0),
        Vec2::new(1000.0, 0.0),
        3,     // 3 lanes
        33.33, // 120 km/h speed limit
    );

    let mut sim = TrafficSimulation::new(network);

    // Spawn a slow truck in the right lane
    let truck = Vehicle::new(
        VehicleId(1),
        LaneId::new("highway_0"),
        50.0,
        15.0,
        DriverModel::idm_default(),
    );
    sim.add_vehicle(truck)?;

    // Spawn a faster car behind it with MOBIL enabled
    let mut car = Vehicle::new(
        VehicleId(2),
        LaneId::new("highway_0"),
        10.0,
        25.0,
        DriverModel::krauss_default(),
    );
    car.mobil = MobilParameters {
        politeness: 0.2,
        switching_threshold: 0.15,
        safe_decel: 4.0,
        bias_right: 0.1,
        min_clearance: 2.5,
    };
    sim.add_vehicle(car)?;

    // Step simulation
    for _ in 0..200 {
        sim.step(0.1);
    }

    println!("Active vehicles: {}", sim.active_vehicles().count());
    assert!(sim.check_collisions().is_empty());

    Ok(())
}
```

---

### Bevy RPG Integration Example

In your Bevy systems, you can step `TrafficSimulation` as a Resource and translate vehicles into 3D world transforms:

```rust
use bevy::prelude::*;
use sumo_rs::bevy_adapter::vehicle_to_bevy_transform;
use sumo_rs::simulation::TrafficSimulation;
use sumo_rs::types::VehicleId;

#[derive(Component)]
pub struct VehicleEntity(pub VehicleId);

pub fn update_traffic_system(
    mut sim: ResMut<TrafficSimulation>,
    time: Res<Time>,
    mut query: Query<(&VehicleEntity, &mut Transform)>,
) {
    // Step simulation at fixed or delta timestep
    sim.step(time.delta_secs());

    for (veh_entity, mut transform) in query.iter_mut() {
        if let Ok(veh) = sim.vehicle(&veh_entity.0) {
            let bevy_tf = vehicle_to_bevy_transform(veh, &sim.network, 0.0);
            transform.translation = bevy_tf.translation;
            transform.rotation = bevy_tf.rotation;
        }
    }
}
```

---

## Examples

Run the included examples:

```bash
# 3-lane highway overtaking simulation
cargo run --example highway_mobil

# 4-way intersection with SPaT traffic signal cycle and 3D Bevy transforms
cargo run --example four_way_intersection
```

---

## Testing

Run the comprehensive unit test suite:

```bash
cargo test
```

Test coverage includes:
- Krauss safe speed mathematical verification and stochastic dawdling bounds.
- IDM dynamic headway, continuous acceleration, and emergency braking curves.
- MOBIL incentive evaluation, politeness filtering, and follower deceleration safety guards.
- Traffic light state cycling (Green/Yellow/Red clearance).
- Left-turn yield arbitration and All-Way stop FIFO queues.
- Junction conflict reservation and multi-vehicle collision checking.

---

## License

Licensed under either of:
- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.
