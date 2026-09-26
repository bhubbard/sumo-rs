use crate::types::{ConnectionId, EdgeId, LaneId};
use glam::Vec2;
use serde::{Deserialize, Serialize};

/// Represents an individual driving lane within an edge or intersection internal link.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Lane {
    pub id: LaneId,
    pub edge_id: EdgeId,
    pub index: usize,
    pub length: f32,
    pub width: f32,
    pub speed_limit: f32, // m/s
    pub shape: Vec<Vec2>,
    pub left_lane: Option<LaneId>,
    pub right_lane: Option<LaneId>,
    pub outgoing_connections: Vec<ConnectionId>,
}

impl Lane {
    pub fn new(
        id: LaneId,
        edge_id: EdgeId,
        index: usize,
        shape: Vec<Vec2>,
        speed_limit: f32,
    ) -> Self {
        let length = calculate_polyline_length(&shape);
        Self {
            id,
            edge_id,
            index,
            length,
            width: 3.5, // standard 3.5m lane width
            speed_limit,
            shape,
            left_lane: None,
            right_lane: None,
            outgoing_connections: Vec::new(),
        }
    }

    pub fn with_width(mut self, width: f32) -> Self {
        self.width = width;
        self
    }

    pub fn with_neighbors(mut self, left: Option<LaneId>, right: Option<LaneId>) -> Self {
        self.left_lane = left;
        self.right_lane = right;
        self
    }

    /// Evaluates the 2D world coordinates at longitudinal position $s$ (offset along lane from 0 to length).
    pub fn position_at_offset(&self, offset: f32) -> Vec2 {
        if self.shape.is_empty() {
            return Vec2::ZERO;
        }
        if self.shape.len() == 1 {
            return self.shape[0];
        }

        let target_s = offset.clamp(0.0, self.length);
        let mut accumulated_dist = 0.0;

        for window in self.shape.windows(2) {
            let p0 = window[0];
            let p1 = window[1];
            let segment_len = (p1 - p0).length();

            if segment_len > 1e-6 {
                if accumulated_dist + segment_len >= target_s {
                    let factor = (target_s - accumulated_dist) / segment_len;
                    return p0.lerp(p1, factor);
                }
                accumulated_dist += segment_len;
            }
        }

        *self.shape.last().unwrap()
    }

    /// Tangent direction vector at longitudinal offset $s$.
    pub fn tangent_at_offset(&self, offset: f32) -> Vec2 {
        if self.shape.len() < 2 {
            return Vec2::X;
        }

        let target_s = offset.clamp(0.0, self.length);
        let mut accumulated_dist = 0.0;

        for window in self.shape.windows(2) {
            let p0 = window[0];
            let p1 = window[1];
            let segment_len = (p1 - p0).length();

            if segment_len > 1e-6 {
                if accumulated_dist + segment_len >= target_s
                    || (target_s == self.length
                        && accumulated_dist + segment_len >= self.length - 1e-5)
                {
                    return (p1 - p0).normalize();
                }
                accumulated_dist += segment_len;
            }
        }

        let n = self.shape.len();
        (self.shape[n - 1] - self.shape[n - 2]).normalize_or_zero()
    }

    /// Heading angle in radians at offset $s$.
    pub fn heading_at_offset(&self, offset: f32) -> f32 {
        let t = self.tangent_at_offset(offset);
        t.y.atan2(t.x)
    }

    /// Project a 2D world coordinate onto the lane centerline.
    /// Returns `(offset_s, lateral_distance)`.
    pub fn project_point(&self, point: Vec2) -> (f32, f32) {
        if self.shape.len() < 2 {
            return (0.0, (point - self.position_at_offset(0.0)).length());
        }

        let mut best_dist_sq = f32::INFINITY;
        let mut best_s = 0.0;
        let mut accumulated_dist = 0.0;

        for window in self.shape.windows(2) {
            let p0 = window[0];
            let p1 = window[1];
            let seg = p1 - p0;
            let seg_len = seg.length();

            if seg_len > 1e-6 {
                let v = point - p0;
                let t = (v.dot(seg) / (seg_len * seg_len)).clamp(0.0, 1.0);
                let proj = p0 + seg * t;
                let dist_sq = (point - proj).length_squared();

                if dist_sq < best_dist_sq {
                    best_dist_sq = dist_sq;
                    best_s = accumulated_dist + t * seg_len;
                }
                accumulated_dist += seg_len;
            }
        }

        (best_s, best_dist_sq.sqrt())
    }
}

pub fn calculate_polyline_length(points: &[Vec2]) -> f32 {
    if points.len() < 2 {
        return 0.0;
    }
    points
        .windows(2)
        .map(|window| (window[1] - window[0]).length())
        .sum()
}
