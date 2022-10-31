use crate::grid::{Grid, GridErr, KartPoint};
use micromath::F32;

/// The entrypoint to the automatic emergency braking algorithm.
pub struct Aeb<const GridN: usize> {
    /// The occupancy grid
    grid: Grid<GridN>,
    /// Current velocity of the kart in m/s
    velocity: f32,
    /// Current angle of the virtual ackermann wheel, in degrees, positive is right.
    steering_angle: f32,
    /// The distance between axles, in meters
    wheelbase: f32,
    /// The collision box of the vehicle, defined by the top left point of the box relative to the center of the rear axel,
    /// and the bottom right point relative to the rear axel.
    collision_box: ((f32, f32), (f32, f32)),
    /// Rear axel to range sensor transform
    axel_to_range: KartPoint,
    /// The minimum allowed time to collision.
    min_ttc: f32,
}

impl<const GridN: usize> Aeb<GridN> {
    /// Configures AEB.
    ///
    /// # Config
    ///
    /// - start_vel: starting velocity, in m/s
    /// - start_wheel_angle: starting ackermann wheel angle, in degrees, positive is right.
    /// - wheelbase: distance between axles, in meters.
    /// - collision_box: collision bounding box, defined as the top left and bottom right points relative
    /// to the center of the rear axel. This is in normal coordinates.
    /// - axel_to_sensor: The transform of the axel to the front range sensor, in the kart frame.
    /// - min_ttc: the minimum allowed time to collision.
    pub fn new(
        start_vel: f32,
        start_wheel_angle: f32,
        wheelbase: f32,
        collision_box: ((f32, f32), (f32, f32)),
        axel_to_sensor: KartPoint,
        min_ttc: f32,
    ) -> Self {
        let grid = Grid::<GridN>::new();

        Self {
            grid,
            velocity: start_vel,
            steering_angle: start_wheel_angle,
            wheelbase,
            collision_box,
            min_ttc,
            axel_to_range: axel_to_sensor,
        }
    }

    /// Runs the collision checking algorithm with the current parameters.
    /// The passed points will also be added to the occupancy grid, if desired.
    /// After this call, the grid will be cleared.
    ///
    /// Returns true if the vehicle should brake.
    pub fn collision_check(&mut self, points: Option<&[KartPoint]>) -> bool {
        // Add points
        if let Some(p) = points {
            self.add_points(p)
        }

        const STEP_MS: usize = 10;
        // Convert ttc to millis
        let ttc = (self.min_ttc * 1000.0) as usize;

        // Collision check by integrating over our model
        for timestep in (0..ttc).step_by(STEP_MS) {
            // Predict position at time with definite integral of forward kinematics
            let (pos, heading) = self.predict_pos(timestep);

            let obb = self.create_obb(pos, heading);

            // Actually collision check
            if self.grid.polygon_collide(&obb) {
                self.grid.reset();
                return true;
            }
        }

        self.grid.reset();
        false
    }

    /// Predicts the future position of the kart given the currently configured params, at some time.
    ///
    /// Returns final (position, heading), where heading is in rad and negative if left.
    ///
    /// # Args
    /// - t: time in ms
    pub fn predict_pos(&self, t: usize) -> (KartPoint, f32) {
        let t = t as f32 / 1000.0;

        if self.steering_angle == 0.0 {
            let displacement = t * self.velocity;
            return (
                KartPoint(
                    displacement - self.axel_to_range.0,
                    0.0 - self.axel_to_range.1,
                ),
                0.0,
            );
        }

        // Forward kinematics equations integrated over time
        let x = (self.wheelbase * (self.velocity * F32(self.steering_angle.to_radians()) * t))
            .sin()
            / F32(self.steering_angle.to_radians());
        let y = (self.wheelbase
            * (-((self.velocity * F32(self.steering_angle.to_radians()) * t) / self.wheelbase)
                .cos()
                + 1.0))
            / F32(self.steering_angle.to_radians());

        // Integrate heading separately for OBB
        let heading =
            self.velocity * t * F32(self.steering_angle.to_radians()).tan() / self.wheelbase;

        // Translate the rear axel to its real location by pushing it back by its distance to the range sensor
        (
            KartPoint(x.0 - self.axel_to_range.0, y.0 - self.axel_to_range.1),
            heading.0,
        )
    }

    /// Creates the oriented bounding box given the karts position and yaw in rad.
    ///
    /// This box may lie outside the grid partially or totally.
    pub fn create_obb(&self, pos: KartPoint, yaw: f32) -> [((f32, f32), (f32, f32)); 4] {
        let n = self.grid.get_size();

        // Swap axis labels to keep me sane
        let (pred_y, pred_x) = pos.into();
        let ((tlx, tly), (brx, bry)) = self.collision_box;

        // Define the box's lines
        let tl_to_tr = (
            Self::rotate_point_about(KartPoint(pred_y + tly, pred_x + tlx), pos, yaw)
                .transform_to_grid_f(n)
                .raw(),
            Self::rotate_point_about(KartPoint(pred_y + bry, pred_x + tlx), pos, yaw)
                .transform_to_grid_f(n)
                .raw(),
        );
        let tr_to_br = (
            Self::rotate_point_about(KartPoint(pred_y + bry, pred_x + tlx), pos, yaw)
                .transform_to_grid_f(n)
                .raw(),
            Self::rotate_point_about(KartPoint(pred_y + bry, pred_x + brx), pos, yaw)
                .transform_to_grid_f(n)
                .raw(),
        );
        let br_to_bl = (
            Self::rotate_point_about(KartPoint(pred_y + bry, pred_x + brx), pos, yaw)
                .transform_to_grid_f(n)
                .raw(),
            Self::rotate_point_about(KartPoint(pred_y + tly, pred_x + brx), pos, yaw)
                .transform_to_grid_f(n)
                .raw(),
        );
        let bl_to_tl = (
            Self::rotate_point_about(KartPoint(pred_y + tly, pred_x + brx), pos, yaw)
                .transform_to_grid_f(n)
                .raw(),
            Self::rotate_point_about(KartPoint(pred_y + tly, pred_x + tlx), pos, yaw)
                .transform_to_grid_f(n)
                .raw(),
        );

        [tl_to_tr, tr_to_br, br_to_bl, bl_to_tl]
    }

    /// Rotates a point about another, by the angle in rad.
    fn rotate_point_about(p: KartPoint, origin: KartPoint, angle: f32) -> KartPoint {
        let angle = F32(angle);
        let (x1, y1) = p.into();
        // Matrix rotation
        let x2 = (x1 - origin.0) * angle.cos() - (y1 - origin.1) * angle.sin() + origin.0;
        let y2 = (x1 - origin.0) * angle.sin() + (y1 - origin.1) * angle.cos() + origin.1;

        KartPoint(x2.0, y2.0)
    }

    /// Updates the current velocity, in m/s.
    pub fn update_velocity(&mut self, velocity: f32) {
        self.velocity = velocity;
    }

    /// Updates the current ackermann steering angle, in degrees.
    pub fn update_steering(&mut self, steering_angle: f32) {
        self.steering_angle = steering_angle;
    }

    /// Adds points to the grid to be used during the next collision check.
    ///
    /// Points that are < 15cm from the data source will be filtered out.
    pub fn add_points(&mut self, points: &[KartPoint]) {
        for p in points {
            // If point is less than 15cm from kart, filter out
            if p.0 < 0.15 {
                continue;
            }

            // Transform point to grid, skipping if out of bounds
            let p = p.transform_to_grid(self.grid.get_size());
            let p = if let Ok(point) = p { point } else { continue };

            self.grid.mark_occupied(p);
        }
    }
}

#[cfg(test)]
mod test {
    extern crate std;
    use crate::grid::KartPoint;
    use crate::Aeb;
    use std::prelude::rust_2021::*;

    #[test]
    fn prediction_works() {
        let mut sys = Aeb::<71>::new(
            5.0,
            0.0,
            3.0,
            ((-0.5, 3.0), (0.5, -0.2)),
            KartPoint(3.1, 0.0),
            2.0,
        );

        // Straight line, with translation to be about sensor
        let pred = sys.predict_pos(1000);
        assert_eq!(pred.0 .0, 5.0 - 3.1);
        assert_eq!(pred.1, 0.0);

        sys.update_steering(-5.0);

        // Slight turn
        let pred = sys.predict_pos(1000);
        std::println!("{:?}", pred)
        //TODO create tests based off of IRL data
    }

    #[test]
    fn obb_created_correctly() {
        //TODO mostly test that points are rotated correctly
    }

    #[test]
    fn full_aeb_works() {
        //TODO also using the real data, after the above two
    }
}
