use crate::grid::{Grid, GridErr, KartPoint};
use micromath::F32;

/// The entrypoint to the automatic emergency braking algorithm.
pub struct Aeb<const GridN: usize> {
    /// The occupancy grid
    grid: Grid<GridN>,
    /// Current velocity of the kart in m/s
    velocity: f32,
    /// Current angle of the virtual ackermann wheel, in degrees
    steering_angle: f32,
    /// The distance between axles, in meters
    wheelbase: f32,
    /// The collision box of the vehicle, defined by the top left point of the box relative to the center of the rear axel,
    /// and the bottom right point relative to the rear axel.
    collision_box: ((f32, f32), (f32, f32)),
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
    /// - min_ttc: the minimum allowed time to collision.
    pub fn new(
        start_vel: f32,
        start_wheel_angle: f32,
        wheelbase: f32,
        collision_box: ((f32, f32), (f32, f32)),
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

        //TODO determine step
        const STEP_MS: usize = 10;
        // Convert ttc to millis
        let ttc = (self.min_ttc / 1000.0) as usize;
        // Track yaw over time, in rad
        let mut current_yaw = 0.0f32;

        // Collision check by integrating over our model
        for timestep in (0..ttc).step_by(STEP_MS) {
            let (pos, yaw) = self.predict_pos(timestep);
            current_yaw += yaw;

            if let Ok(obb) = self.create_obb(pos, current_yaw) {
                // Actually collision check
                if self.grid.polygon_collide(&obb) {
                    self.grid.reset();
                    return true;
                }
            } else {
                // Since we are OOB now, just exit
                self.grid.reset();
                break;
            }
        }

        self.grid.reset();
        false
    }

    /// Predicts the future position of the kart given the currently configured params, and some
    /// timestep t. Second tuple element is yaw change in radians, negative is left.
    ///
    /// # Args
    /// - t: timestep in ms
    fn predict_pos(&self, t: usize) -> (KartPoint, f32) {
        //TODO this is wrong, figure out why
        let t = t as f32 / 1000.0;
        let displacement = t * self.velocity;

        if self.steering_angle == 0.0 {
            return (KartPoint(displacement, 0.0), 0.0);
        }

        let turn_radius = self.wheelbase / F32(self.steering_angle.to_radians()).tan();
        let orientation_change = displacement / turn_radius;
        let final_normal_dir = (-orientation_change.sin(), orientation_change.cos());

        let new_pos = KartPoint(
            (turn_radius * final_normal_dir.1).into(),
            (turn_radius * final_normal_dir.0).into(),
        );

        (new_pos, orientation_change.0)
    }

    /// Creates the oriented bounding box given the karts position and yaw in rad.
    fn create_obb(
        &self,
        pos: KartPoint,
        yaw: f32,
    ) -> Result<[((f32, f32), (f32, f32)); 4], GridErr> {
        let n = self.grid.get_size();

        // Swap axis labels to keep me sane
        let (pred_y, pred_x) = pos.into();
        let ((tlx, tly), (brx, bry)) = self.collision_box;

        // Define the box's lines
        let tl_to_tr = (
            Self::rotate_point_about(KartPoint(pred_x + tlx, pred_y + tly), pos, yaw)
                .transform_to_grid_f(n)?
                .raw(),
            Self::rotate_point_about(KartPoint(pred_x + brx, pred_y + tly), pos, yaw)
                .transform_to_grid_f(n)?
                .raw(),
        );
        let tr_to_br = (
            Self::rotate_point_about(KartPoint(pred_x + brx, pred_y + tly), pos, yaw)
                .transform_to_grid_f(n)?
                .raw(),
            Self::rotate_point_about(KartPoint(pred_x + brx, pred_y + bry), pos, yaw)
                .transform_to_grid_f(n)?
                .raw(),
        );
        let br_to_bl = (
            Self::rotate_point_about(KartPoint(pred_x + brx, pred_y + bry), pos, yaw)
                .transform_to_grid_f(n)?
                .raw(),
            Self::rotate_point_about(KartPoint(pred_x + tlx, pred_y + bry), pos, yaw)
                .transform_to_grid_f(n)?
                .raw(),
        );
        let bl_to_tl = (
            Self::rotate_point_about(KartPoint(pred_x + tlx, pred_y + bry), pos, yaw)
                .transform_to_grid_f(n)?
                .raw(),
            Self::rotate_point_about(KartPoint(pred_x + tlx, pred_y + tly), pos, yaw)
                .transform_to_grid_f(n)?
                .raw(),
        );

        Ok([tl_to_tr, tr_to_br, br_to_bl, bl_to_tl])
    }

    /// Rotates a point about another, by the angle in yaw.
    fn rotate_point_about(p: KartPoint, origin: KartPoint, angle: f32) -> KartPoint {
        let angle = F32(angle);
        let (y1, x1) = p.into();
        // Matrix rotation
        let x2 = x1 * angle.cos() - y1 * angle.sin();
        let y2 = x1 * angle.sin() + y1 * angle.cos();

        KartPoint(x2.0 + origin.1, y2.0 + origin.0)
    }

    /// Updates the current velocity
    pub fn update_velocity(&mut self, velocity: f32) {
        self.velocity = velocity;
    }

    /// Updates the current ackermann steering angle, in degrees
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
    use crate::Aeb;
    use std::prelude::rust_2021::*;

    #[test]
    fn prediction_works() {
        let mut sys = Aeb::<71>::new(5.0, 0.0, 3.0, ((-0.5, 3.0), (0.5, -0.2)), 2.0);

        // Straight line
        let pred = sys.predict_pos(1000);
        assert_eq!(pred.0.0, 5.0);
        assert_eq!(pred.1, 0.0);

        sys.update_steering(-5.0);

        // Slight turn
        let pred = sys.predict_pos(1000);
        std::println!("{:?}", pred)
    }

    #[test]
    fn obb_created_correctly() {
        //TODO test this when phys working, OOB seems to be fine though, just the phys func is bad
        let mut sys = Aeb::<71>::new(5.0, 0.0, 3.0, ((-0.5, 3.0), (0.5, -0.2)), 2.0);

        let pred = sys.predict_pos(100);
        std::println!("{:?}", pred);
        let obb = sys.create_obb(pred.0, -1.3).unwrap();
        std::println!("{:?}", obb);

        sys.grid.draw_polygon(&obb);

        std::println!("{}", sys.grid)
    }
}
