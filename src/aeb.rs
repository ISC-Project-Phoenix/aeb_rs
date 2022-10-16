use crate::grid::{Grid, KartPoint};

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
    /// to the center of the rear axel.
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
    ///
    /// Returns true if the vehicle should brake.
    pub fn collision_check(&mut self, points: &[KartPoint]) -> bool {
        // Mark all detected obstacles as occupied on the grid
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

        //TODO determine step
        const STEP_MS: usize = 10;
        //Convert ttc to millis
        let ttc = (self.min_ttc / 1000.0) as usize;

        // Collision check by integrating over our model
        for timestep in (0..ttc).step_by(STEP_MS) {
            todo!("Continue once model is complete")
        }

        false
    }

    /// Updates the current velocity
    pub fn update_velocity(&mut self, velocity: f32) {
        self.velocity = velocity;
    }

    /// Updates the current ackermann steering angle, in degrees
    pub fn update_steering(&mut self, steering_angle: f32) {
        self.steering_angle = steering_angle;
    }
}
