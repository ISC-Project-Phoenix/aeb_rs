use crate::grid::{Cell, Grid, KartPoint};

/// The entrypoint to the automatic emergency braking algorithm.
pub struct Aeb<const GridN: usize> {
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

            let grid_idx = p.into();

            if self.grid[grid_idx] != Cell::Occupied {
                self.grid[grid_idx] = Cell::Occupied
            }
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
