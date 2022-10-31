use aeb_rs::grid::Grid;
use aeb_rs::Aeb;
use std::thread::sleep;
use std::time::Duration;

/// Visualizes the forward kinematics used in collision detection. Visualized squares are exactly the same that will be collision checked in real AEB.
fn main() {
    // Please don't actually use AEB like this lol
    let mut aeb = Aeb::<51>::new(6.0, 0.0, 1.08, ((-0.5, 1.2), (0.5, -0.2)), 2.0);
    let mut grid = Grid::<51>::new();

    // Look over some steering angles
    for theta in 0..40 {
        aeb.update_steering(theta as f32);

        // Simulate 2 seconds into the future
        for t in (0..2000).step_by(20) {
            let pred = aeb.predict_pos(t);

            if let Ok(point) = pred.0.transform_to_grid(grid.get_size()) {
                grid.mark_occupied(point);
            }

            let obb = aeb.create_obb(pred.0, pred.1);
            grid.draw_polygon(&obb);

            println!("angle: {theta}, time: {}ms", t);
            println!("{}", grid);
            print!("\x1B[2J");
            grid.reset();

            sleep(Duration::from_millis(15))
        }
        grid.reset()
    }
}
