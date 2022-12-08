use aeb_rs::grid::{Grid, KartPoint};
use aeb_rs::Aeb;
use std::thread::sleep;
use std::time::Duration;

/// Visualizes the forward kinematics used in collision detection. Visualized squares are exactly the same that will be collision checked in real AEB.
fn main() {
    //V len = 202cm
    //H width = 135cm
    //Axel to Lidar = 143cm baby lidar

    // Please don't actually use AEB like this lol
    let mut aeb = Aeb::<71>::new(
        3.0,
        0.0,
        1.08,
        ((-0.675, 1.43), (0.675, -0.59)),
        KartPoint(1.43, 0.0),
        3.0,
    );
    let mut grid = Grid::<71>::new();

    // Look over some steering angles
    for theta in 0..29 {
        aeb.update_steering(theta as f32);

        // Simulate 2 seconds into the future
        for t in (0..3000).step_by(20) {
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
