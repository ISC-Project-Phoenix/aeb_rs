use aeb_rs::grid::Grid;
use nalgebra::{max, Point2, Translation2};
use std::thread::sleep;
use std::time::{Duration, Instant};

/// Demos polygon drawing on the occupancy grid by bouncing a box around.
fn main() {
    let mut grid = Grid::<41>::new();
    let n = grid.get_size().raw() as f32;

    // Our polygon to rotate
    let mut lines = vec![
        (
            Point2::from_slice(&[10.0f32, 5.0]),
            Point2::from_slice(&[20.0, 5.0]),
        ),
        (
            Point2::from_slice(&[20.0, 5.0]),
            Point2::from_slice(&[20.0, 10.0]),
        ),
        (
            Point2::from_slice(&[20.0, 10.0]),
            Point2::from_slice(&[10.0, 10.0]),
        ),
        (
            Point2::from_slice(&[10.0, 10.0]),
            Point2::from_slice(&[10.0, 5.0]),
        ),
    ];
    let mut trans = Translation2::new(0.5, 0.5);

    let start_tim = Instant::now();
    let mut frames = 0u128;

    loop {
        let lines_us: Vec<((f32, f32), (f32, f32))> = lines
            .iter()
            .map(|l| ((l.0.x, l.0.y), (l.1.x, l.1.y)))
            .collect();

        grid.draw_polygon(&lines_us);

        // Print FPS info
        frames += 1;
        println!(
            "FPS: {}",
            frames / max((Instant::now() - start_tim).as_secs() as u128, 1)
        );

        // Draw grid
        println!("{}", grid);
        print!("{esc}c", esc = 27 as char);
        grid.reset();

        // Bounce box off walls
        for l in lines.iter_mut() {
            if l.0.x > n || l.0.x <= 0.0 || l.1.x > n || l.1.x <= 0.0 {
                trans = Translation2::new(trans.x * -1.0, trans.y);
            }
            if l.0.y > n || l.0.y <= 0.0 || l.1.y > n || l.1.y <= 0.0 {
                trans = Translation2::new(trans.x, trans.y * -1.0);
            }
        }

        // Update translation for next frame
        for l in lines.iter_mut() {
            *l = ((trans * l.0), (trans * l.1));
        }

        // DIY V-Sync lol
        sleep(Duration::from_millis(18))
    }
}
