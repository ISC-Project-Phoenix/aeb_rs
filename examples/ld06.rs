use ld06_embed::error::ParseError;
use ld06_embed::LD06;
use linux_embedded_hal::nb::Error;
use linux_embedded_hal::serial_core::{
    BaudOther, CharSize, FlowControl, ParityNone, PortSettings, SerialPort, StopBits,
};
use linux_embedded_hal::Serial;
use std::path::Path;

use aeb_rs::grid::*;

/// Reads points from an LD06 LiDAR into an occupancy grid.
fn main() {
    let port = Path::new("/dev/ttyUSB0");
    let mut serial = Serial::open(port).unwrap();
    let conf = PortSettings {
        baud_rate: BaudOther(230_400),
        char_size: CharSize::Bits8,
        parity: ParityNone,
        stop_bits: StopBits::Stop1,
        flow_control: FlowControl::FlowNone,
    };
    serial.0.configure(&conf).unwrap();

    let mut ld06 = LD06::new(serial);

    // Create grid
    let mut grid = Grid::<31>::new();
    let size = grid.get_size();

    loop {
        match ld06.read_next_byte() {
            Err(err) => match err {
                Error::Other(parse_err) => match parse_err {
                    ParseError::SerialErr => {
                        println!("Serial issue")
                    }
                    ParseError::CrcFail => {
                        println!("CRC failed")
                    }
                },
                Error::WouldBlock => {
                    println!("Would block")
                }
            },
            Ok(Some(scan)) => {
                // Filter for scans in the +-25 degrees cone
                if (360.0 - 25.0..=360.0).contains(&scan.start_angle)
                    || (..=25.0).contains(&scan.start_angle)
                    || (360.0 - 25.0..=360.0).contains(&scan.end_angle)
                    || (..=25.0).contains(&scan.end_angle)
                {
                    for (i, p) in scan.data.into_iter().enumerate() {
                        // Filter out close or unlikely points
                        if p.dist < 150 || p.confidence < 50 {
                            continue;
                        }

                        // Convert LiDAR points into standard polar points
                        let polar = scan.get_range_in_polar(i as u16);
                        // Transform from polar to cartiesian
                        let kart = KartPoint::from_polar(polar.0 / 1000.0, polar.1);
                        // Apply linear transform to grid
                        if let Ok(transformed) = kart.transform_to_grid(size) {
                            grid.mark_occupied(transformed)
                        }
                    }

                    println!("{}", grid);
                    //Clear terminal
                    print!("\x1B[2J");
                } else {
                    grid.reset()
                }
            }
            _ => {}
        }
    }
}
