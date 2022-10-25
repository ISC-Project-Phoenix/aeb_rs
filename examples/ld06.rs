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
    let mut grid = Grid::<41>::new();
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
                if (scan.start_angle >= (360.0 - 25.0) && scan.start_angle <= 360.0)
                    || (scan.start_angle <= 25.0 && scan.start_angle >= 0.0)
                {
                    let points = scan.data.into_iter().enumerate().map(|(i, p)| {
                        if p.dist < 150 {
                            Err(GridErr::OutOfBounds)
                        } else {
                            let mut polar = scan.get_range_in_polar(i as u16);

                            KartPoint::from_polar(polar.0 / 1000.0, polar.1).transform_to_grid(size)
                        }
                    });

                    for p in points {
                        if let Ok(p) = p {
                            grid.mark_occupied(p);
                        }
                    }

                    println!("{}", grid);
                } else {
                    grid.reset()
                }
            }
            _ => {}
        }
    }
}
