use aeb_rs::Aeb;
use ld06_embed::error::ParseError;
use ld06_embed::LD06;
use linux_embedded_hal::nb::Error;
use linux_embedded_hal::serial_core::{
    BaudOther, CharSize, FlowControl, ParityNone, PortSettings, SerialPort, StopBits,
};
use linux_embedded_hal::Serial;
use std::path::Path;

use aeb_rs::grid::*;

/// Runs AEB using an LD06 Lidar.
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

    let mut aeb = Aeb::<411>::new(
        1.0,
        0.0,
        1.08,
        ((-0.675, 1.43), (0.675, -0.59)),
        KartPoint(1.43, 0.0),
        5.0,
    );
    aeb.update_timestep(20);

    let mut last_in_range = false;

    println!("Initialised, beginning loop");

    loop {
        match ld06.read_next_byte() {
            Err(err) => match err {
                Error::Other(parse_err) => match parse_err {
                    ParseError::SerialErr(_) => {
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
                    last_in_range = true;

                    for (i, p) in scan.data.into_iter().enumerate() {
                        // Filter out close or unlikely points
                        if p.dist < 150 || p.confidence < 150 {
                            continue;
                        }

                        // Convert LiDAR points into standard polar points
                        let polar = scan.get_range_in_polar(i as u16);
                        // Transform from polar to cartiesian
                        let kart = KartPoint::from_polar(polar.0 / 1000.0, polar.1);

                        aeb.add_points(&[kart]);
                    }
                } else if last_in_range {
                    last_in_range = false;

                    let (collides, time) = aeb.collision_check(None);

                    if collides {
                        println!(
                            "Collision at {}m!",
                            (time as f32 / 1000.0) / aeb.get_velocity()
                        )
                    }
                }
            }
            _ => {}
        }
    }
}
