//! This crate is an implementation of the automatic emergency braking algorithm for [Project Phoenix](https://github.com/ISC-Project-Phoenix).
//!
//!
//! This crate is no_std, and designed to be run on embedded devices.
//!
//! For algorithm and system integration details, please see the [AEB design document](https://github.com/ISC-Project-Phoenix/design/blob/main/software/AEB.md).
//!
//! # Example
//!
//! ```rust
//! # use aeb_rs::Aeb;
//! # use aeb_rs::grid::KartPoint;
//!
//! // Configure AEB struct with values from Phoenix
//! let mut aeb = Aeb::<81>::new(
//!         3.0,
//!         0.0,
//!         1.08,
//!         ((-0.675, 1.43), (0.675, -0.59)),
//!         KartPoint(1.43, 0.0),
//!         3.0,
//!     );
//!
//! // Update parameters
//! aeb.update_velocity(5.0);
//! aeb.update_steering(0.0);
//! aeb.update_ttc(2.0);
//!
//! // Add some (very fake) obstacle readings
//! let points = [KartPoint(1.0, 0.0); 12];
//! aeb.add_points(&points);
//!
//! // Collision detect!
//! let (should_stop, collides_at) = aeb.collision_check(None);
//! // let should_stop = aeb.collision_check(&points); alternatively
//!
//! println!("Collides at: {}", collides_at);
//! assert!(should_stop);
//! assert_eq!(collides_at, 210);
//! ```

#![no_std]

mod aeb;
pub mod grid;

pub use aeb::Aeb;
