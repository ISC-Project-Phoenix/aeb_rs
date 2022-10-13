//TODO make grid

use core::f32::consts::PI;
use micromath::F32;

/// Grid accosted errors
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum GridErr {
    /// Grid point was OOB.
    OutOfBounds,
    /// The grid size was too large for a f32.
    GreaterThanF32,
    /// The grid size was not odd.
    NotOdd,
}

/// The n in nxn grid. This is garrentied to be odd and < f32::MAX.
#[derive(Copy, Clone, PartialEq, Eq, Debug, PartialOrd, Ord)]
pub struct GridSize(usize); //only produced by grid, which checks the above in its constructor

impl GridSize {
    /// Gets the raw n value of the grid.
    pub fn raw(&self) -> usize {
        self.0
    }
}

/// A point in the reference frame of the kart.
///
/// This is in (x,y), where x is parallel to the kart and positive y is perpendicular to the right of the kart.
#[derive(Copy, Clone, PartialEq, Debug, PartialOrd)]
pub struct KartPoint(pub f32, pub f32);

/// An index indo the grid, garrentied to be in bounds. We define a point on the grid to be the top left corner of the grid squares
/// if a grid is drawn out. The top left of the grid is (0,0).
#[derive(Copy, Clone, PartialEq, Eq, Debug, PartialOrd, Ord)]
pub struct GridPoint(usize, usize);

#[allow(clippy::from_over_into)]
impl Into<(usize, usize)> for GridPoint {
    fn into(self) -> (usize, usize) {
        (self.0, self.1)
    }
}

impl KartPoint {
    /// Transforms a point from the frame of the kart to the frame of the grid. This function will
    /// error if the kart point lands outside of the grid.
    pub fn transform_to_grid(&self, n: GridSize) -> Result<GridPoint, GridErr> {
        let GridSize(n) = n;

        // m is our grid scale (ie. units per grid square side)
        let m = 10.0 / (n as f32);

        let (r, c) = (self.0, self.1);

        // Matrix operation to rotate 180 then translate from the kart, which is at idx [n][(n-1)/2]
        let out_r = n as f32 - (r / m);
        let out_c = ((n as f32 - 1.) / 2.) + (c / m);

        // Point is too far off the grid forward or left
        if out_r < 0.0 || out_c < 0.0 {
            return Err(GridErr::OutOfBounds);
        }

        // Round to nearest grid square to allow for use as an index.
        let out_r = out_r as usize;
        let out_c = out_c as usize;

        // Bounds check backwards and right
        if out_c < n && out_r < n {
            Ok(GridPoint(out_r, out_c))
        } else {
            Err(GridErr::OutOfBounds)
        }
    }

    /// Creates a point in kart frame from a polar coordinate in kart frame.
    ///
    /// Theta is in degrees.
    pub fn from_polar(r: f32, theta: f32) -> Self {
        let theta = (F32::from(theta) * PI) / 180.0;

        // x and y are swapped from normal to reflect the karts axis
        let x = r * theta.sin();
        let y = r * theta.cos();

        KartPoint(x.0, y.0)
    }
}

#[cfg(test)]
mod test {
    use crate::grid::{GridPoint, GridSize, KartPoint};

    #[test]
    fn transform_point_works() {
        let k = KartPoint(2.0, -3.0);
        assert_eq!(k.transform_to_grid(GridSize(5)).unwrap(), GridPoint(4, 0));

        let k = KartPoint(3.0, 1.0);
        assert_eq!(k.transform_to_grid(GridSize(5)).unwrap(), GridPoint(3, 2));

        let k = KartPoint(4.0, 0.0);
        assert_eq!(k.transform_to_grid(GridSize(5)).unwrap(), GridPoint(3, 2));

        //Test exact max bounds
        let k = KartPoint(10.0, 0.0);
        assert!(k.transform_to_grid(GridSize(5)).is_ok());

        //Test OOB
        let k = KartPoint(10.1, 0.0);
        assert!(k.transform_to_grid(GridSize(5)).is_err());
    }

    #[test]
    fn polar_works() {
        let stright = KartPoint::from_polar(10.0, 90.0);
        assert_eq!(stright, KartPoint(10.0, 0.0));

        let right = KartPoint::from_polar(10.0, 0.0);
        assert_eq!(right, KartPoint(0.0, 10.0));

        let left = KartPoint::from_polar(10.0, 180.0);
        assert_eq!(left, KartPoint(0.0, -10.0));

        let point = KartPoint::from_polar(5.0, 53.13);
        assert_eq!((point.0.round(), point.1.round()), (4.0, 3.0));
    }
}
