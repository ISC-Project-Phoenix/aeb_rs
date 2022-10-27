use core::cmp::max;
use core::f32::consts::PI;
use core::fmt::{Debug, Display, Formatter};
use core::ops::{Index, IndexMut};
use line_drawing::Midpoint;
use micromath::F32;

/// An nxn occupancy grid.
#[derive(Debug)]
pub struct Grid<const N: usize> {
    data: [[Cell; N]; N],
}

impl<const N: usize> Grid<N> {
    // This is used to enforce N constraints at compile time
    const _SIZE: usize = if N % 2 == 0 {
        panic!("Even number used for gridsize!");
    } else if N > f32::MAX as usize {
        panic!("Gridsize is greater than max f32!");
    } else {
        N
    };

    /// Creates a new grid. This function will panic if N is even.
    pub const fn new() -> Self {
        // Force the compiler to not remove _SIZE
        let _ = Self::_SIZE;

        let arr = [[Cell::Unoccupied; N]; N];

        Self { data: arr }
    }

    /// Marks a cell as occupied.
    pub fn mark_occupied(&mut self, idx: GridPoint) {
        self[idx.into()] = Cell::Occupied
    }

    /// Checks if a cell is occupied.
    pub fn is_occupied(&self, idx: GridPoint) -> bool {
        self[idx.into()] == Cell::Occupied
    }

    /// Gets the size of one side of the grid, aka n.
    pub fn get_size(&self) -> GridSize {
        GridSize(N)
    }

    /// Resets the occupancy grid to a fully free state.
    pub fn reset(&mut self) {
        self.data = [[Cell::Unoccupied; N]; N];
    }

    /// Draws (Rasterizes) a filled polygon onto the grid. Lines must form a polygon, else this
    /// function will fail. This function will not fault if a point falls off the grid.
    ///
    /// Be aware that this function will use usize*N stack space.
    pub fn draw_polygon(&mut self, lines: &[Line]) {
        // The first and last point on a row
        #[derive(Default, Clone, Copy)]
        struct RowReg {
            start: usize,
            end: Option<usize>,
        }

        // Array of the first and last points on each line. Row is index, col is in the reg
        let mut ends: [Option<RowReg>; N] = [None; N];

        // Draw the lines, and record the ends of each row
        for p in lines
            .iter()
            .flat_map(|l| Midpoint::<f32, i64>::new(l.0, l.1))
        {
            // Handle when OOB is negative
            let x = max(p.0, 0) as usize;
            let y = max(p.1, 0) as usize;

            // Register ends
            if let Some(Some(reg)) = ends.get_mut(x).as_mut() {
                if reg.start == y {
                    continue;
                }

                // If point is left-more than start, it is the starting point
                if y < reg.start {
                    // Move old start to end if there is none (if some, then end will already be bigger)
                    if reg.end.is_none() {
                        reg.end = Some(reg.start)
                    }
                    reg.start = y
                }
                // If there was no end, this point must be the biggest. If there is, then replace if new point is bigger
                else if reg.end.is_none() || reg.end.unwrap() < y {
                    reg.end = Some(y)
                }
            } else if x < N {
                ends[x] = Option::from(RowReg {
                    start: y,
                    end: None,
                })
            }

            // Mark line square, if in bounds. Do this here as we still want to fill in the area via ends
            // even if this point is OOB
            if !(p.0 < 0 || p.0 >= N as i64 || p.1 < 0 || p.1 >= N as i64) {
                self.data[x][y] = Cell::Occupied;
            }
        }

        // Now fill in the inside of the polygon, bounded by the ends
        for (x, reg) in ends.iter().enumerate() {
            if let Some(reg) = reg {
                // Ignore rows with only one cell filled
                if let Some(end) = reg.end {
                    for y in reg.start..end {
                        // Bounds check
                        if !(x >= N || y >= N) {
                            self.data[x][y] = Cell::Occupied
                        }
                    }
                }
            }
        }
    }

    /// Checks if a filled polygon overlaps with any occupied space.
    ///
    /// - Lines must form a closed polygon, else this function will fail.
    ///
    /// - This function will not fault if a point falls off the grid.
    ///
    /// - Be aware that this function will use usize*N stack space.
    pub fn polygon_collide(&self, lines: &[Line]) -> bool {
        // The first and last point on a row
        #[derive(Default, Clone, Copy)]
        struct RowReg {
            start: usize,
            end: Option<usize>,
        }

        // Array of the first and last points on each line. Row is index, col is in the reg
        let mut ends: [Option<RowReg>; N] = [None; N];

        // Check the lines, and record the ends of each row
        for p in lines
            .iter()
            .flat_map(|l| Midpoint::<f32, i64>::new(l.0, l.1))
        {
            // Handle when OOB is negative
            let x = max(p.0, 0) as usize;
            let y = max(p.1, 0) as usize;

            // Register ends
            if let Some(Some(reg)) = ends.get_mut(x).as_mut() {
                if reg.start == y {
                    continue;
                }

                // If point is left-more than start, it is the starting point
                if y < reg.start {
                    // Move old start to end if there is none (if some, then end will already be bigger)
                    if reg.end.is_none() {
                        reg.end = Some(reg.start)
                    }
                    reg.start = y
                }
                // If there was no end, this point must be the biggest. If there is, then replace if new point is bigger
                else if reg.end.is_none() || reg.end.unwrap() < y {
                    reg.end = Some(y)
                }
            } else if x < N {
                ends[x] = Option::from(RowReg {
                    start: y,
                    end: None,
                })
            }

            // Bounds check, then check for collision on the line
            if !(p.0 < 0 || p.0 >= N as i64 || p.1 < 0 || p.1 >= N as i64)
                && self.data[x][y] == Cell::Occupied
            {
                return true;
            }
        }

        // Now check the inside of the polygon, bounded by the ends
        for (x, reg) in ends.iter().enumerate() {
            if let Some(reg) = reg {
                // Ignore rows with only one cell filled
                if let Some(end) = reg.end {
                    for y in reg.start..end {
                        // Bounds check, then check for collision
                        if !(x >= N || y >= N) && self.data[x][y] == Cell::Occupied {
                            return true;
                        }
                    }
                }
            }
        }

        false
    }
}

impl<const N: usize> Index<(usize, usize)> for Grid<N> {
    type Output = Cell;

    fn index(&self, index: (usize, usize)) -> &Self::Output {
        &self.data[index.0][index.1]
    }
}

impl<const N: usize> IndexMut<(usize, usize)> for Grid<N> {
    fn index_mut(&mut self, index: (usize, usize)) -> &mut Self::Output {
        &mut self.data[index.0][index.1]
    }
}

impl<const N: usize> Display for Grid<N> {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        let m = 10.0 / self.get_size().0 as f32;

        // Print numbers
        write!(f, "     ")?;
        for col_n in 0..self.data.len() {
            write!(f, "{:<4}", col_n)?;
        }
        writeln!(f)?;

        // Print data
        for (i, row) in self.data.into_iter().enumerate() {
            write!(f, "{:<4}[ ", i)?;
            for (j, col) in row.into_iter().enumerate() {
                match col {
                    Cell::Occupied => f.write_str("# ")?,
                    Cell::Unoccupied => f.write_str("  ")?,
                };

                if j + 1 != row.len() {
                    write!(f, "| ")?;
                }
            }
            writeln!(f, "]{:>4} {:>4.2}m", i, 10.0 - (m * i as f32))?;

            // Add separating lines
            write!(f, "    ")?;
            for _ in row {
                write!(f, "----")?;
            }
            writeln!(f)?;
        }
        Ok(())
    }
}

pub type Line = ((f32, f32), (f32, f32));

/// A cell in the grid
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub enum Cell {
    Occupied,
    Unoccupied,
}

/// Grid accosted errors
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum GridErr {
    /// Grid point was OOB.
    OutOfBounds,
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
pub struct GridPoint(usize, usize); // You should only get these by converting from KartPoints

impl GridPoint {
    pub fn raw(self) -> (usize, usize) {
        self.into()
    }
}

#[allow(clippy::from_over_into)]
impl Into<(usize, usize)> for GridPoint {
    fn into(self) -> (usize, usize) {
        (self.0, self.1)
    }
}

/// A point in gridspace, not aligned to the grid nor bounds checked.
#[derive(Copy, Clone, PartialEq, Debug, PartialOrd)]
pub struct GridPointf(f32, f32);

impl GridPointf {
    /// Transforms this point into a grid point, if in bounds.
    pub fn into_gridpoint(self, n: GridSize) -> Result<GridPoint, GridErr> {
        if self.0 < 0.0 || self.1 < 0.0 || self.0 >= n.raw() as f32 || self.1 >= n.raw() as f32 {
            Err(GridErr::OutOfBounds)
        } else {
            Ok(GridPoint(self.0 as usize, self.1 as usize))
        }
    }

    pub fn raw(self) -> (f32, f32) {
        self.into()
    }
}

#[allow(clippy::from_over_into)]
impl Into<(f32, f32)> for GridPointf {
    fn into(self) -> (f32, f32) {
        (self.0, self.1)
    }
}

impl KartPoint {
    /// Transforms a point from the frame of the kart to the frame of the grid, aligning to grid squares. This function will
    /// error if the kart point lands outside of the grid.
    pub fn transform_to_grid(&self, n: GridSize) -> Result<GridPoint, GridErr> {
        self.transform_to_grid_f(n).into_gridpoint(n)
    }

    /// Transforms a point from the frame of the kart to the frame of the grid. This function will not
    /// apply any bounds checking, use [Self::transform_to_grid] to bounds check and align points to grid.
    pub fn transform_to_grid_f(&self, n: GridSize) -> GridPointf {
        let GridSize(n) = n;

        // m is our grid scale (ie. units per grid square side)
        let m = 10.0 / (n as f32);

        let (r, c) = (self.0, self.1);

        // Matrix operation to rotate 180 then translate from the kart, which is at idx [n][(n-1)/2]
        let out_r = n as f32 - (r / m);
        let out_c = ((n as f32 - 1.) / 2.) + (c / m);

        GridPointf(out_r, out_c)
    }

    /// Creates a point in kart frame from a polar coordinate in kart frame.
    /// Note that 0 degrees is still pointing to the right.
    /// Theta is in degrees.
    pub fn from_polar(r: f32, theta: f32) -> Self {
        let theta = F32(theta.to_radians());

        // x and y are swapped from normal to reflect the karts axis
        let x = r * theta.sin();
        let y = r * theta.cos();

        KartPoint(x.0, y.0)
    }
}

#[allow(clippy::from_over_into)]
impl Into<(f32, f32)> for KartPoint {
    fn into(self) -> (f32, f32) {
        (self.0, self.1)
    }
}

#[cfg(test)]
mod test {
    use crate::grid::{Cell, Grid, GridPoint, GridSize, KartPoint};

    extern crate std;
    use std::prelude::rust_2021::*;

    #[test]
    fn transform_point_works() {
        let k = KartPoint(2.0, -3.0);
        assert_eq!(k.transform_to_grid(GridSize(5)).unwrap(), GridPoint(4, 0));

        let k = KartPoint(3.0, 1.0);
        assert_eq!(k.transform_to_grid(GridSize(5)).unwrap(), GridPoint(3, 2));

        let k = KartPoint(4.0, 0.0);
        assert_eq!(k.transform_to_grid(GridSize(5)).unwrap(), GridPoint(3, 2));

        let k = KartPoint(8.0, 5.5);
        assert_eq!(k.transform_to_grid(GridSize(5)).unwrap(), GridPoint(1, 4));

        //Test exact max bounds
        let k = KartPoint(10.0, 0.0);
        assert!(k.transform_to_grid(GridSize(5)).is_ok());

        //Test OOB
        let k = KartPoint(10.1, 0.0);
        assert!(k.transform_to_grid(GridSize(5)).is_err());
    }

    #[test]
    fn kartpoint_polar_works() {
        let stright = KartPoint::from_polar(10.0, 90.0);
        assert_eq!(stright, KartPoint(10.0, 0.0));

        let right = KartPoint::from_polar(10.0, 0.0);
        assert_eq!(right, KartPoint(0.0, 10.0));

        let left = KartPoint::from_polar(10.0, 180.0);
        assert_eq!(left, KartPoint(0.0, -10.0));

        //Just some point
        let point = KartPoint::from_polar(5.0, 53.13);
        assert_eq!((point.0.round(), point.1.round()), (4.0, 3.0));
    }

    #[test]
    fn grid_constructs() {
        let grid = Grid::<5>::new();

        assert_eq!(grid.get_size(), GridSize(5));
    }

    #[test]
    fn grid_resets() {
        let mut grid = Grid::<5>::new();

        grid[(0, 0)] = Cell::Occupied;
        assert_eq!(grid[(0, 0)], Cell::Occupied);

        grid.reset();
        assert_eq!(grid[(0, 0)], Cell::Unoccupied);
    }

    #[test]
    fn grid_indexes() {
        let mut grid = Grid::<5>::new();

        assert_eq!(grid[(0, 0)], Cell::Unoccupied);
        grid[(0, 0)] = Cell::Occupied;
        assert_eq!(grid[(0, 0)], Cell::Occupied);

        assert_eq!(grid[(4, 3)], Cell::Unoccupied);
        grid[(4, 3)] = Cell::Occupied;
        assert_eq!(grid[(4, 3)], Cell::Occupied);
    }

    #[test]
    fn insert_into_grid() {
        //Simulate the grid rutine
        let mut grid = Grid::<5>::new();
        let k = KartPoint(2.0, -3.0);
        let size = grid.get_size();

        let idx = k.transform_to_grid(size).unwrap();
        grid.mark_occupied(idx);
        assert!(grid.is_occupied(idx))
    }

    #[test]
    fn display_grid() {
        let mut grid = Grid::<41>::new();
        let k = KartPoint(2.0, -3.0);
        let size = grid.get_size();

        let idx = k.transform_to_grid(size).unwrap();
        grid.mark_occupied(idx);

        let point: (usize, usize) = idx.into();
        std::println!("point at: {:?}", point);
        std::println!("m={}m", 10.0 / 411.);
        std::println!("{}", grid)
    }

    #[test]
    fn polygon_methods_work() {
        let mut grid = Grid::<41>::new();

        // Ensure this does not fault despite being OOB
        grid.draw_polygon(&[
            ((1.0, 1.0), (10.0, 10.0)),
            ((1.0, 1.0), (1.0, 44.0)),
            ((1.0, 44.0), (10.0, 10.0)),
        ]);

        std::println!("{}", grid);

        let res = grid.polygon_collide(&[
            ((1.0, 1.0), (10.0, 10.0)),
            ((1.0, 1.0), (1.0, 14.0)),
            ((1.0, 14.0), (10.0, 10.0)),
        ]);

        assert!(res)
    }
}
