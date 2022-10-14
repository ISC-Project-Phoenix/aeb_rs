use core::f32::consts::PI;
use core::ops::{Index, IndexMut};
use micromath::F32;

/// An nxn occupancy grid.
pub struct Grid<const N: usize> {
    data: [[Cell; N]; N],
}

impl<const N: usize> Grid<N> {
    /// Creates a new grid. This function will panic if N is even.
    pub const fn new() -> Self {
        // Ensure our N value is valid
        if N % 2 == 0 || N > f32::MAX as usize {
            panic!("Odd number used!");
        }

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
    /// Note that 0 degrees is still pointing to the right.
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
    use crate::grid::{Cell, Grid, GridPoint, GridSize, KartPoint};

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
}
