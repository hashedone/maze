//! I would like to approach the problem in two distinct ways
//!
//! One of them is floodfill - solution is highly suboptimal in terms of computational complexity,
//! but it parallelizes perfectly - every iteration step recalculates new maze path data basing
//! entirely on previous iteration. The aproach has a problem, that every iteration step is O(n)
//! itself, where n is entire maze size. However - the solution scales perfectly if we can have
//! separated thread for every field, which happens if we are on some kind of strong SIMD
//! architecture - like GPU. I see that in the offer there was a "FPGA" thing, and as we are
//! talking about financial calculation, I assume this is a reason of "FPGA" being there.
//!
//! The other approach is trying to have just nice solution for normal processors - just implement
//! properly aligned A* as pretty easy and common solutions for pathfinding. Nothing special there,
//! but on SISD arch it should behave pretty nicely (it could be probably improved by using some
//! more sophisticated algo like double ended A*, but I am lazy - to much work not showing too
//! much, if I would really find more time I would rather try to do something more interesting -
//! visualization, or kind of optimization - but I don't believe I would find motivation for that).
//!
//! I figured out additional "approach" (except taking completely different search algo). Maze
//! could be easly preprocessed to directed graph, where each cell (so actually non wall maze field)
//! has connection to the closest path crossing, and then running any pathfinding alg on that.
//! Benefit of that is that pathfinding itself is performed on strongly reduced graph, downside is
//! obviously need of preprocessing (not this much - possible to be done in O(x * y), but every
//! field have to be visited, while most reasonable finding algorithms avoids visiting every
//! field). The problem that if exit is not on the crossing then there is no incomming path to it
//! is actually not difficult to solve - simple raycast from exit can be done to find all fields
//! "connected" to exit (O(x + y)).
//!
//! In terms of visualization (even printing to text) - I don't even try to be efficient.

use std::cmp::Ordering;
use std::io::BufRead;

mod flood;
pub use flood::flood;

mod astar;
pub use astar::astar;

/// Direction from which its needed to approach the field to achieve it with given cost. As it is
/// possible to have same distance from multiple directions, it is a simple bitset. This is needed,
/// as in oru problem cost of next step is dependent on the fact if there is a turn on this step.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
struct Dir(u8);

impl Dir {
    pub const NONE: Dir = Dir(0);
    pub const LEFT: Dir = Dir(1);
    pub const UP: Dir = Dir(2);
    pub const RIGHT: Dir = Dir(4);
    pub const DOWN: Dir = Dir(8);
    pub const ANY: Dir = Dir(1 | 2 | 4 | 8);

    pub fn has_all(&self, Dir(other): Dir) -> bool {
        self.0 & other == other
    }

    /// Returns directions in which at least one step is needed
    pub fn vec((from_x, from_y): (usize, usize), (to_x, to_y): (usize, usize)) -> Self {
        let h = match from_x.cmp(&to_x) {
            Ordering::Less => Self::LEFT,
            Ordering::Greater => Self::RIGHT,
            Ordering::Equal => Self::NONE,
        };
        let v = match from_y.cmp(&to_y) {
            Ordering::Less => Self::UP,
            Ordering::Greater => Self::DOWN,
            Ordering::Equal => Self::NONE,
        };

        h | v
    }

    /// Rotates left
    pub fn left(mut self) -> Self {
        let down = (self.0 & 1) << 3;
        self.0 >>= 1;
        self.0 |= down;
        self
    }

    /// Rotates right
    pub fn right(mut self) -> Self {
        let left = (self.0 & 8) >> 3;
        self.0 <<= 1;
        self.0 |= left;
        self.0 &= 0xf;
        self
    }

    /// Returns minimal number of rotations so at least one encoded direction would match every
    /// given direction at least once
    pub fn min_rotation(self, other: Self) -> usize {
        // I have feeling it is strongly suboptimal; Actually as both directions are encoded as 4
        // bits, just precalculated table would be best solution
        let mut min = 4;
        for dir in [Self::LEFT, Self::RIGHT, Self::UP, Self::DOWN].iter() {
            let mut d = *dir;

            if !self.has_all(d) {
                continue;
            }

            let mut o = other.0 & !dir.0;
            let mut cnt = 0;
            while o != 0 {
                cnt += 1;
                d = d.left();
                o &= !d.0;
            }
            min = std::cmp::min(min, cnt);

            d = *dir;
            o = other.0 & !dir.0;
            cnt = 0;
            while o != 0 {
                cnt += 1;
                d = d.right();
                o &= !d.0;
            }
            min = std::cmp::min(min, cnt);
        }

        min
    }
}

impl std::ops::BitOr for Dir {
    type Output = Self;

    fn bitor(self, rhs: Self) -> Self {
        Self(self.0 | rhs.0)
    }
}

/// Single field in maze
#[derive(Clone, Copy, Debug)]
enum Field {
    Empty,
    Wall,
    /// Empty field with known distance from the start of the maze
    /// It doesn't need to be the closes path - it is distance calulated using some path
    Calculated(Dir, usize),
}

/// Whole maze reprezentation
pub struct Maze {
    /// All fields flattened
    maze: Box<[Field]>,
    /// Width of maze as it is needed for proper addressing (inlcuding external wall)
    w: usize,
}

impl Maze {
    /// Maps coord to field index
    fn idx(&self, x: usize, y: usize) -> usize {
        // On overflow just give invalid (too big) index - anything from here would be wall by
        // default which is simplification on purpose
        y.saturating_mul(self.w).saturating_add(x)
    }

    /// Maps field index to coordinates
    fn coords(&self, idx: usize) -> (usize, usize) {
        (idx % self.w, idx / self.w)
    }

    /// Returns index of field in given direction (defined to be wrapping)
    fn in_dir_idx(&self, idx: usize, dir: Dir) -> usize {
        let (x, y) = self.coords(idx);
        // Doing wrapping sub basically because maze size is way smaller than my indexing type size
        // (considering >= 16bit machine), so after wrapping I would have invalid field, so Wall by
        // default
        let (x, y) = match dir {
            Dir::UP => (x, y.wrapping_sub(1)),
            Dir::DOWN => (x, y + 1),
            Dir::LEFT => (x.wrapping_sub(1), y),
            Dir::RIGHT => (x + 1, y),
            _ => (x, y),
        };

        self.idx(x, y)
    }

    /// Returns field in given direction from given one (Wall if no such field)
    /// If Dir has more than one direction encoded, field with same idx is returned
    fn in_dir(&self, idx: usize, dir: Dir) -> Field {
        self.maze
            .get(self.in_dir_idx(idx, dir))
            .copied()
            .unwrap_or(Field::Wall)
    }

    /// Gives field from given coord (Wall if no such field)
    fn field(&self, x: usize, y: usize) -> Field {
        self.maze
            .get(self.idx(x, y))
            .copied()
            .unwrap_or(Field::Wall)
    }

    /// Gives mutable field from given coord
    fn field_mut(&mut self, x: usize, y: usize) -> Option<&mut Field> {
        self.maze.get_mut(self.idx(x, y))
    }

    /// Creates valid maze from input containing maze description, and x/y dimentions of it
    pub fn from_input(x: usize, y: usize, input: impl BufRead) -> Self {
        // Iterating over bytes is bad idea, but only interesting charactes are 0 and 1 which
        // happens to be ASCII bytes. I am aware it wont work with any non-ASCII UTF representation
        // of 0 and 1 and "I don't care, what they're going to say..."
        let maze = input
            .lines()
            .take(y)
            .flat_map(|line| line.unwrap().into_bytes())
            .map(|field| match field {
                b'0' => Field::Wall,
                b'1' => Field::Empty,
                _ => panic!("Invalid input"),
            })
            .collect();

        Maze { maze, w: x }
    }
}

#[cfg(feature = "text_visualize")]
impl std::fmt::Display for Maze {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        // While printing maze, externall wall is not printed
        for line in self.maze.chunks(self.w) {
            let line: String = line
                .iter()
                .map(|field| match field {
                    Field::Empty => ' ',
                    Field::Wall => '#',
                    Field::Calculated(_, distance) => {
                        (distance % 10).to_string().chars().last().unwrap()
                    }
                })
                .chain(std::iter::once('\n'))
                .collect();

            f.write_str(&line)?;
        }

        Ok(())
    }
}

/// As both "parts" of excercise are actually two separated applications, here we have maze "main"
/// (with preparsed arguments).
///
/// The last argument is function for caluclating the shortest path.
/// As an argument it takes initial maze, with at least one field with known distance - which is
/// considered to be an "initial cost" of entering into the maze with this input, and additionally
/// a field where we algorithm is looking path to. Returned maze contains exit field calculated to
/// the closest path, and some another field calculated to have "at least this good" path.
///
/// If there is no path to given exit, calculator should return maze with not calculated exit field
pub fn main(
    x: usize,
    y: usize,
    input: impl BufRead,
    calculator: impl Fn(Maze, usize, usize) -> Maze,
) {
    let mut maze = Maze::from_input(x, y, input);
    *maze.field_mut(0, 1).unwrap() = Field::Calculated(Dir::ANY, 0);

    #[cfg(feature = "text_visualize")]
    println!("Initial maze:\n\n{}\n", maze);

    let maze = calculator(maze, x - 1, y - 2);

    #[cfg(feature = "text_visualize")]
    println!("Calculated maze:\n\n{}\n", maze);

    match maze.field(x - 1, y - 2) {
        Field::Empty => println!("UNREACHABLE"),
        Field::Wall => println!("INVALID"),
        Field::Calculated(_, cost) => println!("{}", cost),
    }
}
