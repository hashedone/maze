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
//! visualization, or kind of optimization for SSE/MMX - but I don't believe I would find
//! motivation for that).
//!
//! In terms of visualization (even printing to text) - I don't even try to be efficient.

use std::io::BufRead;

mod flood;
pub use flood::flood;

/// Single field in maze
#[derive(Clone, Copy)]
enum Field {
    Empty,
    Wall,
    /// Empty field with known distance from the start of the maze
    /// It doesn't need to be the closes path - it is distance calulated using some path
    Calculated(usize),
}

/// Whole maze reprezentation
pub struct Maze {
    /// All fields flattened
    maze: Box<[Field]>,
    /// Width of maze as it is needed for proper addressing
    w: usize,
}

impl Maze {
    /// Maps coord to field index
    fn idx(&self, x: usize, y: usize) -> usize {
        y * self.w + x
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

        Maze {
            maze,
            w: x,
        }
    }
}

#[cfg(feature = "text_visualize")]
impl std::fmt::Display for Maze {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        for line in self.maze.chunks(self.w) {
            let line: String = line
                .into_iter()
                .map(|field| match field {
                    Field::Empty => ' ',
                    Field::Wall => '#',
                    Field::Calculated(distance) => {
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
pub fn main(x: usize, y: usize, input: impl BufRead, calculator: impl Fn(Maze, usize, usize) -> Maze) {
    let mut maze = Maze::from_input(x, y, input);
    maze.maze[maze.idx(0, 1)] = Field::Calculated(0);

    #[cfg(feature = "text_visualize")]
    println!("Initial maze:\n\n{}\n", maze);

    let maze = calculator(maze, x - 1, y - 2);

    #[cfg(feature= "text_visualize")]
    println!("Calculated maze:\n\n{}\n", maze);
}
