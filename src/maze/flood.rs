use super::{Maze, Field};

/// Implementation of flood search algorithm
///
/// As an argument it takes initial maze, with at least one field with known distance - which is
/// considered to be an "initial cost" of entering into the maze with this input, and additionally
/// a field where we algorithm is looking path to. Returned maze contains exit field calculated to
/// the closest path, and some another field calculated to have "at least this good" path.
pub fn flood(mut maze: Maze, x: usize, y: usize) -> Maze {
    let mut backbuffer = vec![Field::Empty; maze.maze.len()].into_boxed_slice();
    let mut updates: Box<[Option<usize>]> = vec![None; maze.maze.len()].into_boxed_slice();
    maze
}
