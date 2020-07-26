use super::{Dir, Field, Maze};
use std::collections::BinaryHeap;

/// Item to be stored on priority queue (aka binary heap) to find the best candidate for closest
/// path
#[derive(PartialEq, Eq, Debug)]
struct QueueItem {
    /// Cost from the beginning (+ minimum additional turns as heuristic)
    cost: usize,
    /// Field index
    idx: usize,
}

impl std::cmp::PartialOrd for QueueItem {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl std::cmp::Ord for QueueItem {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        // Ordering on cost is reversed, as it is designed to be used in max-heap to look for min candidate
        let left = (std::cmp::Reverse(self.cost), self.idx);
        let right = (std::cmp::Reverse(other.cost), other.idx);
        left.cmp(&right)
    }
}

/// Implementation of A* search algorithm
///
/// As an argument it takes initial maze, with at least one field with known distance - which is
/// considered to be an "initial cost" of entering into the maze with this input, and additionally
/// a field where we algorithm is looking path to. Returned maze contains exit field calculated to
/// the closest path, and some another field calculated to have "at least this good" path.
pub fn astar(mut maze: Maze, x: usize, y: usize) -> Maze {
    let dirs = [
        (Dir::LEFT, Dir::RIGHT),
        (Dir::RIGHT, Dir::LEFT),
        (Dir::UP, Dir::DOWN),
        (Dir::DOWN, Dir::UP),
    ];

    let mut queue: BinaryHeap<_> = maze
        .maze
        .iter()
        .enumerate()
        .filter_map(|(idx, field)| match field {
            Field::Calculated(dir, cost) => {
                let v = Dir::vec((x, y), maze.coords(idx));
                let rotations = dir.min_rotation(v);
                Some(QueueItem {
                    cost: *cost + rotations,
                    idx,
                })
            }
            _ => None,
        })
        .collect();

    while let Some(QueueItem { idx, cost, .. }) = queue.pop() {
        let field = maze.maze[idx];
        #[cfg(feature = "text_visualize")]
        println!(
            "Expanding field {:?} ({:?}), expected cost: {}",
            maze.coords(idx),
            field,
            cost
        );

        for (from, to) in dirs.iter() {
            let cost = match field {
                Field::Calculated(dir, cost) => cost + (!dir.has_all(*from) as usize),
                _ => continue,
            };

            let next_idx = maze.in_dir_idx(idx, *to);
            match maze.maze.get(next_idx).copied().unwrap_or(Field::Wall) {
                Field::Calculated(dir, pcost) if pcost == cost => {
                    maze.maze[next_idx] = Field::Calculated(dir | *from, cost);
                    let v = Dir::vec(maze.coords(next_idx), (x, y));
                    let rotations = (dir | *from).min_rotation(v);
                    queue.push(QueueItem {
                        cost: cost + rotations,
                        idx: next_idx,
                    })
                }
                Field::Calculated(_, pcost) if cost < pcost => {
                    maze.maze[next_idx] = Field::Calculated(*from, cost);
                    let v = Dir::vec(maze.coords(next_idx), (x, y));
                    let rotations = from.min_rotation(v);
                    queue.push(QueueItem {
                        cost: cost + rotations,
                        idx: next_idx,
                    })
                }
                Field::Empty => {
                    maze.maze[next_idx] = Field::Calculated(*from, cost);
                    let v = Dir::vec(maze.coords(next_idx), (x, y));
                    let rotations = from.min_rotation(v);
                    queue.push(QueueItem {
                        cost: cost + rotations,
                        idx: next_idx,
                    })
                }
                _ => (),
            }
        }

        #[cfg(feature = "text_visualize")]
        println!("Next iteration:\n\n{}", maze);

        if let Field::Calculated(_, _) = maze.field(x, y) {
            break;
        }
    }

    maze
}
