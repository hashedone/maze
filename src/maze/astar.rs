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

struct AStar {
    maze: Maze,
    queue: BinaryHeap<QueueItem>,
    exit: (usize, usize),
}

impl AStar {
    fn new(maze: Maze, x: usize, y: usize) -> Self {
        let queue: BinaryHeap<_> = maze
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

        Self {
            queue,
            maze,
            exit: (x, y),
        }
    }

    fn enqueue(&mut self, idx: usize) {
        if let Field::Calculated(dir, cost) = self.maze.maze[idx] {
            let v = Dir::vec(self.maze.coords(idx), self.exit);
            let rotations = dir.min_rotation(v);
            self.queue.push(QueueItem {
                cost: cost + rotations,
                idx,
            })
        }
    }

    fn run(mut self) -> Maze {
        let dirs = [
            (Dir::LEFT, Dir::RIGHT),
            (Dir::RIGHT, Dir::LEFT),
            (Dir::UP, Dir::DOWN),
            (Dir::DOWN, Dir::UP),
        ];

        while let Some(QueueItem { idx, .. }) = self.queue.pop() {
            let field = self.maze.maze[idx];

            for (from, to) in dirs.iter() {
                let cost = match field {
                    Field::Calculated(dir, cost) => cost + (!dir.has_all(*from) as usize),
                    _ => continue,
                };

                let next_idx = self.maze.in_dir_idx(idx, *to);
                match self.maze.maze.get(next_idx).copied().unwrap_or(Field::Wall) {
                    Field::Calculated(dir, pcost) if pcost == cost => {
                        self.maze.maze[next_idx] = Field::Calculated(dir | *from, cost);
                        self.enqueue(next_idx);
                    }
                    Field::Calculated(_, pcost) if cost < pcost => {
                        self.maze.maze[next_idx] = Field::Calculated(*from, cost);
                        self.enqueue(next_idx);
                    }
                    Field::Empty => {
                        self.maze.maze[next_idx] = Field::Calculated(*from, cost);
                        self.enqueue(next_idx);
                    }
                    _ => (),
                }
            }

            #[cfg(feature = "text_visualize")]
            println!("Next iteration:\n\n{}", self.maze);

            let (x, y) = self.exit;
            if let Field::Calculated(_, _) = self.maze.field(x, y) {
                break;
            }
        }

        self.maze
    }
}

/// Implementation of A* search algorithm
///
/// As an argument it takes initial maze, with at least one field with known distance - which is
/// considered to be an "initial cost" of entering into the maze with this input, and additionally
/// a field where we algorithm is looking path to. Returned maze contains exit field calculated to
/// the closest path, and some another field calculated to have "at least this good" path.
pub fn astar(maze: Maze, x: usize, y: usize) -> Maze {
    AStar::new(maze, x, y).run()
}
