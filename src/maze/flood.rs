use super::{Dir, Field, Maze};
use rayon::prelude::*;
use std::convert::identity as ident;

/// Calculates new cost of single field with directions from which the best value is achievable.
///
/// input - previous iteration output
/// idx - index of calculated field
fn update_field(input: &Maze, idx: usize) -> (Dir, usize) {
    let dirs = [Dir::UP, Dir::DOWN, Dir::LEFT, Dir::RIGHT];
    let mut best = (Dir::NONE, 0);

    if matches!(input.maze[idx], Field::Wall) {
        return best;
    }

    for dir in dirs.iter() {
        let updated = match input.in_dir(idx, *dir) {
            Field::Wall | Field::Empty => None,
            Field::Calculated(pdir, cost) => Some(cost + (!pdir.has_all(*dir) as usize)),
        };

        best = match (best, updated) {
            // Previously there was no best direction, just overwrite
            ((Dir::NONE, _), Some(ncost)) => (*dir, ncost),
            // We have same cost as from previously approached direction, combine directions
            ((bdir, bcost), Some(ncost)) if bcost == ncost => (bdir | *dir, bcost),
            // We have better cost than previously, overwrite
            ((_, bcost), Some(ncost)) if ncost < bcost => (*dir, ncost),
            // No cost calculated from this direction, or cost is worse - leave best as it was
            (best, _) => best,
        };
    }

    best
}

/// Single iteration of loop, basically frame update. Normally done by GPU rasterizer, here for
/// some profit I would just use Rayon.
///
/// input - previous iteration output (front buffer)
/// output - next iteration output (back buffer)
/// updates - additional buffer marking which fields changed to better value; If there was no
/// changes at all, it means that nothing more can be done for finding better way (if there is no
/// path to exit yet, there is none at all). Additionally if the value on the current cost to exit
/// is lower or equal to any updated field, it is also impossible to find better solution, so
/// iteration can be finished earlier.
///
/// As it would be more "idiomatic" or "functional", to return the new maze as result, I have in
/// mind this is optimized for SIMD, so I am really into doing all the calculations inplace on
/// existing buffers (instead of allocating backbuffer every frame).
fn iteration(input: &Maze, output: &mut [Field], updates: &mut [Option<usize>]) {
    let output = output.par_iter_mut();
    let updates = updates.par_iter_mut();

    // After all I figured out, that the whole calculation could be splitted even more - instead of
    // "per field", it could be "per field per direction" and after all merged - adding to that
    // filtering walls outside of "kernel" it would remove internal loop and reduce need of
    // branching in kernel (however branching would be moved to "merging" phase - I leave it as
    // possible improvement, but I would not spend more time on that.
    output
        .zip(updates)
        .enumerate()
        .for_each(|(idx, (output, update))| {
            let updated = update_field(input, idx);

            let (updated, change) = match (updated, input.maze[idx]) {
                ((Dir::NONE, _), field) => {
                    // Field has no update from any direction
                    (field, None)
                }
                ((udir, ucost), Field::Calculated(pdir, pcost)) if ucost == pcost => {
                    // Field is considered as updated only, if additional directions with this cost
                    // are added
                    if !pdir.has_all(udir) {
                        (Field::Calculated(udir | pdir, ucost), Some(ucost))
                    } else {
                        (Field::Calculated(udir, ucost), None)
                    }
                }
                ((udir, ucost), Field::Calculated(_, pcost)) if ucost < pcost => {
                    // Better path is found to this field. Even if the path is from another
                    // direction, path going out of this field cannot be worse that they were to be
                    // - the cost from this point can increase at max 1 (additional turn), but
                    // after all, cost is decreased for at least 1
                    (Field::Calculated(udir, ucost), Some(ucost))
                }
                ((dir, cost), Field::Empty) => {
                    // Calculated cost for previously not calculated field - it is best for now
                    (Field::Calculated(dir, cost), Some(cost))
                }
                (_, field) => {
                    // Updated value is not better than previous in any way, so nothing should be
                    // updated
                    (field, None)
                }
            };

            *output = updated;
            *update = change;
        })
}

/// Predicate calulating, if algorithm should stop. It happens in two cases:
/// 1. There was no updates on last iteration
/// 2. All changes in last iteration updated their fields to cost higher or equal that current exit cost
fn is_done(exit: Field, updates: &[Option<usize>]) -> bool {
    let best = updates.par_iter().copied().filter_map(ident).min();
    match (best, exit) {
        (None, _) => true,
        (Some(best), Field::Calculated(_, exit)) if best >= exit => true,
        _ => false,
    }
}

/// Implementation of flood search algorithm
///
/// As an argument it takes initial maze, with at least one field with known distance - which is
/// considered to be an "initial cost" of entering into the maze with this input, and additionally
/// a field where we algorithm is looking path to. Returned maze contains exit field calculated to
/// the closest path, and some another field calculated to have "at least this good" path.
pub fn flood(mut maze: Maze, x: usize, y: usize) -> Maze {
    let mut backbuffer = vec![Field::Wall; maze.maze.len()].into_boxed_slice();

    // Updates is initialized to anything which is not fully `None` - this is to ensure, that the
    // iteration would not end before it starts.
    let mut updates = vec![Some(0); maze.maze.len()].into_boxed_slice();
    while !is_done(maze.field(x, y), &updates) {
        iteration(&maze, &mut backbuffer, &mut updates);
        std::mem::swap(&mut maze.maze, &mut backbuffer);
        #[cfg(feature = "text_visualize")]
        println!("Next iteration:\n\n{}", maze);
    }

    maze
}
