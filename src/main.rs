//! Assumptions:
//!
//! No error handling, unwrap any invariants. If I would find more time, then I prefer to do
//! something funny than just look for any possible error - in normal circumstances I would use
//! just `thiserror`/`anyhow`. After all - invalid input is assumed to either crash, or give
//! invalid output.
//!
//! Maze part is in `maze` module, conversion part is in `conv` module - those are basically two
//! separated applications.
//!
//! I also don't create tests - I assume application to be just showup "POC", and as before about
//! error handling - if I would find additional time, I would do something funny.

use std::io::{stdin, BufRead, BufReader};

mod maze;

/// Takes buffered raed and just parses the first line as it is just metadata (and is probably
/// irrelevant as lines are separated with `\n`, and assumption of reading until EOF should be good
/// enaugh). Reason to read X and Y is to verify invariants.
fn read_xy(input: &mut impl BufRead) -> (usize, usize) {
    let mut line = String::new();
    input.read_line(&mut line).unwrap();
    let mut splited = line.trim().split(',');

    let x = splited.next().unwrap().parse().unwrap();
    let y = splited.next().unwrap().parse().unwrap();

    (x, y)
}

fn main() {
    let mut input = BufReader::new(stdin());

    let (x, y) = read_xy(&mut input);

    maze::main(x, y, input, maze::flood);
}
