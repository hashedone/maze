//! "Bignum" bin -> dec implementation
//!
//! As algorithm for this conversion is trivial, I just use existing implementation - `ramp` seems
//! pretty fast. I don't think I am able to implement anything better for such conversion in
//! reasonable time.

use num_bigint::BigUint;
use std::io::BufRead;

pub fn main(cnt: usize, input: impl BufRead) {
    for line in input.lines().take(cnt) {
        println!(
            "{}",
            BigUint::parse_bytes(line.unwrap().as_bytes(), 2).unwrap()
        );
    }
}
