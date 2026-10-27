# Contributing

Thanks for helping! This project is MIT licensed; by contributing you agree your work is released under the same license.

## Development

```bash
cargo test --workspace --all-features
cargo clippy --workspace --all-targets --all-features -- -D warnings
cargo fmt --all
cargo bench -p rebirth-ik --features rayon       # Criterion benchmarks
```

Keep build artifacts out of the repository tree if disk space is tight: `export CARGO_TARGET_DIR=/some/other/disk`.

## Good first contributions

* **A fair benchmark harness against the original C library** ([TheComet/ik](https://github.com/TheComet/ik)). Notes: the original runs its full iteration budget without an early exit and defaults to `double`; build it with `-DIK_PRECISION=float` for an `f32` comparison and compare against `Config { early_exit: false, .. }` so both sides do the same work. Please verify the original's output is actually converged before quoting numbers.
* More constraint types (twist limits, ellipsoidal cones).
* A structure-of-arrays batch solver.
* Bindings: C FFI, `pyo3`, `napi-rs`.
* Bug reports with a minimal skeleton that reproduces them.

## Style

* Keep `rebirth-ik` free of `unsafe`; the core crate has `#![forbid(unsafe_code)]`. FFI crates may use it at the boundary only.
* New behaviour needs a test. Prefer scenario tests that assert geometry (reach, bone length, limits) over snapshot numbers.
