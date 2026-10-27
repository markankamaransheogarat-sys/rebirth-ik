# Changelog

## 0.1.0 — initial release

A from-scratch Rust rewrite of [TheComet/ik](https://github.com/TheComet/ik).

- FABRIK for arbitrary bone trees with any number of effectors (chain decomposition ported from the original, with its test scenarios).
- Effector weights (linear and NLERP), per-effector chain length, optional joint rotations and target rotations.
- `Cone`, `Hinge` and `Stiff` joint constraints.
- Analytic `one_bone` and `two_bone` solvers (with pole vector).
- Allocation-free `solve_chain` and parallel `solve_chains_par` (feature `rayon`).
- `#![forbid(unsafe_code)]`, `no_std` (feature `libm`), WebAssembly bindings and a live demo.
