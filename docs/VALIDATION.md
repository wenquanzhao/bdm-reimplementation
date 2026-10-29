# Validation contract

Independent static prototype, not the complete BDM algorithm.

## Required invariants
- Occupied input cells query as occupied.
- Free input cells query as free, including free-to-occupied interfaces.
- Missing cells query as unknown.
- Rebuild discards previous state.
- Negative coordinates and disconnected free intervals work.

## Validation strategy
- Dense-vs-sparse exhaustive randomized differential testing.
- Adversarial columns with occupied/free/unknown transitions.
- Sanitizers and GCC/Clang CI.
- Performance measurement with resident memory, throughput, and update time.

## Missing
Paper-accurate O(1) query, probabilistic ray casting, sliding local grid,
incremental global-local update, ROS2, large-scale benchmarks.
