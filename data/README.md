# Scenario generation

For v0.3 the authoritative scenario data are generated in
`src/joint/sample_joint_problem.cpp`, not loaded from JSON.

- `baseline20`: 10 ARR and 10 DEP, 48 candidate options; A01/P1 has an
  arrival window [600,650] conflicting with D01's hard NTOT [630,690].
- `route-switch`: relaxes A01/P1's special arrival window; tests a different
  route-choice feasible region, without asserting which route is optimal.
- `fix-conflict`: uses 180-second X/Y fix separation instead of 90 seconds.
- `infeasible`: D01 has a hard window [500,600] but earliest takeoff >=630.

All values are synthetic and NOT approved aviation separation standards.
External JSON/CSV replay loading, dynamic STAR topology and capacity rules
remain outside v0.3. This limitation is explicit, not silently replaced by
hard-coded production assumptions.
