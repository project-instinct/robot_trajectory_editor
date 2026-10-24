# Contact-free support diagnosis and smooth warm start

2026-10-21, branch `fix/contact-free`, based on `dea39e7`.

## Classification

The trace's interval/fraction pair is zero based. Fraction 1 in interval 285
is node 286; fraction 1 in interval 383 is node 384. Measurements below use
exact shape/terrain distances, not broad-phase bounds, on the baseline's best
evaluated dynamics candidate.

| Sample | Left/right foot gap | COM vertical acceleration | Classification |
|---|---|---|---|
| Kengo 285, fraction 1 | 49.37 / 43.52 mm | +0.744 m/s² | Missed standing support |
| Nokov 383, fraction 1 | 101.41 / 113.32 mm | +12.608 m/s² | Missed seated/body support |

Kengo's raw feet are 51.45 / 45.59 mm high at node 286. Its COM remains around
0.79 m through neighboring frames, with finite-difference vertical acceleration
near zero. In the evaluated candidate, nodes 95–299 have no inferred contact;
this prolonged nearly level motion is inconsistent with ballistic flight.
Neighboring nodes 276–288 have similar foot gaps and COM acceleration between
about -0.4 and +0.75 m/s².

Nokov's feet are intentionally suspended while seating. At node 384 the nearest
body surfaces are the left wrist (7.08 mm) and pelvis (8.32 mm), with upward
terrain normals. The candidate loses all contacts at nodes 381–388, between
nearby knee/body support and resumed hip/body support. The raw reference also
has a short gap at nodes 381–386; its endpoint COM acceleration at node 384 is
+0.26 m/s², rather than -9.81 m/s². This is a missed seating/support transition,
not evidence for adding foot contacts during flight.

## Change

The previous warm start used `clearances()` values that may be conservative
bounding-sphere lower bounds when a shape is culled. Such bounds are valid for
rejecting distant collision candidates but cannot measure how far a foot/body
must move to reach the terrain. Aligning to them can leave centimeter gaps.

`clearances(exact=True)` bypasses that culling for the warm start. A 21-frame
median clearance window selects nearby, upward-facing geometry, including body
support. Fit the resulting vertical alignment targets with a cubic B-spline,
using knots approximately 0.3 seconds apart on trajectory nodes. Add both its
positions and analytic velocities so the Hermite curve reproduces the same
smooth correction. A null-space fit constrains the offset to zero at fixed
poses. Two/three-frame inputs use linear/quadratic fits.

This is only an initialization change. No contact labels are installed in the
model or validator. The 1 mm contact-inference margin, 1e-5 m penetration
tolerance, effort bounds, deadline, and independent float32 validation remain
unchanged. A spline fit is approximate: it does not ensure contact throughout
every inferred support phase.

## Verification

Execution uses `ziwen-galaxea-desktop-rustdesk`, isolated scratch
`/data/Projects/robot_trajectory_editor_contact_free`, conda `omniretargeting`,
and `/data/Projects/robot_trajectory_editor/.venv-processing/bin/python`.
Imports resolve to the scratch backend. All benchmarks use the default penalty
solver, 45-second budget, friction 0.7, no anchors, `--iterations 40`, and
`--global-worst`; BLAS/OpenMP threads are limited to one.

The existing worker heuristic caps a 300-frame motion at six workers even with
`TRAJECTORY_OPTIMIZE_WORKERS=16`. The Kengo comparison therefore uses a scratch
benchmark harness that overrides only pool creation and interval splitting to
force 16 actual workers/chunks. Solver source keeps its existing worker policy.
Nokov's requested six workers are already honored, using the STL terrain from
the README. Source files and tests are checked by SHA-256 after transfer.

| Motion | Frames/workers | Before global worst | After global worst | Solve outcome |
|---|---|---|---|---|
| Kengo walk_0052 | 300 / 16 | 433.70 N | 155.11 N | Penalty search fails after 25.28 s |
| Nokov jumpsit2, STL | 750 / 6 | 707.58 N | 534.07 N | Timeout after 45.02 s |

These are minima over completed, parent-merged evaluations, not accepted outputs.
The force-best Kengo candidate has effort violation 171.30 Nm, penetration
1.77 mm, and position violation 0.000146 rad. Its worst sample moves to interval
95, fraction 0.788675, still without inferred contact. The solver's retained
failure diagnostic remains 665.27 N; best normalized-score selection does not
select the minimum-force candidate. The baseline also terminates unsuccessfully
(22.86 s with 16 actual workers).

The force-best Nokov candidate has effort violation 166.70 Nm, penetration
14.30 mm (baseline 278.69 Nm and 11.51 mm), and position violation 0.00204 rad.
Its worst sample remains interval 383, fraction 1, without inferred contact.
Both runs fail before final output validation; **neither produces a validated
trajectory or applies any result**. Lower dynamics residual alone is not a
feasibility result, and Nokov penetration worsens. The handoff's baseline
Nokov minimum was approximately 703 N; the repeated 45-second run here reaches
707.58 N, so either comparison still shows a material improvement.

Logs and captured candidates stay in the remote scratch directory:
`kengo_16_baseline.log`, `kengo_16_final.log`, `nokov_baseline.log`,
`nokov_final.log`, matching `*_best.npz` snapshots, `*_diagnosis.log`, and the
scratch `contact_free_capture*.py` harnesses. The earlier unforced Kengo runs
used six effective workers and are not the 16-worker comparison above.

The full remote backend suite passes: **58 passed**. New tests check
the actual distance of a hovering box foot and its dense validation after
repair, fixed-pose preservation with continuous reconstructed acceleration,
and absence of added support for distant airborne geometry.

Remaining blocker: the default penalty search still leaves contact-free gaps
and large dynamics/effort violations. Smooth initialization reduces the global
force residual but does not establish whole-motion feasibility within 45 s.
