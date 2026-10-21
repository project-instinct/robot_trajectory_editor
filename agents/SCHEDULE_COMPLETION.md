# Schedule completion and live-contact selection: diagnosis and negative result

2026-10-21, branch `fix/schedule-completion`, based on integrated `feat/TO`
at `fac4ae3`. The behavioral changes described below were measured and then
reverted; this note records the diagnosis and why they did not earn a commit.

## Measured baseline (unchanged code)

45 s, default penalty, `--global-worst`, 12 workers, Kengo walk_0052 and Nokov
`jumpsit2` first 750 frames:

| Motion | Failure | Residuals |
|---|---|---|
| Kengo walk_0052 | validation at frame 733 | dynamics 501.42 N, position 1.93e-4 rad, clearance 1.08 mm |
| Nokov jumpsit2 | restoration score | effort 49.63 Nm, dynamics 691.12 N, clearance 14.32 mm |

The full backend suite is **58 passed**.

## Diagnosis

Exact distances, signed normals, and geometry names were measured on the
smoothed, aligned reference and on the captured candidates.

- **Nokov 350-360 is genuine flight, not missed support.** At frames 350/355
  the base is at z 0.948/0.971 m and the nearest shapes are the right ankle
  against a box *side* (39.1/43.4 mm, normal `[0,1,0]`, normal z ~0). Base
  vertical acceleration is -11.5/-8.5 m/s2, i.e. ballistic. Seeding support
  there would be a false contact. The knee only meets the 0.5 m box top at
  frame 360 (15.9 mm, normal z 0.28), then penetrates by 0.2 mm at 365
  (normal z -0.47) and 14.3 mm at 370.
- **The real Nokov support loss is a 1-8 mm hover above existing support.**
  At frames 379-382 the hips/pelvis are 1.9-7.7 mm above the box top; the
  fixed schedule already names 4-5 contacts there, but they are outside the
  1 mm live margin. Live inference therefore drops them and the base force
  reaches 691 N (frame 381). Frame 370 is the complementary 14.3 mm knee
  penetration.
- **Kengo 733 is created by restoration, not by inference.** The nearest right
  ankle is 1.29 mm above ground with five scheduled contacts; the search
  candidate keeps live contact. The 501 N candidate is produced later by the
  smoothing plus whole-trajectory penetration lift, which leaves a 1.3 mm gap.
- **Kengo contains non-ballistic floating phases.** Frames 642/646 have both
  feet ~31 mm above ground while the base vertical acceleration is -1.0/-0.5
  instead of -9.81 m/s2, giving 392/401 N with zero contacts. The old 5 cm
  schedule supplied support across those gaps (a false contact); the live
  validator correctly rejects it.

## Changes tested and reverted

1. Schedule completion: proximity 5 cm -> 2 cm, and accept normal z > 0.1
   (instead of 0.7) for tilted box corners and seated/body support. This adds
   the knee phase at frame 360 and removes the 3-5 cm false support at Kengo
   642-646.
2. A per-sample `gap` group: for samples where the schedule has support,
   `max(0, min(clearance) - 1 mm)`, normalized by 1e-4 and included in the
   candidate score.
3. A gap penalty with a correct gradient. The earlier one-sided penalty was
   inert because `penetration_only` collision caching returns zero distance
   Jacobians for hovering shapes; the replacement pulls base z along the
   contact normal.
4. Contact-aware restoration: ballistic-gated (`|a_z + g| < 1.5` skips genuine
   flight) local close/lift of support phases, assessed on live inference, so
   restoration cannot lift the base off live support.

## Result

| Motion | After | Verdict |
|---|---|---|
| Kengo | validation at 733, dynamics 501.49 N, clearance 1.08 mm | unchanged |
| Nokov | effort 48.31 Nm, dynamics 672.07 N, clearance 14.33 mm | marginal |

The completion rule does seed the missing knee and seated contacts, and the
gap term reduces zero-contact samples on captured candidates, but the selected
and validated candidate does not change materially. The 50 cm-box and Kengo
runs still fail before producing a valid trajectory, so no result is applied.

## Remaining blocker

The failures are reference-level: a ~31 mm non-ballistic float (Kengo
642-646) and a 14 mm knee/hip penetration into a box top (Nokov 368-372).
Closing the float needs ~30 mm of base motion in about 0.1 s; the resulting
vertical acceleration is not absorbed by the available support, and sharper
corrections raise the live residual (measured 0.6-2.0 kN). Nokov's effort
violation likewise survives both the tighter schedule and the gap-aware
restoration. Fixing this needs either a reference retargeting pass that
enforces contact/ballistic phases, or a much longer search that jointly
relaxes base and joints under a contact schedule; a 45 s penalty search with a
fixed schedule cannot reach it.

Commands used (local `aorua`, conda `robot-data`):

```bash
TRAJECTORY_OPTIMIZE_WORKERS=12 python scripts/benchmark_trajectory_optimize.py \
  <motion.npz> <robot.urdf> [--terrain <terrain.stl>] --frames 750 \
  --mode solve --iterations 40 --friction 0.7 --global-worst
python -m pytest robot_trajectory_editor/backend/tests -q
```
