import * as THREE from 'three'
import { Trajectory } from '../state/Trajectory'
import {
  CARTESIAN_ORIENTATION_TOLERANCE,
  CARTESIAN_POSITION_TOLERANCE,
  IkSolver,
  type IkGoalTarget,
} from './IkSolver'
import type { RobotModel } from './RobotModel'

export type CartesianInterpolationMethod = 'linear' | 'cubic'
export type CartesianLinkConstraint = 'position' | 'pose'
export type CartesianLocalPoint = [number, number, number]

export interface CartesianLinkSelection {
  linkName: string
  constraint: CartesianLinkConstraint
  /** Point expressed in the selected link's local frame. */
  localPoint?: CartesianLocalPoint
}

export type CartesianInterpolationResult =
  | {
      ok: true
      jointPos: Float32Array
      basePoseW: Float32Array
      baseQuatW: Float32Array
    }
  | { ok: false; message: string }

interface LinkPose {
  pos: THREE.Vector3
  quat: THREE.Quaternion
}

interface RobotConfiguration extends LinkPose {
  joints: Map<string, number>
}

const SOLVE_STATUS_NAMES = ['converged', 'stalled', 'diverged', 'timed out']
const ADAPTIVE_SUBSTEPS = 8

export function interpolationAlpha(t: number, method: CartesianInterpolationMethod): number {
  return method === 'cubic' ? t * t * (3 - 2 * t) : t
}

export function localPointToWorld(
  pose: { pos: THREE.Vector3; quat: THREE.Quaternion },
  localPoint: readonly [number, number, number],
): THREE.Vector3 {
  return new THREE.Vector3(...localPoint).applyQuaternion(pose.quat).add(pose.pos)
}

function frameConfiguration(
  trajectory: Trajectory,
  robot: RobotModel,
  frame: number,
): RobotConfiguration {
  const state = trajectory.getFrame(frame)
  const joints = new Map<string, number>()
  for (const [name, binding] of robot.jointMap) {
    joints.set(name, state.jointPos[binding.index] ?? 0)
  }
  return {
    pos: new THREE.Vector3(
      state.basePoseW[0], state.basePoseW[1], state.basePoseW[2],
    ),
    quat: new THREE.Quaternion(
      state.baseQuatW[1], state.baseQuatW[2], state.baseQuatW[3], state.baseQuatW[0],
    ),
    joints,
  }
}

function readLinkPoses(robot: RobotModel, linkNames: string[]): Map<string, LinkPose> | null {
  const poses = new Map<string, LinkPose>()
  for (const linkName of linkNames) {
    const pose = robot.getLinkWorldPose(linkName)
    if (!pose) return null
    poses.set(linkName, { pos: pose.pos.clone(), quat: pose.quat.clone() })
  }
  return poses
}

function makeTargets(
  linkSelections: CartesianLinkSelection[],
  startPoses: ReadonlyMap<string, LinkPose>,
  endPoses: ReadonlyMap<string, LinkPose>,
  alpha: number,
): Map<string, IkGoalTarget> {
  const targets = new Map<string, IkGoalTarget>()
  for (const { linkName, constraint, localPoint } of linkSelections) {
    const first = startPoses.get(linkName)!
    const last = endPoses.get(linkName)!
    const point: CartesianLocalPoint = localPoint ?? [0, 0, 0]
    targets.set(linkName, {
      pos: constraint === 'position'
        ? localPointToWorld(first, point).lerp(localPointToWorld(last, point), alpha)
        : first.pos.clone().lerp(last.pos, alpha),
      quat: constraint === 'pose' ? first.quat.clone().slerp(last.quat, alpha) : undefined,
      localPoint: constraint === 'position' ? point : undefined,
    })
  }
  return targets
}

function solveConverged(statuses: number[]): boolean {
  return statuses.length > 0 && statuses.every(status => status === 0)
}

function solveStatusSummary(statuses: number[]): string {
  const names = [...new Set(statuses.map(status => SOLVE_STATUS_NAMES[status] ?? `status ${status}`))]
  return names.join(', ')
}

function targetsSatisfied(
  solver: IkSolver,
  targets: ReadonlyMap<string, IkGoalTarget>,
): boolean {
  for (const [linkName, target] of targets) {
    const actual = solver.getLinkWorldPose(linkName)
    if (!actual) return false
    const actualPosition = target.localPoint
      ? localPointToWorld(actual, target.localPoint)
      : actual.pos
    if (actualPosition.distanceTo(target.pos) > CARTESIAN_POSITION_TOLERANCE) return false
    if (
      target.quat
      && actual.quat.angleTo(target.quat) > CARTESIAN_ORIENTATION_TOLERANCE
    ) return false
  }
  return true
}

function rounded(value: number): number {
  return Number(value.toFixed(6))
}

function vectorValues(value: THREE.Vector3): number[] {
  return [rounded(value.x), rounded(value.y), rounded(value.z)]
}

function quaternionValues(value: THREE.Quaternion): number[] {
  return [rounded(value.x), rounded(value.y), rounded(value.z), rounded(value.w)]
}

function jointValues(values: ReadonlyMap<string, number>): Record<string, number> {
  return Object.fromEntries([...values].map(([name, value]) => [name, rounded(value)]))
}

function linkResiduals(
  solver: IkSolver,
  linkSelections: CartesianLinkSelection[],
  targets: ReadonlyMap<string, IkGoalTarget>,
) {
  return linkSelections.map(({ linkName, constraint }) => {
    const target = targets.get(linkName)!
    const actual = solver.getLinkWorldPose(linkName)
    const actualPosition = actual && target.localPoint
      ? localPointToWorld(actual, target.localPoint)
      : actual?.pos
    return {
      link: linkName,
      constraint,
      targetPosition: vectorValues(target.pos),
      localPoint: target.localPoint ?? null,
      actualPosition: actualPosition ? vectorValues(actualPosition) : null,
      positionErrorMeters: actualPosition ? rounded(actualPosition.distanceTo(target.pos)) : null,
      targetQuaternionXyzw: target.quat ? quaternionValues(target.quat) : null,
      actualQuaternionXyzw: actual ? quaternionValues(actual.quat) : null,
      orientationErrorDegrees: actual && target.quat
        ? rounded(THREE.MathUtils.radToDeg(actual.quat.angleTo(target.quat)))
        : null,
    }
  })
}

function logIkFailure(args: {
  solver: IkSolver
  frame: number
  adaptiveStep: number
  segmentStart: number
  segmentEnd: number
  method: CartesianInterpolationMethod
  linkSelections: CartesianLinkSelection[]
  pathT: number
  alpha: number
  directStatuses: number[]
  adaptiveStatuses: number[]
  seed: RobotConfiguration
  seedResiduals: ReturnType<typeof linkResiduals>
  targets: ReadonlyMap<string, IkGoalTarget>
}): void {
  const solvedBase = args.solver.getBasePose()
  const solvedJoints = args.solver.getJointValues()
  let maxJointDelta = { name: '', radians: 0 }
  for (const [name, value] of solvedJoints) {
    const delta = Math.abs(value - (args.seed.joints.get(name) ?? value))
    if (delta > maxJointDelta.radians) maxJointDelta = { name, radians: delta }
  }
  const report = {
    frame: args.frame,
    adaptiveStep: `${args.adaptiveStep}/${ADAPTIVE_SUBSTEPS}`,
    segment: [args.segmentStart, args.segmentEnd],
    method: args.method,
    links: args.linkSelections,
    normalizedTime: rounded(args.pathT),
    interpolationAlpha: rounded(args.alpha),
    directStatuses: args.directStatuses.map(status => SOLVE_STATUS_NAMES[status] ?? status),
    adaptiveStatuses: args.adaptiveStatuses.map(status => SOLVE_STATUS_NAMES[status] ?? status),
    seed: {
      basePosition: vectorValues(args.seed.pos),
      baseQuaternionXyzw: quaternionValues(args.seed.quat),
      joints: jointValues(args.seed.joints),
    },
    seedResiduals: args.seedResiduals,
    solved: {
      basePosition: vectorValues(solvedBase.pos),
      baseQuaternionXyzw: quaternionValues(solvedBase.quat),
      basePositionDeltaMeters: rounded(solvedBase.pos.distanceTo(args.seed.pos)),
      baseOrientationDeltaDegrees: rounded(
        THREE.MathUtils.radToDeg(solvedBase.quat.angleTo(args.seed.quat)),
      ),
      maxJointDelta: {
        name: maxJointDelta.name,
        radians: rounded(maxJointDelta.radians),
      },
      joints: jointValues(solvedJoints),
    },
    residuals: linkResiduals(args.solver, args.linkSelections, args.targets),
  }
  console.error('[Cartesian IK diagnostics]\n' + JSON.stringify(report, null, 2))
}

/**
 * Computes Cartesian interpolation against a clone and returns only candidate
 * joint data. The caller owns committing it, so any failure is transactional.
 */
export function computeCartesianInterpolation(
  trajectory: Trajectory,
  robot: RobotModel,
  linkSelections: CartesianLinkSelection[],
  segmentStart: number,
  segmentEnd: number,
  method: CartesianInterpolationMethod,
  restoreFrame: number,
): CartesianInterpolationResult {
  const start = Math.min(segmentStart, segmentEnd)
  const end = Math.max(segmentStart, segmentEnd)
  if (start === end) return { ok: false, message: 'Select a segment spanning at least two frames.' }
  if (linkSelections.length === 0) return { ok: false, message: 'Select at least one link.' }
  const linkNames = linkSelections.map(selection => selection.linkName)

  const candidate = trajectory.clone()
  const restore = trajectory.getFrame(restoreFrame)

  try {
    robot.applyFrame(
      candidate.getFrame(start).jointPos,
      candidate.getFrame(start).basePoseW,
      candidate.getFrame(start).baseQuatW,
    )
    const startPoses = readLinkPoses(robot, linkNames)

    robot.applyFrame(
      candidate.getFrame(end).jointPos,
      candidate.getFrame(end).basePoseW,
      candidate.getFrame(end).baseQuatW,
    )
    const endPoses = readLinkPoses(robot, linkNames)
    if (!startPoses || !endPoses) {
      return { ok: false, message: 'Could not read one or more selected link poses.' }
    }

    const solver = new IkSolver(robot.urdfRobot, { lockBase: false })
    // The endpoints already define the target poses and are known-valid robot
    // states. Preserve them exactly, and solve the interior as a continuous IK
    // path, warm-starting each frame from the previous solution.
    for (let frame = start + 1; frame < end; frame++) {
      const t = (frame - start) / (end - start)
      const alpha = interpolationAlpha(t, method)
      const targets = makeTargets(linkSelections, startPoses, endPoses, alpha)

      // Forward continuation: the first interior frame starts from the selected
      // start state; every subsequent frame starts from the previous IK result.
      const seed = frameConfiguration(candidate, robot, frame - 1)
      solver.setConfiguration(seed.pos, seed.quat, seed.joints)
      let statuses = targetsSatisfied(solver, targets) ? [0] : solver.solveGoals(targets)
      if (targetsSatisfied(solver, targets)) statuses = [0]
      if (!solveConverged(statuses)) {
        const directStatuses = statuses
        // Retry through smaller hidden Cartesian increments. Reset first so a
        // diverged direct attempt cannot contaminate the continuation path.
        solver.setConfiguration(seed.pos, seed.quat, seed.joints)
        for (let step = 1; step <= ADAPTIVE_SUBSTEPS; step++) {
          const subframeT = ((frame - 1 - start) + step / ADAPTIVE_SUBSTEPS) / (end - start)
          const subAlpha = interpolationAlpha(subframeT, method)
          const subTargets = makeTargets(linkSelections, startPoses, endPoses, subAlpha)
          const seedResiduals = linkResiduals(solver, linkSelections, subTargets)
          statuses = targetsSatisfied(solver, subTargets) ? [0] : solver.solveGoals(subTargets)
          if (targetsSatisfied(solver, subTargets)) statuses = [0]
          if (!solveConverged(statuses)) {
            logIkFailure({
              solver,
              frame,
              adaptiveStep: step,
              segmentStart: start,
              segmentEnd: end,
              method,
              linkSelections,
              pathT: subframeT,
              alpha: subAlpha,
              directStatuses,
              adaptiveStatuses: statuses,
              seed,
              seedResiduals,
              targets: subTargets,
            })
            const status = solveStatusSummary(statuses)
            return {
              ok: false,
              message: `IK ${status} at frame ${frame} (adaptive step ${step}/${ADAPTIVE_SUBSTEPS}). No changes were applied. See the browser console for [Cartesian IK diagnostics].`,
            }
          }
        }
      }

      const solvedBase = solver.getBasePose()
      const baseValues = [solvedBase.pos.x, solvedBase.pos.y, solvedBase.pos.z,
        solvedBase.quat.x, solvedBase.quat.y, solvedBase.quat.z, solvedBase.quat.w]
      if (!baseValues.every(Number.isFinite)) {
        return { ok: false, message: `IK returned an invalid base pose at frame ${frame}. No changes were applied.` }
      }
      candidate.setBasePose(
        frame,
        [solvedBase.pos.x, solvedBase.pos.y, solvedBase.pos.z],
        [solvedBase.quat.w, solvedBase.quat.x, solvedBase.quat.y, solvedBase.quat.z],
      )

      for (const [name, value] of solver.getJointValues()) {
        const binding = robot.jointMap.get(name)
        if (!binding) continue
        if (!Number.isFinite(value)) {
          return { ok: false, message: `IK returned an invalid value at frame ${frame}. No changes were applied.` }
        }
        candidate.setJointValue(frame, binding.index, value)
      }
    }
    return {
      ok: true,
      jointPos: new Float32Array(candidate.jointPos),
      basePoseW: new Float32Array(candidate.basePoseW),
      baseQuatW: new Float32Array(candidate.baseQuatW),
    }
  } catch (error) {
    const detail = error instanceof Error ? error.message : String(error)
    return { ok: false, message: `Cartesian interpolation failed: ${detail}` }
  } finally {
    robot.applyFrame(restore.jointPos, restore.basePoseW, restore.baseQuatW)
  }
}
