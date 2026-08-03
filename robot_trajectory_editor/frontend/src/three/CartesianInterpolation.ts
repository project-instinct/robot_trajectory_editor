import * as THREE from 'three'
import { Trajectory } from '../state/Trajectory'
import { IkSolver } from './IkSolver'
import type { RobotModel } from './RobotModel'

export type CartesianInterpolationMethod = 'linear' | 'cubic'

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

export function interpolationAlpha(t: number, method: CartesianInterpolationMethod): number {
  return method === 'cubic' ? t * t * (3 - 2 * t) : t
}

function frameBase(trajectory: Trajectory, frame: number): LinkPose {
  const state = trajectory.getFrame(frame)
  return {
    pos: new THREE.Vector3(state.basePoseW[0], state.basePoseW[1], state.basePoseW[2]),
    quat: new THREE.Quaternion(
      state.baseQuatW[1], state.baseQuatW[2], state.baseQuatW[3], state.baseQuatW[0],
    ),
  }
}

function frameJointMap(
  trajectory: Trajectory,
  robot: RobotModel,
  frame: number,
): Map<string, number> {
  const state = trajectory.getFrame(frame)
  const values = new Map<string, number>()
  for (const [name, binding] of robot.jointMap) {
    values.set(name, state.jointPos[binding.index] ?? 0)
  }
  return values
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

/**
 * Computes Cartesian interpolation against a clone and returns only candidate
 * joint data. The caller owns committing it, so any failure is transactional.
 */
export function computeCartesianInterpolation(
  trajectory: Trajectory,
  robot: RobotModel,
  linkNames: string[],
  segmentStart: number,
  segmentEnd: number,
  method: CartesianInterpolationMethod,
  restoreFrame: number,
): CartesianInterpolationResult {
  const start = Math.min(segmentStart, segmentEnd)
  const end = Math.max(segmentStart, segmentEnd)
  if (start === end) return { ok: false, message: 'Select a segment spanning at least two frames.' }
  if (linkNames.length === 0) return { ok: false, message: 'Select at least one link.' }

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
      const targets = new Map<string, LinkPose>()
      for (const linkName of linkNames) {
        const first = startPoses.get(linkName)!
        const last = endPoses.get(linkName)!
        targets.set(linkName, {
          pos: first.pos.clone().lerp(last.pos, alpha),
          quat: first.quat.clone().slerp(last.quat, alpha),
        })
      }

      const seedFrame = frame - 1
      const base = frameBase(candidate, seedFrame)
      solver.setConfiguration(base.pos, base.quat, frameJointMap(candidate, robot, seedFrame))
      if (!solver.solvePoseGoals(targets)) {
        return { ok: false, message: `IK failed to converge at frame ${frame}. No changes were applied.` }
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
