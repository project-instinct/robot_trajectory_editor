import * as THREE from 'three'
// Deep imports: the package root pulls in three/IKRootsHelper.js, which imports
// *BufferGeometry symbols removed from three r150+. We only need the core.
import { Solver } from 'closed-chain-ik/src/core/Solver.js'
import { Goal } from 'closed-chain-ik/src/core/Goal.js'
import { Joint } from 'closed-chain-ik/src/core/Joint.js'
import type { DOF } from 'closed-chain-ik/src/core/Joint.js'
import { Link } from 'closed-chain-ik/src/core/Link.js'
import { urdfRobotToIKRoot } from 'closed-chain-ik/src/three/urdfHelpers.js'
import type { URDFRobot } from 'urdf-loader'

// closed-chain-ik declares DOF as an ambient const enum, which cannot be
// accessed at runtime with verbatimModuleSyntax; mirror its values (X=0..EZ=5)
// and cast at the call sites.
const DOF_X = 0 as DOF
const DOF_Y = 1 as DOF
const DOF_Z = 2 as DOF
const DOF_EX = 3 as DOF
const DOF_EY = 4 as DOF
const DOF_EZ = 5 as DOF

export const CARTESIAN_POSITION_TOLERANCE = 4e-3
export const CARTESIAN_ORIENTATION_TOLERANCE = THREE.MathUtils.degToRad(0.1)

export interface IkGoalTarget {
  pos: THREE.Vector3
  /** Omit rotation for a position-only goal. */
  quat?: THREE.Quaternion
}

interface MutableUrdfJoint extends THREE.Object3D {
  jointValue: number[]
  setJointValue: (...values: number[]) => boolean
}

/**
 * urdfRobotToIKRoot copies each URDFJoint's current Object3D transform. Since
 * urdf-loader stores the current joint motion in that transform, converting a
 * posed robot would bake the pose into the fixed origin and then apply the DoF
 * value a second time. Build from zero joint values and restore the visible
 * robot synchronously after conversion.
 */
function makeNeutralIkRoot(urdfRobot: URDFRobot): Joint {
  const joints = Object.values(
    (urdfRobot as unknown as { joints?: Record<string, MutableUrdfJoint> }).joints ?? {},
  )
  const savedValues = joints.map(joint => [...joint.jointValue])
  try {
    joints.forEach(joint => joint.setJointValue(...joint.jointValue.map(() => 0)))
    return urdfRobotToIKRoot(urdfRobot, false) as unknown as Joint
  } finally {
    joints.forEach((joint, index) => joint.setJointValue(...savedValues[index]))
    urdfRobot.updateMatrixWorld(true)
  }
}

/**
 * IK over the URDF kinematic tree using closed-chain-ik.
 *
 * The base ('__world_joint__' root) always has its DoF cleared: the solver
 * never moves the base. Instead the base pose is set explicitly per solve
 * (from the trajectory / drag state), and the solver adjusts joint values:
 *
 * - solvePositionGoal: drag a link to a world position (default drag mode).
 * - solvePoseGoals: keep pinned links at fixed world poses while the base
 *   moves (pinned drag mode).
 *
 * Joint values are exchanged with the caller as name->angle maps matching
 * the URDF joint names (dofValues[0] of each 1-DoF IK joint).
 */
export class IkSolver {
  private ikRoot: Joint
  private solver: Solver
  private baseLocked: boolean
  private ikJoints = new Map<string, Joint>()
  private ikLinks = new Map<string, Link>()
  private goals = new Map<string, Goal>()

  constructor(urdfRobot: URDFRobot, options: { lockBase?: boolean } = {}) {
    // Keep all links (trimUnused=false) so hands/head remain valid IK targets.
    // (d.ts declares a Link return, but for a URDFRobot it returns the root Joint.)
    this.ikRoot = makeNeutralIkRoot(urdfRobot)
    this.baseLocked = options.lockBase ?? true
    // Viewport dragging keeps the trajectory-driven base locked. Cartesian
    // interpolation can opt into solving all six floating-base DoFs.
    if (this.baseLocked) this.ikRoot.clearDoF()

    this.ikRoot.traverse((frame) => {
      const name = frame.name as string
      if (frame instanceof Joint && name && name !== '__world_joint__') {
        this.ikJoints.set(name, frame)
      } else if (frame instanceof Link && name) {
        this.ikLinks.set(name, frame)
      }
      return false
    })

    this.solver = new Solver([this.ikRoot])
    if (this.baseLocked) {
      this.solver.maxIterations = 50
      this.solver.translationConvergeThreshold = 1e-3
      this.solver.rotationConvergeThreshold = 1e-4
    } else {
      // Multi-link floating-base goals are often square or nearly singular.
      // Stronger damping prevents the normal-equation solve from overshooting
      // and falsely reporting divergence on small continuation steps.
      this.solver.maxIterations = 100
      this.solver.translationConvergeThreshold = CARTESIAN_POSITION_TOLERANCE
      this.solver.rotationConvergeThreshold = CARTESIAN_ORIENTATION_TOLERANCE
      // The damped normal-equation path amplifies nearly singular directions
      // in straight-limb poses. SVD drops those directions instead of turning
      // millimetre-scale goals into radian-scale joint changes.
      this.solver.useSVD = true
      this.solver.stallThreshold = 1e-7
      this.solver.dampingFactor = 0.05
      this.solver.divergeThreshold = 0.1
      this.solver.restPoseFactor = 0.05
      this.solver.translationErrorClamp = 0.02
      this.solver.rotationErrorClamp = 0.02
    }
    this.solver.updateStructure()
  }

  hasLink(linkName: string): boolean {
    return this.ikLinks.has(linkName)
  }

  /** Reads a link pose directly from the IK tree for residual diagnostics. */
  getLinkWorldPose(linkName: string): { pos: THREE.Vector3; quat: THREE.Quaternion } | null {
    const link = this.ikLinks.get(linkName)
    if (!link) return null
    this.ikRoot.updateMatrixWorld()
    const position = [0, 0, 0]
    const quaternion = [0, 0, 0, 1]
    link.getWorldPosition(position)
    link.getWorldQuaternion(quaternion)
    return {
      pos: new THREE.Vector3(position[0], position[1], position[2]),
      quat: new THREE.Quaternion(
        quaternion[0], quaternion[1], quaternion[2], quaternion[3],
      ).normalize(),
    }
  }

  /** Sets the base pose (xyz + wxyz) and joint values the solve starts from. */
  setConfiguration(basePos: THREE.Vector3, baseQuat: THREE.Quaternion, jointValues: Map<string, number>): void {
    if (this.baseLocked) {
      this.ikRoot.setPosition(basePos.x, basePos.y, basePos.z)
      this.ikRoot.setQuaternion(baseQuat.x, baseQuat.y, baseQuat.z, baseQuat.w)
    } else {
      // closed-chain-ik composes its EX/EY/EZ values with gl-matrix's
      // quat.fromEuler, whose installed default intrinsic order is ZYX.
      const euler = new THREE.Euler().setFromQuaternion(baseQuat, 'ZYX')
      const baseValues: [DOF, number][] = [
        [DOF_X, basePos.x], [DOF_Y, basePos.y], [DOF_Z, basePos.z],
        [DOF_EX, euler.x], [DOF_EY, euler.y], [DOF_EZ, euler.z],
      ]
      for (const [dof, value] of baseValues) {
        this.ikRoot.setDoFValue(dof, value)
        this.ikRoot.setRestPoseValue(dof, value)
      }
      this.ikRoot.restPoseSet = true
    }
    for (const [name, value] of jointValues) {
      const joint = this.ikJoints.get(name)
      if (joint && joint.dof.length > 0) {
        const dof = joint.dof[0] as unknown as DOF
        joint.setDoFValue(dof, value)
        if (!this.baseLocked) {
          joint.setRestPoseValue(dof, value)
          joint.restPoseSet = true
        }
      }
    }
    this.ikRoot.setMatrixWorldNeedsUpdate()
  }

  /** Positions-only goal (translation DoF), used for dragging a link. */
  solvePositionGoal(linkName: string, targetPos: THREE.Vector3): void {
    this.keepGoalKeys([`${linkName}|pos`])
    const goal = this.ensureGoal(linkName, false)
    goal.setPosition(targetPos.x, targetPos.y, targetPos.z)
    goal.setMatrixNeedsUpdate()
    this.solver.solve()
  }

  /** Mixed position-only and full-pose goals solved together. */
  solveGoals(targets: ReadonlyMap<string, IkGoalTarget>): number[] {
    this.keepGoalKeys([...targets].map(([name, target]) => `${name}|${target.quat ? 'pose' : 'pos'}`))
    for (const [linkName, target] of targets) {
      const goal = this.ensureGoal(linkName, Boolean(target.quat))
      goal.setPosition(target.pos.x, target.pos.y, target.pos.z)
      if (target.quat) {
        goal.setQuaternion(target.quat.x, target.quat.y, target.quat.z, target.quat.w)
      }
      goal.setMatrixNeedsUpdate()
    }
    // Avoid exporting the package's ambient const enum through this wrapper.
    return this.solver.solve().map(Number)
  }

  /** Full-pose convenience wrapper used by pinned-link dragging. */
  solvePoseGoals(targets: ReadonlyMap<string, { pos: THREE.Vector3; quat: THREE.Quaternion }>): number[] {
    return this.solveGoals(targets)
  }

  clearGoals(): void {
    this.keepGoalKeys([])
  }

  /** Reads solved joint values as urdfJointName -> angle. */
  getJointValues(): Map<string, number> {
    const out = new Map<string, number>()
    for (const [name, joint] of this.ikJoints) {
      if (joint.dof.length > 0) {
        out.set(name, Number(joint.getDoFValue(joint.dof[0] as unknown as DOF)))
      }
    }
    return out
  }

  /** Reads the floating base solved by an unlocked-base solver. */
  getBasePose(): { pos: THREE.Vector3; quat: THREE.Quaternion } {
    if (this.baseLocked) throw new Error('IkSolver: the floating base is locked')
    const position = [0, 0, 0]
    const euler = [0, 0, 0]
    this.ikRoot.getDoFPosition(position)
    this.ikRoot.getDoFEuler(euler)
    return {
      pos: new THREE.Vector3(position[0], position[1], position[2]),
      quat: new THREE.Quaternion().setFromEuler(
        new THREE.Euler(euler[0], euler[1], euler[2], 'ZYX'),
      ),
    }
  }

  private ensureGoal(linkName: string, withRotation: boolean): Goal {
    const key = `${linkName}|${withRotation ? 'pose' : 'pos'}`
    const existing = this.goals.get(key)
    if (existing) return existing
    const link = this.ikLinks.get(linkName)
    if (!link) throw new Error(`IkSolver: unknown link "${linkName}"`)

    const goal = new Goal()
    if (withRotation) goal.setDoF(DOF_X, DOF_Y, DOF_Z, DOF_EX, DOF_EY, DOF_EZ)
    else goal.setDoF(DOF_X, DOF_Y, DOF_Z)
    goal.makeClosure(link)

    this.goals.set(key, goal)
    this.solver.updateStructure()
    return goal
  }

  private keepGoalKeys(goalKeys: string[]): void {
    const keep = new Set(goalKeys)
    let changed = false
    for (const [key, goal] of this.goals) {
      if (keep.has(key)) continue
      try { goal.removeChild(goal.child!) } catch { /* already detached */ }
      this.goals.delete(key)
      changed = true
    }
    if (changed) this.solver.updateStructure()
  }
}
