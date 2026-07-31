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

/**
 * IK over the URDF kinematic tree using closed-chain-ik.
 *
 * The base ('__world_joint__' root) always has its DoF cleared: the solver
 * never moves the base. Instead the base pose is set explicitly per solve
 * (from the trajectory / drag state), and the solver adjusts joint values:
 *
 * - solvePositionGoal: drag a link to a world position (default drag mode).
 * - solvePoseGoal: keep a pinned link at a fixed world pose while the base
 *   moves (pinned drag mode).
 *
 * Joint values are exchanged with the caller as name->angle maps matching
 * the URDF joint names (dofValues[0] of each 1-DoF IK joint).
 */
export class IkSolver {
  private ikRoot: Joint
  private solver: Solver
  private ikJoints = new Map<string, Joint>()
  private ikLinks = new Map<string, Link>()
  private goal: Goal | null = null
  private goalKey = ''

  constructor(urdfRobot: URDFRobot) {
    // Keep all links (trimUnused=false) so hands/head remain valid IK targets.
    // (d.ts declares a Link return, but for a URDFRobot it returns the root Joint.)
    this.ikRoot = urdfRobotToIKRoot(urdfRobot, false) as unknown as Joint
    // Lock the floating base: base pose is driven by the trajectory, not IK.
    this.ikRoot.clearDoF()

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
    this.solver.maxIterations = 50
    this.solver.translationConvergeThreshold = 1e-3
    this.solver.rotationConvergeThreshold = 1e-4
    this.solver.updateStructure()
  }

  hasLink(linkName: string): boolean {
    return this.ikLinks.has(linkName)
  }

  /** Sets the base pose (xyz + wxyz) and joint values the solve starts from. */
  setConfiguration(basePos: THREE.Vector3, baseQuat: THREE.Quaternion, jointValues: Map<string, number>): void {
    this.ikRoot.setPosition(basePos.x, basePos.y, basePos.z)
    this.ikRoot.setQuaternion(baseQuat.x, baseQuat.y, baseQuat.z, baseQuat.w)
    for (const [name, value] of jointValues) {
      const joint = this.ikJoints.get(name)
      if (joint && joint.dof.length > 0) {
        joint.setDoFValue(joint.dof[0] as unknown as DOF, value)
      }
    }
    this.ikRoot.setMatrixWorldNeedsUpdate()
  }

  /** Positions-only goal (translation DoF), used for dragging a link. */
  solvePositionGoal(linkName: string, targetPos: THREE.Vector3): void {
    const goal = this.ensureGoal(linkName, false)
    goal.setPosition(targetPos.x, targetPos.y, targetPos.z)
    goal.setMatrixNeedsUpdate()
    this.solver.solve()
  }

  /** Full pose goal (translation + rotation DoF), used for pinned links. */
  solvePoseGoal(linkName: string, targetPos: THREE.Vector3, targetQuat: THREE.Quaternion): void {
    const goal = this.ensureGoal(linkName, true)
    goal.setPosition(targetPos.x, targetPos.y, targetPos.z)
    goal.setQuaternion(targetQuat.x, targetQuat.y, targetQuat.z, targetQuat.w)
    goal.setMatrixNeedsUpdate()
    this.solver.solve()
  }

  clearGoal(): void {
    if (this.goal) {
      try { this.goal.removeChild(this.goal.child!) } catch (_) { /* already detached */ }
      this.goal = null
      this.goalKey = ''
      this.solver.updateStructure()
    }
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

  private ensureGoal(linkName: string, withRotation: boolean): Goal {
    const key = `${linkName}|${withRotation ? 'pose' : 'pos'}`
    if (this.goal && this.goalKey === key) return this.goal

    if (this.goal) {
      try { this.goal.removeChild(this.goal.child!) } catch (_) { /* already detached */ }
      this.goal = null
      this.goalKey = ''
    }
    const link = this.ikLinks.get(linkName)
    if (!link) throw new Error(`IkSolver: unknown link "${linkName}"`)

    const goal = new Goal()
    if (withRotation) goal.setDoF(DOF_X, DOF_Y, DOF_Z, DOF_EX, DOF_EY, DOF_EZ)
    else goal.setDoF(DOF_X, DOF_Y, DOF_Z)
    goal.makeClosure(link)

    this.goal = goal
    this.goalKey = key
    this.solver.updateStructure()
    return goal
  }
}
