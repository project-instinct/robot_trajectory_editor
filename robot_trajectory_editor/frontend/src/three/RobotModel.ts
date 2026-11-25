import * as THREE from 'three'
import { type URDFRobot } from 'urdf-loader'

interface JointBinding {
  urdfJoint: THREE.Object3D
  name: string
  index: number
  lower: number
  upper: number
}

interface UrdfLinkObject extends THREE.Object3D {
  isURDFLink?: boolean
}

interface UrdfJointObject extends THREE.Object3D {
  jointType?: string
  angle?: number
  jointValue?: number[]
  limit?: { lower: number; upper: number }
}

export interface RobotSceneState {
  jointNames: string[]
  jointPos: Float32Array
  basePoseW: Float32Array
  baseQuatW: Float32Array
}

function readUrdfJointValue(joint: UrdfJointObject): number {
  if (typeof joint.angle === 'number' && Number.isFinite(joint.angle)) return joint.angle
  const first = joint.jointValue?.[0]
  return typeof first === 'number' && Number.isFinite(first) ? first : 0
}

function resolveJointLimit(limit?: { lower: number; upper: number }): { lower: number; upper: number } {
  const valid = limit !== undefined
    && Number.isFinite(limit.lower)
    && Number.isFinite(limit.upper)
    && limit.upper > limit.lower
  return valid ? { lower: limit.lower, upper: limit.upper } : { lower: -Math.PI, upper: Math.PI }
}

/** Finds the enclosing URDF link object for any descendant (mesh, visual, ...). */
export function findLinkAncestor(obj: THREE.Object3D): THREE.Object3D | null {
  let cur: THREE.Object3D | null = obj
  while (cur) {
    if ((cur as UrdfLinkObject).isURDFLink) return cur
    cur = cur.parent
  }
  return null
}

export class RobotModel {
  urdfRobot: URDFRobot
  rootGroup: THREE.Group
  rootLinkName: string
  jointMap: Map<string, JointBinding> = new Map()
  linkMeshes: Map<string, THREE.Mesh[]> = new Map()
  linkNodes: Map<string, THREE.Object3D> = new Map()
  pinnedLinks: string[] = []
  private pinnedOriginals: [THREE.Mesh, THREE.Material | THREE.Material[]][] = []
  private warnedInvalidFrame = false

  constructor(urdfRobot: URDFRobot, jointNames: string[]) {
    this.urdfRobot = urdfRobot
    this.rootGroup = new THREE.Group()
    this.rootGroup.add(urdfRobot)

    this.rootLinkName = ''
    urdfRobot.traverse((obj) => {
      const link = obj as UrdfLinkObject
      if (link.isURDFLink) {
        if (!this.rootLinkName) this.rootLinkName = link.name
        if (link.name) this.linkNodes.set(link.name, obj)
      }
    })

    this.refreshLinkMeshes()
    this.bindJoints(jointNames)
  }

  private getUrdfJoints(): Record<string, UrdfJointObject> {
    return (this.urdfRobot as unknown as { joints?: Record<string, UrdfJointObject> }).joints ?? {}
  }

  /** (Re)binds trajectory joint names to URDF joints (called on trajectory load). */
  bindJoints(jointNames: string[]): void {
    this.jointMap.clear()
    const urdfJoints = this.getUrdfJoints()
    jointNames.forEach((name, idx) => {
      const urdfJoint = urdfJoints[name]
      if (!urdfJoint) return
      const { lower, upper } = resolveJointLimit(urdfJoint.limit)
      this.jointMap.set(name, {
        urdfJoint,
        name,
        index: idx,
        lower,
        upper,
      })
    })
    if (jointNames.length > 0) {
      const missing = jointNames.filter(n => !urdfJoints[n])
      if (missing.length > 0) {
        console.warn(
          `[RobotModel] ${missing.length}/${jointNames.length} trajectory joints not found in URDF ` +
          `(e.g. ${missing.slice(0, 5).join(', ')}). Those joints will not be posed.`,
        )
      }
    }
  }

  getJointLimit(name: string): { lower: number; upper: number } | null {
    const b = this.jointMap.get(name)
    if (b) return { lower: b.lower, upper: b.upper }
    const urdfJoint = this.getUrdfJoints()[name]
    return urdfJoint ? resolveJointLimit(urdfJoint.limit) : null
  }

  /** Movable (non-fixed) URDF joint names in model order. */
  getMovableJointNames(): string[] {
    return Object.entries(this.getUrdfJoints())
      .filter(([, joint]) => joint.jointType !== 'fixed')
      .map(([name]) => name)
  }

  /** Live joint values from the 3D scene keyed by URDF joint name. */
  getJointValueMap(): Map<string, number> {
    const map = new Map<string, number>()
    const urdfJoints = this.getUrdfJoints()
    for (const [name, joint] of Object.entries(urdfJoints)) {
      if (joint.jointType === 'fixed') continue
      map.set(name, readUrdfJointValue(joint))
    }
    return map
  }

  /** Current base pose (xyz + wxyz) and movable joint values directly from the 3D scene. */
  getSceneState(): RobotSceneState {
    const urdfJoints = this.getUrdfJoints()
    const jointNames = this.getMovableJointNames()
    const jointPos = new Float32Array(jointNames.length)
    for (let i = 0; i < jointNames.length; i++) {
      jointPos[i] = readUrdfJointValue(urdfJoints[jointNames[i]])
    }
    const p = this.rootGroup.position
    const q = this.rootGroup.quaternion
    return {
      jointNames,
      jointPos,
      basePoseW: new Float32Array([p.x, p.y, p.z]),
      baseQuatW: new Float32Array([q.w, q.x, q.y, q.z]),
    }
  }

  /** Sets a single URDF joint value directly in the 3D scene. */
  setJointValue(name: string, value: number): void {
    if (!Number.isFinite(value)) return
    ;(this.urdfRobot as unknown as { setJointValue: (n: string, v: number) => void })
      .setJointValue(name, value)
    this.rootGroup.updateMatrixWorld()
  }

  /** Sets multiple URDF joint values directly in the 3D scene. */
  setJointValues(values: Map<string, number>): void {
    const setter = this.urdfRobot as unknown as { setJointValue: (n: string, v: number) => void }
    for (const [name, value] of values) {
      if (Number.isFinite(value)) setter.setJointValue(name, value)
    }
    this.rootGroup.updateMatrixWorld()
  }

  /** Sets the robot base pose (xyz + wxyz) directly in the 3D scene. */
  setBasePose(basePos: Float32Array | number[], baseQuat: Float32Array | number[]): void {
    const baseOk = [basePos[0], basePos[1], basePos[2], baseQuat[0], baseQuat[1], baseQuat[2], baseQuat[3]]
      .every(Number.isFinite)
    if (!baseOk) return
    this.rootGroup.position.set(basePos[0], basePos[1], basePos[2])
    const n = Math.hypot(baseQuat[0], baseQuat[1], baseQuat[2], baseQuat[3])
    if (n > 1e-6) {
      this.rootGroup.quaternion.set(baseQuat[1] / n, baseQuat[2] / n, baseQuat[3] / n, baseQuat[0] / n)
    } else {
      this.rootGroup.quaternion.identity()
    }
    this.rootGroup.updateMatrixWorld()
  }

  /** Links available as Cartesian interpolation targets. */
  getCartesianTargetLinkNames(): string[] {
    return [...this.linkNodes.keys()]
      .sort((a, b) => a.localeCompare(b))
  }

  /**
   * (Re)collects meshes per link. urdf-loader attaches visual meshes
   * asynchronously AFTER its onLoad callback, so this must run lazily
   * (on first use), not only in the constructor.
   */
  refreshLinkMeshes(): void {
    this.linkMeshes.clear()
    this.urdfRobot.traverse((obj) => {
      if (!(obj as THREE.Mesh).isMesh) return
      const mesh = obj as THREE.Mesh
      const linkName = findLinkAncestor(mesh)?.name ?? ''
      if (!this.linkMeshes.has(linkName)) this.linkMeshes.set(linkName, [])
      this.linkMeshes.get(linkName)!.push(mesh)
    })
  }

  /**
   * Applies a trajectory frame: base pose (xyz + wxyz) and joint values by name index.
   * Non-finite values are skipped (keeping the last valid pose) so a corrupted
   * frame cannot push the whole model into NaN-space and vanish from the scene.
   */
  applyFrame(jointValues: Float32Array, basePos: Float32Array, baseQuat: Float32Array): void {
    const baseOk = [basePos[0], basePos[1], basePos[2], baseQuat[0], baseQuat[1], baseQuat[2], baseQuat[3]]
      .every(Number.isFinite)
    if (baseOk) {
      this.rootGroup.position.set(basePos[0], basePos[1], basePos[2])
      const n = Math.hypot(baseQuat[0], baseQuat[1], baseQuat[2], baseQuat[3])
      if (n > 1e-6) {
        this.rootGroup.quaternion.set(baseQuat[1] / n, baseQuat[2] / n, baseQuat[3] / n, baseQuat[0] / n)
      } else {
        this.rootGroup.quaternion.identity()
      }
    } else if (!this.warnedInvalidFrame) {
      this.warnedInvalidFrame = true
      console.warn('[RobotModel] frame has non-finite base pose; keeping previous base pose')
    }

    for (const [, info] of this.jointMap) {
      const val = jointValues[info.index]
      if (val !== undefined && Number.isFinite(val)) {
        (this.urdfRobot as unknown as { setJointValue: (n: string, v: number) => void })
          .setJointValue(info.name, val)
      }
    }

    this.rootGroup.updateMatrixWorld()
  }

  getLinkWorldPose(linkName: string): { pos: THREE.Vector3; quat: THREE.Quaternion } | null {
    const node = this.linkNodes.get(linkName)
    if (!node) return null
    const pos = new THREE.Vector3()
    const quat = new THREE.Quaternion()
    node.getWorldPosition(pos)
    node.getWorldQuaternion(quat)
    return { pos, quat }
  }

  /** Highlights pinned links' meshes; restores original materials on unpin. */
  setPinned(linkNames: string[]): void {
    if (this.pinnedLinks.length === linkNames.length && this.pinnedLinks.every((name, i) => name === linkNames[i])) return
    this.pinnedLinks = [...linkNames]

    for (const [mesh, mat] of this.pinnedOriginals) mesh.material = mat
    this.pinnedOriginals = []

    if (linkNames.length === 0) return
    // Meshes attach asynchronously after load; make sure the map is current.
    this.refreshLinkMeshes()
    for (const linkName of linkNames) {
      const meshes = this.linkMeshes.get(linkName)
      if (!meshes) continue
      for (const mesh of meshes) {
        this.pinnedOriginals.push([mesh, mesh.material])
        const src = Array.isArray(mesh.material) ? mesh.material[0] : mesh.material
        const highlight = (src as THREE.MeshStandardMaterial).clone()
        if (highlight.emissive) highlight.emissive.set(0x664400)
        mesh.material = highlight
      }
    }
  }
}
