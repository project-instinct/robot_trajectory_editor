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

  /** (Re)binds trajectory joint names to URDF joints (called on trajectory load). */
  bindJoints(jointNames: string[]): void {
    this.jointMap.clear()
    const urdfJoints = (this.urdfRobot as unknown as { joints?: Record<string, THREE.Object3D> }).joints ?? {}
    jointNames.forEach((name, idx) => {
      const urdfJoint = urdfJoints[name]
      if (!urdfJoint) return
      const limit = (urdfJoint as unknown as { limit?: { lower: number; upper: number } }).limit
      this.jointMap.set(name, {
        urdfJoint,
        name,
        index: idx,
        lower: limit?.lower ?? -Math.PI,
        upper: limit?.upper ?? Math.PI,
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
    return b ? { lower: b.lower, upper: b.upper } : null
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
