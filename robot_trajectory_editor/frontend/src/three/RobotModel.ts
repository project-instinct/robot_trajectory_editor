import * as THREE from 'three'
import { type URDFRobot } from 'urdf-loader'
import type { IkSolver } from './IkSolver'
import { CcdIkSolver } from './IkSolver'

export class RobotModel {
  urdfRobot: URDFRobot
  rootGroup: THREE.Group
  jointMap: Map<string, { urdfJoint: THREE.Object3D; name: string; index: number }> = new Map()
  linkMeshes: Map<string, THREE.Mesh[]> = new Map()
  pinnedLink: string | null = null
  pinnedHighlight: THREE.MeshStandardMaterial[] = []
  ikSolver: IkSolver = new CcdIkSolver()

  constructor(urdfRobot: URDFRobot, jointNames: string[]) {
    this.urdfRobot = urdfRobot
    this.rootGroup = new THREE.Group()
    this.rootGroup.add(urdfRobot)

    jointNames.forEach((name, idx) => {
      const joint = this.findJoint(urdfRobot, name)
      if (joint) this.jointMap.set(name, { urdfJoint: joint, name, index: idx })
    })

    this.collectLinkMeshes(urdfRobot)
  }

  private findJoint(root: THREE.Object3D, name: string): THREE.Object3D | null {
    if (root.name === name && root.type === 'URDFJoint') return root
    for (const child of root.children) {
      const found = this.findJoint(child, name)
      if (found) return found
    }
    return null
  }

  private collectLinkMeshes(root: THREE.Object3D): void {
    root.traverse((obj) => {
      if ((obj as THREE.Mesh).isMesh) {
        const mesh = obj as THREE.Mesh
        const linkName = mesh.userData?.linkName || ''
        if (!this.linkMeshes.has(linkName)) this.linkMeshes.set(linkName, [])
        this.linkMeshes.get(linkName)!.push(mesh)
      }
    })
  }

  applyFrame(jointValues: Float32Array, basePos: Float32Array, baseQuat: Float32Array): void {
    this.rootGroup.position.set(basePos[0], basePos[1], basePos[2])
    this.rootGroup.quaternion.set(baseQuat[1], baseQuat[2], baseQuat[3], baseQuat[0])

    for (const [, info] of this.jointMap) {
      const val = jointValues[info.index]
      if (val !== undefined) {
        info.urdfJoint.rotation.set(0, 0, 0)
        info.urdfJoint.rotateOnWorldAxis(new THREE.Vector3(0, 0, 1), val)
      }
    }

    this.urdfRobot.updateMatrixWorld()
  }

  setPinned(linkName: string | null): void {
    if (this.pinnedLink === linkName) return
    this.pinnedLink = linkName

    for (const material of this.pinnedHighlight) {
      material.emissive?.set(0x000000)
    }
    this.pinnedHighlight = []

    if (linkName) {
      const meshes = this.linkMeshes.get(linkName)
      if (meshes) {
        for (const mesh of meshes) {
          const mat = mesh.material as THREE.MeshStandardMaterial
          if (mat.emissive) {
            const cloned = mat.clone()
            cloned.emissive = new THREE.Color(0x444400)
            mat.emissive = new THREE.Color(0x444400)
            this.pinnedHighlight.push(cloned)
          }
          mesh.material = mesh.material
        }
      }
    } else {
      this.pinnedHighlight = []
    }
  }

  getJointValues(): number[] {
    const vals: number[] = []
    for (const [_, info] of this.jointMap) {
      const euler = new THREE.Euler().setFromQuaternion(info.urdfJoint.quaternion, 'XYZ')
      vals.push(euler.z)
    }
    return vals
  }

  solveIkForPinned(_linkName: string, targetWorld: THREE.Matrix4): number[] {
    return this.ikSolver.solve(
      { jointNames: [], jointTypes: [], jointOrigins: [], endEffectorOffset: new THREE.Matrix4() },
      targetWorld,
      this.getJointValues(),
    )
  }

  physicsUpdate(_dt: number): void {}
}
