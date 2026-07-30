import * as THREE from 'three'
import type { Viewport } from './Viewport'
import type { RobotModel } from './RobotModel'
import { useStore } from '../state/store'
import { CcdIkSolver } from './IkSolver'
import type { IkChain } from './IkSolver'

export class DragController {
  viewport: Viewport
  raycaster: THREE.Raycaster
  mouse: THREE.Vector2
  isDragging = false
  dragTarget: THREE.Object3D | null = null
  dragPlane: THREE.Plane
  dragStartPoint: THREE.Vector3
  pinnedWorldTarget: THREE.Vector3 | null = null
  ikSolver = new CcdIkSolver()
  private _robot: RobotModel | null = null

  constructor(viewport: Viewport) {
    this.viewport = viewport
    this.raycaster = new THREE.Raycaster()
    this.mouse = new THREE.Vector2()
    this.dragPlane = new THREE.Plane()
    this.dragStartPoint = new THREE.Vector3()

    const dom = viewport.renderer.domElement
    dom.addEventListener('pointerdown', this.onPointerDown.bind(this))
    dom.addEventListener('pointermove', this.onPointerMove.bind(this))
    dom.addEventListener('pointerup', this.onPointerUp.bind(this))
    dom.addEventListener('dblclick', this.onDoubleClick.bind(this) as EventListener)
  }

  setRobot(robot: RobotModel): void { this._robot = robot }

  private getPointerPosition(event: MouseEvent): void {
    const rect = this.viewport.renderer.domElement.getBoundingClientRect()
    this.mouse.x = ((event.clientX - rect.left) / rect.width) * 2 - 1
    this.mouse.y = -((event.clientY - rect.top) / rect.height) * 2 + 1
  }

  private getRobotMeshes(): THREE.Object3D[] {
    const meshes: THREE.Object3D[] = []
    this._robot?.rootGroup.traverse((obj) => {
      if ((obj as THREE.Mesh).isMesh) meshes.push(obj)
    })
    return meshes
  }

  private onPointerDown(event: PointerEvent): void {
    if (event.button !== 0) return
    const robot = this._robot
    if (!robot) return

    const store = useStore.getState()
    if (store.trajectory.frameCount === 0) return

    this.getPointerPosition(event)
    this.raycaster.setFromCamera(this.mouse, this.viewport.camera)

    const intersects = this.raycaster.intersectObjects(this.getRobotMeshes(), false)
    if (intersects.length === 0) return

    this.isDragging = true
    this.dragTarget = intersects[0].object
    this.dragStartPoint.copy(intersects[0].point)
    this.dragPlane.setFromNormalAndCoplanarPoint(
      this.viewport.camera.getWorldDirection(new THREE.Vector3()).negate(),
      intersects[0].point,
    )

    if (store.pinnedLink && this._robot) {
      const linkMeshes = this._robot.linkMeshes.get(store.pinnedLink)
      if (linkMeshes && linkMeshes.length > 0) {
        const worldPos = new THREE.Vector3()
        linkMeshes[0].getWorldPosition(worldPos)
        this.pinnedWorldTarget = worldPos.clone()
      }
    }
  }

  private onPointerMove(event: PointerEvent): void {
    if (!this.isDragging || !this.dragTarget) return
    this.getPointerPosition(event)
    this.raycaster.setFromCamera(this.mouse, this.viewport.camera)

    const intersect = new THREE.Vector3()
    this.raycaster.ray.intersectPlane(this.dragPlane, intersect)
    if (!intersect) return

    const store = useStore.getState()
    const { trajectory, currentFrame, pinnedLink } = store

    if (trajectory.frameCount === 0) return

    if (pinnedLink && this._robot && this.pinnedWorldTarget) {
      const delta = intersect.clone().sub(this.dragStartPoint)
      const newPos = this._robot.rootGroup.position.clone().add(delta)
      this._robot.rootGroup.position.copy(newPos)

      try {
        const ikChain = this.buildIkChain(pinnedLink)
        if (ikChain.jointNames.length > 0) {
          const targetMatrix = new THREE.Matrix4().makeTranslation(
            this.pinnedWorldTarget.x - newPos.x,
            this.pinnedWorldTarget.y - newPos.y,
            this.pinnedWorldTarget.z - newPos.z,
          )
          const currentAngles = this._robot.getJointValues()
          const solvedAngles = this.ikSolver.solve(ikChain, targetMatrix, currentAngles)

          for (let i = 0; i < Math.min(solvedAngles.length, trajectory.jointCount); i++) {
            trajectory.setJointValue(currentFrame, i, solvedAngles[i])
          }
          store.setTrajectory(trajectory)
          const current = trajectory.getFrame(currentFrame)
          this._robot.applyFrame(current.jointPos, current.basePoseW, current.baseQuatW)
        }
      } catch (_e) {
        // IK may fail; just use direct positioning
      }
    } else {
      const current = trajectory.getFrame(currentFrame)
      const newPos = new Float32Array([intersect.x, intersect.y, intersect.z])
      const newQuat = new Float32Array(current.baseQuatW)
      trajectory.setBasePose(currentFrame, newPos, newQuat)
      store.setTrajectory(trajectory)
      if (this._robot) {
        this._robot.applyFrame(current.jointPos, newPos, newQuat)
      }
    }
  }

  private buildIkChain(_linkName: string): IkChain {
    const chain: IkChain = {
      jointNames: [],
      jointTypes: [],
      jointOrigins: [],
      endEffectorOffset: new THREE.Matrix4(),
    }

    if (!this._robot) return chain

    const jointNames: string[] = []
    const jointTypes: string[] = []
    const jointOrigins: THREE.Matrix4[] = []

    for (const [name] of this._robot.jointMap) {
      jointNames.push(name)
      jointTypes.push('revolute')
      jointOrigins.push(new THREE.Matrix4().identity())
    }

    return { jointNames, jointTypes, jointOrigins, endEffectorOffset: new THREE.Matrix4() }
  }

  private onPointerUp(): void {
    this.isDragging = false
    this.dragTarget = null
    this.pinnedWorldTarget = null
  }

  private onDoubleClick(event: MouseEvent): void {
    const robot = this._robot
    if (!robot) return
    this.getPointerPosition(event)
    this.raycaster.setFromCamera(this.mouse, this.viewport.camera)

    const intersects = this.raycaster.intersectObjects(this.getRobotMeshes(), false)
    if (intersects.length === 0) return

    const linkName = intersects[0].object.userData?.linkName || ''
    const store = useStore.getState()
    if (store.pinnedLink === linkName) {
      store.setPinnedLink(null)
      robot.setPinned(null)
    } else {
      store.setPinnedLink(linkName)
      robot.setPinned(linkName)
    }
  }

  update(): void {}

  dispose(): void {
    const dom = this.viewport.renderer.domElement
    dom.removeEventListener('pointerdown', this.onPointerDown.bind(this))
    dom.removeEventListener('pointermove', this.onPointerMove.bind(this))
    dom.removeEventListener('pointerup', this.onPointerUp.bind(this))
    dom.removeEventListener('dblclick', this.onDoubleClick.bind(this) as EventListener)
  }
}
