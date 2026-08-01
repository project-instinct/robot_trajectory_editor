import * as THREE from 'three'
import type { Viewport } from './Viewport'
import { RobotModel, findLinkAncestor } from './RobotModel'
import { IkSolver } from './IkSolver'
import { useStore } from '../state/store'

type DragMode = 'none' | 'base' | 'link'

const Z_AXIS = new THREE.Vector3(0, 0, 1)

/**
 * Viewport drag editing:
 * - default mode: dragging a body link IK-drags that link (base stays);
 *   dragging the base (root) link translates the base (Shift+drag = yaw).
 * - pinned mode (double-click a link): the link is held at its pin-time world
 *   pose; dragging moves/rotates the base and IK re-solves the joints so the
 *   pinned link stays fixed.
 * All edits are written to the trajectory at the current frame.
 *
 * Pointer handlers run in the capture phase and stop propagation when a drag
 * starts so OrbitControls does not also react.
 */
export class DragController {
  private viewport: Viewport
  private raycaster = new THREE.Raycaster()
  private mouse = new THREE.Vector2()
  private robot: RobotModel | null = null
  private ikSolver: IkSolver | null = null

  private mode: DragMode = 'none'
  private dragLinkName = ''
  private dragPlane = new THREE.Plane()
  private dragStartPoint = new THREE.Vector3()
  private grabOffset = new THREE.Vector3()
  private startBasePos = new THREE.Vector3()
  private startBaseQuat = new THREE.Quaternion()
  private pendingPoint: THREE.Vector3 | null = null
  private pendingRotate = false
  private dirty = false

  private pinnedPos = new THREE.Vector3()
  private pinnedQuat = new THREE.Quaternion()

  constructor(viewport: Viewport) {
    this.viewport = viewport
    const dom = viewport.renderer.domElement
    dom.addEventListener('pointerdown', this.onPointerDown, true)
    dom.addEventListener('pointermove', this.onPointerMove, true)
    dom.addEventListener('pointerup', this.onPointerUp, true)
    dom.addEventListener('dblclick', this.onDoubleClick)
  }

  setRobot(robot: RobotModel): void {
    this.robot = robot
    this.ikSolver = new IkSolver(robot.urdfRobot)
  }

  private setPointer(event: MouseEvent): void {
    const rect = this.viewport.renderer.domElement.getBoundingClientRect()
    this.mouse.x = ((event.clientX - rect.left) / rect.width) * 2 - 1
    this.mouse.y = -((event.clientY - rect.top) / rect.height) * 2 + 1
  }

  private raycastRobot(event: MouseEvent): THREE.Intersection | null {
    if (!this.robot) return null
    this.setPointer(event)
    this.raycaster.setFromCamera(this.mouse, this.viewport.camera)
    const hits = this.raycaster.intersectObject(this.robot.rootGroup, true)
    return hits.length > 0 ? hits[0] : null
  }

  private frameBase(): { pos: THREE.Vector3; quat: THREE.Quaternion } {
    const { trajectory, currentFrame } = useStore.getState()
    const frame = trajectory.getFrame(currentFrame)
    const pos = new THREE.Vector3(frame.basePoseW[0], frame.basePoseW[1], frame.basePoseW[2])
    const quat = new THREE.Quaternion(frame.baseQuatW[1], frame.baseQuatW[2], frame.baseQuatW[3], frame.baseQuatW[0])
    return { pos, quat }
  }

  private frameJointMap(): Map<string, number> {
    const { trajectory, currentFrame } = useStore.getState()
    const frame = trajectory.getFrame(currentFrame)
    const map = new Map<string, number>()
    if (!this.robot) return map
    for (const [name, info] of this.robot.jointMap) {
      map.set(name, frame.jointPos[info.index] ?? 0)
    }
    return map
  }

  private writeSolvedJoints(solved: Map<string, number>): void {
    const { trajectory, currentFrame } = useStore.getState()
    if (!this.robot) return
    for (const [name, value] of solved) {
      const info = this.robot.jointMap.get(name)
      if (info) trajectory.setJointValue(currentFrame, info.index, value)
    }
  }

  private onPointerDown = (event: PointerEvent): void => {
    if (event.button !== 0 || !this.robot) return
    const store = useStore.getState()
    if (store.trajectory.frameCount === 0) return

    const hit = this.raycastRobot(event)
    if (!hit) return

    useStore.getState().beginTrajectoryEdit()

    // A drag starts here: keep OrbitControls out of this gesture.
    event.stopPropagation()
    this.viewport.controls.enabled = false

    this.dragStartPoint.copy(hit.point)
    this.dragPlane.setFromNormalAndCoplanarPoint(
      this.viewport.camera.getWorldDirection(new THREE.Vector3()).negate(),
      hit.point,
    )

    const { pos, quat } = this.frameBase()
    this.startBasePos.copy(pos)
    this.startBaseQuat.copy(quat)
    this.grabOffset.copy(pos).sub(hit.point)

    if (store.pinnedLink) {
      // Pinned mode: any robot drag moves the base; IK restores the pinned link.
      this.mode = 'base'
      this.dragLinkName = ''
    } else {
      const linkName = findLinkAncestor(hit.object)?.name ?? ''
      if (linkName && linkName !== this.robot.rootLinkName) {
        this.mode = 'link'
        this.dragLinkName = linkName
      } else {
        this.mode = 'base'
        this.dragLinkName = ''
      }
    }
  }

  private onPointerMove = (event: PointerEvent): void => {
    if (this.mode === 'none') return
    event.stopPropagation()
    this.setPointer(event)
    this.raycaster.setFromCamera(this.mouse, this.viewport.camera)
    const intersect = new THREE.Vector3()
    if (this.raycaster.ray.intersectPlane(this.dragPlane, intersect)) {
      this.pendingPoint = intersect
      this.pendingRotate = event.shiftKey
      this.dirty = true
    }
  }

  private onPointerUp = (event: PointerEvent): void => {
    if (this.mode === 'none') return
    event.stopPropagation()
    this.mode = 'none'
    this.pendingPoint = null
    this.viewport.controls.enabled = true
  }

  private onDoubleClick = (event: MouseEvent): void => {
    if (!this.robot) return
    const hit = this.raycastRobot(event)
    if (!hit) return
    const linkName = findLinkAncestor(hit.object)?.name
    if (!linkName) return

    const store = useStore.getState()
    if (store.pinnedLink === linkName) {
      store.setPinnedLink(null)
      this.robot.setPinned(null)
      this.ikSolver?.clearGoal()
    } else {
      const pose = this.robot.getLinkWorldPose(linkName)
      if (!pose) return
      this.ikSolver?.clearGoal()
      this.pinnedPos.copy(pose.pos)
      this.pinnedQuat.copy(pose.quat)
      store.setPinnedLink(linkName)
      this.robot.setPinned(linkName)
    }
  }

  /** Called every frame from the viewport render loop; applies pending drags. */
  update(): void {
    if (!this.dirty || !this.pendingPoint || !this.robot) return
    this.dirty = false

    const store = useStore.getState()
    const { trajectory, currentFrame, pinnedLink } = store
    if (trajectory.frameCount === 0) return

    if (this.mode === 'base') {
      let newPos: THREE.Vector3
      let newQuat: THREE.Quaternion
      if (this.pendingRotate) {
        // Yaw around world Z through the base origin, following the grab point.
        const a0 = Math.atan2(this.dragStartPoint.y - this.startBasePos.y, this.dragStartPoint.x - this.startBasePos.x)
        const a1 = Math.atan2(this.pendingPoint.y - this.startBasePos.y, this.pendingPoint.x - this.startBasePos.x)
        newPos = this.startBasePos.clone()
        newQuat = new THREE.Quaternion().setFromAxisAngle(Z_AXIS, a1 - a0).multiply(this.startBaseQuat)
      } else {
        newPos = this.pendingPoint.clone().add(this.grabOffset)
        newQuat = this.startBaseQuat.clone()
      }
      trajectory.setBasePose(currentFrame, [newPos.x, newPos.y, newPos.z], [newQuat.w, newQuat.x, newQuat.y, newQuat.z])

      if (pinnedLink && this.ikSolver?.hasLink(pinnedLink)) {
        this.ikSolver.setConfiguration(newPos, newQuat, this.frameJointMap())
        if (this.ikSolver.getChainActuatedDoF(pinnedLink) >= 6) {
          this.ikSolver.solvePoseGoal(pinnedLink, this.pinnedPos, this.pinnedQuat)
        } else {
          this.ikSolver.solvePositionGoal(pinnedLink, this.pinnedPos)
        }
        this.writeSolvedJoints(this.ikSolver.getJointValues())
      }
    } else if (this.mode === 'link' && this.ikSolver) {
      if (this.ikSolver.hasLink(this.dragLinkName)) {
        const { pos, quat } = this.frameBase()
        this.ikSolver.setConfiguration(pos, quat, this.frameJointMap())
        this.ikSolver.solvePositionGoal(this.dragLinkName, this.pendingPoint)
        this.writeSolvedJoints(this.ikSolver.getJointValues())
      }
    }

    store.touchTrajectory()
    const frame = trajectory.getFrame(currentFrame)
    this.robot.applyFrame(frame.jointPos, frame.basePoseW, frame.baseQuatW)
  }

  dispose(): void {
    const dom = this.viewport.renderer.domElement
    dom.removeEventListener('pointerdown', this.onPointerDown, true)
    dom.removeEventListener('pointermove', this.onPointerMove, true)
    dom.removeEventListener('pointerup', this.onPointerUp, true)
    dom.removeEventListener('dblclick', this.onDoubleClick)
  }
}
