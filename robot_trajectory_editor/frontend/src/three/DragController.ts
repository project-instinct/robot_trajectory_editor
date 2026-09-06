import * as THREE from 'three'
import { TransformControls } from 'three/examples/jsm/controls/TransformControls.js'
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
 * - pinned mode (double-click links): every pinned link is held at its pin-time
 *   world pose; dragging moves/rotates the base and IK re-solves the joints.
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

  // TransformControls exposes translation and rotation separately. Attaching
  // both to one world-space proxy produces the requested six-DoF link gizmo
  // without directly re-parenting or mutating the URDF scene graph.
  private gizmoTarget = new THREE.Object3D()
  private translateGizmo: TransformControls
  private rotateGizmo: TransformControls
  private translateGizmoHelper: THREE.Object3D
  private rotateGizmoHelper: THREE.Object3D
  private selectedLinkName = ''
  private gizmoEditStarted = false

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
  private pointerDown = new THREE.Vector2()
  private pointerMoved = false
  private pointerDownLinkName = ''
  private selectionTimer: ReturnType<typeof setTimeout> | null = null

  private pinnedPoses = new Map<string, { pos: THREE.Vector3; quat: THREE.Quaternion }>()

  constructor(viewport: Viewport) {
    this.viewport = viewport
    const dom = viewport.renderer.domElement

    this.gizmoTarget.name = '__selected_link_gizmo_target__'
    viewport.scene.add(this.gizmoTarget)
    this.translateGizmo = new TransformControls(viewport.camera, dom)
    this.rotateGizmo = new TransformControls(viewport.camera, dom)
    this.translateGizmo.setMode('translate')
    this.rotateGizmo.setMode('rotate')
    this.translateGizmo.setSize(0.8)
    this.rotateGizmo.setSize(0.8)
    this.translateGizmoHelper = this.translateGizmo.getHelper()
    this.rotateGizmoHelper = this.rotateGizmo.getHelper()
    viewport.scene.add(this.translateGizmoHelper)
    viewport.scene.add(this.rotateGizmoHelper)
    this.translateGizmo.addEventListener('dragging-changed', this.onGizmoDraggingChanged)
    this.rotateGizmo.addEventListener('dragging-changed', this.onGizmoDraggingChanged)
    this.translateGizmo.addEventListener('objectChange', this.onGizmoObjectChange)
    this.rotateGizmo.addEventListener('objectChange', this.onGizmoObjectChange)

    dom.addEventListener('pointerdown', this.onPointerDown, true)
    dom.addEventListener('pointermove', this.onPointerMove, true)
    dom.addEventListener('pointerup', this.onPointerUp, true)
    dom.addEventListener('dblclick', this.onDoubleClick)
  }

  setRobot(robot: RobotModel): void {
    this.selectLink('')
    this.robot = robot
    this.ikSolver = new IkSolver(robot.urdfRobot)
  }

  private gizmoDragging(): boolean {
    return this.translateGizmo.dragging || this.rotateGizmo.dragging
  }

  private selectLink(linkName: string): void {
    if (linkName === this.selectedLinkName) return
    this.translateGizmo.detach()
    this.rotateGizmo.detach()
    this.selectedLinkName = linkName
    if (!linkName || !this.robot) return
    const pose = this.robot.getLinkWorldPose(linkName)
    if (!pose) {
      this.selectedLinkName = ''
      return
    }
    this.gizmoTarget.position.copy(pose.pos)
    this.gizmoTarget.quaternion.copy(pose.quat)
    this.gizmoTarget.updateMatrixWorld(true)
    this.translateGizmo.attach(this.gizmoTarget)
    this.rotateGizmo.attach(this.gizmoTarget)
  }

  /** Clears the current link selection. Returns true when a gizmo was removed. */
  clearSelectedLink(): boolean {
    const hadSelection = Boolean(this.selectedLinkName)
    const hadPendingSelection = this.selectionTimer !== null
    if (this.selectionTimer !== null) {
      clearTimeout(this.selectionTimer)
      this.selectionTimer = null
    }
    if (hadSelection) this.selectLink('')
    return hadSelection || hadPendingSelection
  }

  /** Keeps a selected link's gizmo aligned through frame changes and direct drags. */
  private syncSelectedLinkPose(): void {
    if (!this.selectedLinkName || !this.robot || this.gizmoDragging()) return
    const pose = this.robot.getLinkWorldPose(this.selectedLinkName)
    if (!pose) return
    this.gizmoTarget.position.copy(pose.pos)
    this.gizmoTarget.quaternion.copy(pose.quat)
    this.gizmoTarget.updateMatrixWorld(true)
  }

  private onGizmoDraggingChanged = (event: THREE.Event & { target: TransformControls }): void => {
    const active = event.target
    if (active.dragging) {
      // The controls share a canvas and proxy. Whichever handle starts first
      // owns the gesture so an overlapping picker cannot rotate and translate
      // the target from the same pointer movement.
      if (active === this.translateGizmo) this.rotateGizmo.enabled = false
      else this.translateGizmo.enabled = false
    }
    const dragging = this.gizmoDragging()
    this.viewport.controls.enabled = !dragging
    if (dragging && !this.gizmoEditStarted) {
      useStore.getState().beginTrajectoryEdit()
      this.gizmoEditStarted = true
    } else if (!dragging) {
      this.translateGizmo.enabled = true
      this.rotateGizmo.enabled = true
      this.gizmoEditStarted = false
      this.syncSelectedLinkPose()
    }
  }

  private onGizmoObjectChange = (): void => {
    if (!this.gizmoDragging() || !this.robot || !this.ikSolver || !this.selectedLinkName) return
    const store = useStore.getState()
    const { trajectory, currentFrame, pinnedLinks } = store
    if (trajectory.frameCount === 0) return

    const target = {
      pos: this.gizmoTarget.position.clone(),
      quat: this.gizmoTarget.quaternion.clone(),
    }
    let { pos: basePos, quat: baseQuat } = this.frameBase()

    if (this.selectedLinkName === this.robot.rootLinkName) {
      // The IK solver intentionally locks the floating base. For the root link,
      // apply the gizmo's world-space delta to the trajectory base pose first.
      // This also handles a non-identity transform between rootGroup and the
      // URDF root link instead of assuming their origins are identical.
      const current = this.robot.getLinkWorldPose(this.selectedLinkName)
      if (!current) return
      const currentLink = new THREE.Matrix4().compose(current.pos, current.quat, new THREE.Vector3(1, 1, 1))
      const targetLink = new THREE.Matrix4().compose(target.pos, target.quat, new THREE.Vector3(1, 1, 1))
      const baseMatrix = new THREE.Matrix4().compose(basePos, baseQuat, new THREE.Vector3(1, 1, 1))
      const solvedBase = targetLink.multiply(currentLink.invert()).multiply(baseMatrix)
      basePos = new THREE.Vector3()
      baseQuat = new THREE.Quaternion()
      solvedBase.decompose(basePos, baseQuat, new THREE.Vector3())
      trajectory.setBasePose(
        currentFrame,
        [basePos.x, basePos.y, basePos.z],
        [baseQuat.w, baseQuat.x, baseQuat.y, baseQuat.z],
      )
    } else if (!this.ikSolver.hasLink(this.selectedLinkName)) {
      return
    }

    this.ikSolver.setConfiguration(basePos, baseQuat, this.frameJointMap())

    // A gizmo edit and fixed-link mode remain compatible: all existing pin
    // poses are solved together with the selected link's requested pose. When
    // the selected link is itself pinned, deliberately moving its gizmo moves
    // that fixed world anchor to the new target.
    const goals = new Map<string, { pos: THREE.Vector3; quat: THREE.Quaternion }>()
    for (const linkName of pinnedLinks) {
      const pose = this.pinnedPoses.get(linkName)
      if (pose && linkName !== this.selectedLinkName && this.ikSolver.hasLink(linkName)) {
        goals.set(linkName, pose)
      }
    }
    if (this.selectedLinkName !== this.robot.rootLinkName) {
      goals.set(this.selectedLinkName, target)
    }
    if (goals.size > 0) {
      this.ikSolver.solvePoseGoals(goals)
      this.writeSolvedJoints(this.ikSolver.getJointValues())
    }
    if (pinnedLinks.includes(this.selectedLinkName)) {
      this.pinnedPoses.set(this.selectedLinkName, {
        pos: target.pos.clone(),
        quat: target.quat.clone(),
      })
    }

    store.touchTrajectory()
    const frame = trajectory.getFrame(currentFrame)
    this.robot.applyFrame(frame.jointPos, frame.basePoseW, frame.baseQuatW)
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
    // Reserve direct touch gestures for OrbitControls: one finger rotates and
    // two fingers pan/zoom. Mouse and pen input still perform robot edits.
    if (event.pointerType === 'touch') return
    if (event.button !== 0 || !this.robot) return
    // A second click may be the start of a double-click. Cancel the first
    // click's deferred selection so pinning never leaves a gizmo behind.
    if (this.selectionTimer !== null) {
      clearTimeout(this.selectionTimer)
      this.selectionTimer = null
      this.selectLink('')
    }
    // Let either transform-control layer own pointer gestures over its handles.
    if (this.translateGizmo.axis || this.rotateGizmo.axis) return
    const store = useStore.getState()
    if (store.trajectory.frameCount === 0) return

    const hit = this.raycastRobot(event)
    if (!hit) return

    this.pointerDown.set(event.clientX, event.clientY)
    this.pointerMoved = false
    this.pointerDownLinkName = findLinkAncestor(hit.object)?.name ?? ''

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

    if (store.pinnedLinks.length > 0) {
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
    if (Math.hypot(event.clientX - this.pointerDown.x, event.clientY - this.pointerDown.y) > 3) {
      this.pointerMoved = true
    }
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
    if (!this.pointerMoved && this.pointerDownLinkName) {
      const linkName = this.pointerDownLinkName
      this.selectionTimer = setTimeout(() => {
        this.selectionTimer = null
        this.selectLink(this.selectedLinkName === linkName ? '' : linkName)
      }, 300)
    }
    this.mode = 'none'
    this.pendingPoint = null
    this.gizmoEditStarted = false
    this.viewport.controls.enabled = true
  }

  private onDoubleClick = (event: MouseEvent): void => {
    if (!this.robot) return
    if (this.selectionTimer !== null) {
      clearTimeout(this.selectionTimer)
      this.selectionTimer = null
    }
    this.selectLink('')
    const hit = this.raycastRobot(event)
    if (!hit) return
    const linkName = findLinkAncestor(hit.object)?.name
    if (!linkName) return

    const store = useStore.getState()
    if (store.pinnedLinks.includes(linkName)) {
      this.pinnedPoses.delete(linkName)
      store.togglePinnedLink(linkName)
      this.robot.setPinned(store.pinnedLinks.filter(name => name !== linkName))
    } else {
      const pose = this.robot.getLinkWorldPose(linkName)
      if (!pose) return
      this.pinnedPoses.set(linkName, { pos: pose.pos.clone(), quat: pose.quat.clone() })
      store.togglePinnedLink(linkName)
      this.robot.setPinned([...store.pinnedLinks, linkName])
    }
    this.ikSolver?.clearGoals()
  }

  /** Called every frame from the viewport render loop; applies pending drags. */
  update(): void {
    this.syncSelectedLinkPose()
    if (!this.dirty || !this.pendingPoint || !this.robot) return
    this.dirty = false

    const store = useStore.getState()
    const { trajectory, currentFrame, pinnedLinks } = store
    if (trajectory.frameCount === 0) return

    // Delay undo capture until a click has actually become a drag. A plain
    // selection click must not create an empty undo entry.
    if (!this.pointerMoved) return
    if (!this.gizmoEditStarted) {
      store.beginTrajectoryEdit()
      this.gizmoEditStarted = true
    }

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

      if (pinnedLinks.length > 0 && this.ikSolver) {
        this.ikSolver.setConfiguration(newPos, newQuat, this.frameJointMap())
        const goals = new Map<string, { pos: THREE.Vector3; quat: THREE.Quaternion }>()
        for (const linkName of pinnedLinks) {
          const pose = this.pinnedPoses.get(linkName)
          if (pose && this.ikSolver.hasLink(linkName)) goals.set(linkName, pose)
        }
        this.ikSolver.solvePoseGoals(goals)
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
    if (this.selectionTimer !== null) clearTimeout(this.selectionTimer)
    this.translateGizmo.removeEventListener('dragging-changed', this.onGizmoDraggingChanged)
    this.rotateGizmo.removeEventListener('dragging-changed', this.onGizmoDraggingChanged)
    this.translateGizmo.removeEventListener('objectChange', this.onGizmoObjectChange)
    this.rotateGizmo.removeEventListener('objectChange', this.onGizmoObjectChange)
    this.translateGizmo.detach()
    this.rotateGizmo.detach()
    this.translateGizmo.dispose()
    this.rotateGizmo.dispose()
    this.viewport.scene.remove(this.translateGizmoHelper)
    this.viewport.scene.remove(this.rotateGizmoHelper)
    this.viewport.scene.remove(this.gizmoTarget)
  }
}
