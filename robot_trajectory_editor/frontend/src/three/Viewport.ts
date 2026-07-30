import * as THREE from 'three'
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js'
import { RobotModel } from './RobotModel'
import { DragController } from './DragController'


export class Viewport {
  scene: THREE.Scene
  camera: THREE.PerspectiveCamera
  renderer: THREE.WebGLRenderer
  controls: OrbitControls
  robotModel: RobotModel | null = null
  dragController: DragController
  terrainGroup: THREE.Group
  groundPlane: THREE.Mesh | null = null
  grid: THREE.GridHelper
  private _running = true
  private _container: HTMLElement
  private _resizeObserver: ResizeObserver

  constructor(container: HTMLElement) {
    this._container = container

    this.scene = new THREE.Scene()
    this.scene.background = new THREE.Color(0x2a2a4e)

    this.camera = new THREE.PerspectiveCamera(50, 2, 0.01, 100)
    this.camera.position.set(2, 2, 2)
    this.camera.lookAt(0, 0, 1)

    this.renderer = new THREE.WebGLRenderer({ antialias: true })
    this.renderer.setSize(container.clientWidth || 800, container.clientHeight || 600)
    this.renderer.setPixelRatio(window.devicePixelRatio)
    this.renderer.shadowMap.enabled = true
    this.renderer.domElement.style.display = 'block'
    this.renderer.domElement.style.position = 'absolute'
    this.renderer.domElement.style.top = '0'
    this.renderer.domElement.style.left = '0'
    this.renderer.domElement.style.width = '100%'
    this.renderer.domElement.style.height = '100%'
    for (const child of Array.from(container.children)) {
      if (child.tagName === 'CANVAS') child.remove()
    }
    container.appendChild(this.renderer.domElement)

    this.controls = new OrbitControls(this.camera, this.renderer.domElement)
    this.controls.target.set(0, 0, 1)
    this.controls.enableDamping = true
    this.controls.dampingFactor = 0.1
    this.controls.update()

    this.terrainGroup = new THREE.Group()
    this.scene.add(this.terrainGroup)

    this.addLighting()
    this.addGroundPlane()
    this.grid = new THREE.GridHelper(10, 10, 0x555577, 0x333355)
    this.grid.position.z = 0.001
    this.scene.add(this.grid)

    this.dragController = new DragController(this)

    this._resizeObserver = new ResizeObserver(() => this._handleResize())
    this._resizeObserver.observe(container)

    requestAnimationFrame(() => this._handleResize())
    this._start()
  }

  private _start(): void {
    const loop = () => {
      if (!this._running) return
      requestAnimationFrame(loop)
      try {
        this.controls.update()
        this.dragController.update()
        this.renderer.render(this.scene, this.camera)
      } catch (_e) {
        // ignore render errors
      }
    }
    requestAnimationFrame(loop)
  }

  private addLighting(): void {
    const ambient = new THREE.AmbientLight(0x606080, 3)
    this.scene.add(ambient)
    const dir = new THREE.DirectionalLight(0xffffff, 3)
    dir.position.set(5, 10, 5)
    dir.castShadow = true
    dir.shadow.mapSize.set(1024, 1024)
    this.scene.add(dir)
    const hemi = new THREE.HemisphereLight(0x8080aa, 0x404060, 1.5)
    this.scene.add(hemi)
  }

  private addGroundPlane(): void {
    const geom = new THREE.PlaneGeometry(10, 10)
    const mat = new THREE.MeshStandardMaterial({ color: 0x445566, side: THREE.DoubleSide })
    this.groundPlane = new THREE.Mesh(geom, mat)
    this.groundPlane.rotation.x = -Math.PI / 2
    this.groundPlane.receiveShadow = true
    this.scene.add(this.groundPlane)
  }

  setTerrain(group: THREE.Group | null): void {
    while (this.terrainGroup.children.length) this.terrainGroup.remove(this.terrainGroup.children[0])
    if (this.groundPlane) this.groundPlane.visible = !group
    if (this.grid) this.grid.visible = !group
    if (group) this.terrainGroup.add(group)
  }

  setRobotModel(model: RobotModel): void {
    this.robotModel = model
    this.scene.add(model.rootGroup)
    this.dragController.setRobot(model)
  }

  private _handleResize(): void {
    const w = this._container.clientWidth
    const h = this._container.clientHeight
    if (w === 0 || h === 0) return
    this.camera.aspect = w / h
    this.camera.updateProjectionMatrix()
    this.renderer.setSize(w, h)
  }

  dispose(): void {
    this._running = false
    this._resizeObserver.disconnect()
    this.renderer.domElement.remove()
    this.renderer.dispose()
    this.controls.dispose()
    this.dragController.dispose()
  }
}
