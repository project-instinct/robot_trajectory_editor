import { useEffect, useRef, useState } from 'react'
import * as THREE from 'three'
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js'
import { getViewport } from '../three/viewportContext'
import type { CartesianLocalPoint } from '../three/CartesianInterpolation'

interface CartesianPointModalProps {
  linkName: string
  initialPoint: CartesianLocalPoint
  onCancel: () => void
  onConfirm: (point: CartesianLocalPoint) => void
}

const AXES = [
  { label: 'X', color: '#ff7777' },
  { label: 'Y', color: '#77dd88' },
  { label: 'Z', color: '#66aaff' },
] as const

function previewMaterial(material: THREE.Material): THREE.Material {
  const clone = material.clone()
  clone.transparent = true
  clone.opacity = 0.6
  clone.depthWrite = false
  return clone
}

export function CartesianPointModal({
  linkName,
  initialPoint,
  onCancel,
  onConfirm,
}: CartesianPointModalProps) {
  const hostRef = useRef<HTMLDivElement>(null)
  const pointMeshRef = useRef<THREE.Mesh | null>(null)
  const [point, setPoint] = useState<CartesianLocalPoint>(() => [
    initialPoint[0], initialPoint[1], initialPoint[2],
  ])
  const [sliderLimit, setSliderLimit] = useState(0.5)
  const [previewWarning, setPreviewWarning] = useState('')

  useEffect(() => {
    const host = hostRef.current
    if (!host) return

    const scene = new THREE.Scene()
    scene.background = new THREE.Color(0x171725)
    const camera = new THREE.PerspectiveCamera(38, 1, 0.001, 100)
    camera.up.set(0, 0, 1)
    const renderer = new THREE.WebGLRenderer({ antialias: true })
    renderer.setPixelRatio(window.devicePixelRatio)
    renderer.domElement.style.display = 'block'
    renderer.domElement.style.width = '100%'
    renderer.domElement.style.height = '100%'
    host.appendChild(renderer.domElement)

    const controls = new OrbitControls(camera, renderer.domElement)
    controls.enableDamping = true
    controls.dampingFactor = 0.1

    scene.add(new THREE.HemisphereLight(0xaaaacc, 0x303040, 2.5))
    const keyLight = new THREE.DirectionalLight(0xffffff, 3)
    keyLight.position.set(2, -3, 4)
    scene.add(keyLight)

    const visualGroup = new THREE.Group()
    const ownedMaterials: THREE.Material[] = []
    const robot = getViewport()?.robotModel
    const linkNode = robot?.linkNodes.get(linkName)
    if (robot && linkNode) {
      robot.refreshLinkMeshes()
      linkNode.updateWorldMatrix(true, true)
      const worldToLink = linkNode.matrixWorld.clone().invert()
      for (const source of robot.linkMeshes.get(linkName) ?? []) {
        source.updateWorldMatrix(true, false)
        const materials = Array.isArray(source.material)
          ? source.material.map(previewMaterial)
          : previewMaterial(source.material)
        ownedMaterials.push(...(Array.isArray(materials) ? materials : [materials]))
        const mesh = new THREE.Mesh(source.geometry, materials)
        mesh.matrix.copy(worldToLink).multiply(source.matrixWorld)
        mesh.matrixAutoUpdate = false
        visualGroup.add(mesh)
      }
    }
    scene.add(visualGroup)
    visualGroup.updateMatrixWorld(true)

    const bounds = new THREE.Box3().setFromObject(visualGroup)
    const center = bounds.isEmpty() ? new THREE.Vector3() : bounds.getCenter(new THREE.Vector3())
    const size = bounds.isEmpty() ? new THREE.Vector3(0.25, 0.25, 0.25) : bounds.getSize(new THREE.Vector3())
    const maxSize = Math.max(size.x, size.y, size.z, 0.1)
    const limit = Math.max(0.25, maxSize * 1.5, ...initialPoint.map(value => Math.abs(value) * 1.2))
    setSliderLimit(limit)

    const axes = new THREE.AxesHelper(Math.max(0.15, maxSize * 0.75))
    scene.add(axes)

    const pointGeometry = new THREE.SphereGeometry(Math.max(0.008, maxSize * 0.045), 24, 16)
    const pointMaterial = new THREE.MeshStandardMaterial({
      color: 0xffcc33,
      emissive: 0x664400,
      roughness: 0.35,
    })
    const pointMesh = new THREE.Mesh(pointGeometry, pointMaterial)
    pointMesh.position.set(...initialPoint)
    pointMeshRef.current = pointMesh
    scene.add(pointMesh)

    if (!robot || !linkNode) {
      setPreviewWarning(`Link "${linkName}" is not available in the loaded robot.`)
    } else if (visualGroup.children.length === 0) {
      setPreviewWarning('This link has no visual mesh; the point is shown against its local axes.')
    }

    const distance = Math.max(0.55, maxSize / (2 * Math.tan(THREE.MathUtils.degToRad(camera.fov / 2))) * 1.8)
    camera.position.copy(new THREE.Vector3(1, -1, 0.8).normalize().multiplyScalar(distance).add(center))
    controls.target.copy(center)
    controls.update()

    const resize = () => {
      const width = host.clientWidth || 640
      const height = host.clientHeight || 340
      camera.aspect = width / height
      camera.updateProjectionMatrix()
      renderer.setSize(width, height, false)
    }
    const resizeObserver = new ResizeObserver(resize)
    resizeObserver.observe(host)
    resize()

    let animationFrame = 0
    const render = () => {
      animationFrame = requestAnimationFrame(render)
      controls.update()
      renderer.render(scene, camera)
    }
    render()

    return () => {
      cancelAnimationFrame(animationFrame)
      resizeObserver.disconnect()
      controls.dispose()
      renderer.dispose()
      renderer.domElement.remove()
      axes.dispose()
      pointGeometry.dispose()
      pointMaterial.dispose()
      ownedMaterials.forEach(material => material.dispose())
      pointMeshRef.current = null
    }
  }, [initialPoint, linkName])

  useEffect(() => {
    pointMeshRef.current?.position.set(...point)
  }, [point])

  const changeAxis = (axis: 0 | 1 | 2, value: number) => {
    setPoint(current => {
      const next: CartesianLocalPoint = [current[0], current[1], current[2]]
      next[axis] = value
      return next
    })
  }

  return (
    <div
      style={overlayStyle}
      onClick={event => {
        event.stopPropagation()
        onCancel()
      }}
    >
      <div style={modalStyle} onClick={event => event.stopPropagation()}>
        <h3>Position Point: {linkName}</h3>
        <p style={helpStyle}>
          Choose the constrained point in this link&apos;s local frame. Drag to orbit the preview.
        </p>
        <div ref={hostRef} style={previewStyle} />
        {previewWarning && <div style={warningStyle}>{previewWarning}</div>}
        <div style={slidersStyle}>
          {AXES.map((axis, index) => (
            <label key={axis.label} style={sliderRowStyle}>
              <span style={{ ...axisLabelStyle, color: axis.color }}>{axis.label}</span>
              <input
                aria-label={`${axis.label} local position`}
                type="range"
                min={-sliderLimit}
                max={sliderLimit}
                step={sliderLimit / 200}
                value={point[index]}
                onChange={event => changeAxis(index as 0 | 1 | 2, Number(event.target.value))}
              />
              <span style={valueStyle}>{point[index].toFixed(3)} m</span>
            </label>
          ))}
        </div>
        <div style={actionsStyle}>
          <button onClick={onCancel}>Cancel</button>
          <button onClick={() => onConfirm(point)}>Use Point</button>
        </div>
      </div>
    </div>
  )
}

const overlayStyle: React.CSSProperties = {
  position: 'fixed', inset: 0, zIndex: 240,
  display: 'flex', alignItems: 'center', justifyContent: 'center',
  background: 'rgba(0, 0, 0, 0.78)',
}

const modalStyle: React.CSSProperties = {
  width: '680px', maxWidth: '94vw', maxHeight: '94vh', overflowY: 'auto', padding: '16px',
  background: '#1e1e2e', border: '1px solid #555', borderRadius: '8px',
}

const helpStyle: React.CSSProperties = {
  margin: '6px 0 10px', color: '#aaa', fontSize: '12px',
}

const previewStyle: React.CSSProperties = {
  position: 'relative', width: '100%', height: '340px', overflow: 'hidden',
  border: '1px solid #444', borderRadius: '5px',
}

const slidersStyle: React.CSSProperties = {
  display: 'flex', flexDirection: 'column', gap: '9px', marginTop: '12px',
}

const sliderRowStyle: React.CSSProperties = {
  display: 'grid', gridTemplateColumns: '20px 1fr 82px', alignItems: 'center', gap: '9px',
}

const axisLabelStyle: React.CSSProperties = {
  fontWeight: 'bold', textAlign: 'center',
}

const valueStyle: React.CSSProperties = {
  textAlign: 'right', fontVariantNumeric: 'tabular-nums', fontSize: '12px',
}

const warningStyle: React.CSSProperties = {
  marginTop: '8px', padding: '7px', color: '#f9c66d', background: '#392f20', borderRadius: '4px',
}

const actionsStyle: React.CSSProperties = {
  display: 'flex', justifyContent: 'flex-end', gap: '8px', marginTop: '14px',
}
