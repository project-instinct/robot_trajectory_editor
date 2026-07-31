import { useState, useEffect, useRef } from 'react'
import * as THREE from 'three'
import { TransformControls } from 'three/examples/jsm/controls/TransformControls.js'
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js'
import { cropTerrain, downsampleTerrain, bakeTerrainTransform } from '../three/Terrain'

interface TerrainEditorProps {
  terrain: THREE.Group | null
  /** Called with the edited terrain on Apply, or null on Cancel. */
  onClose: (modified: THREE.Group | null) => void
}

function cloneTerrainDeep(terrain: THREE.Group): THREE.Group {
  const clone = terrain.clone(true)
  clone.traverse((obj) => {
    if ((obj as THREE.Mesh).isMesh) {
      const mesh = obj as THREE.Mesh
      mesh.geometry = mesh.geometry.clone()
    }
  })
  return clone
}

export function TerrainEditor({ terrain, onClose }: TerrainEditorProps) {
  // Editable copy created once per editor open; gizmo edits apply directly to it.
  const [editable] = useState<THREE.Group | null>(() => (terrain ? cloneTerrainDeep(terrain) : null))
  const [bbox] = useState<THREE.Box3>(() => {
    const box = new THREE.Box3()
    if (editable) box.setFromObject(editable)
    return box
  })
  const [cropMin, setCropMin] = useState<[number, number, number]>([bbox.min.x, bbox.min.y, bbox.min.z])
  const [cropMax, setCropMax] = useState<[number, number, number]>([bbox.max.x, bbox.max.y, bbox.max.z])
  const [ratio, setRatio] = useState(0.5)

  const handleCrop = () => {
    if (!editable) return
    cropTerrain(editable, new THREE.Box3(
      new THREE.Vector3(...cropMin),
      new THREE.Vector3(...cropMax),
    ))
  }

  const handleDownsample = () => {
    if (!editable) return
    downsampleTerrain(editable, ratio)
  }

  const handleApply = () => {
    // Bake gizmo move/rotate into geometry so the exported/committed mesh is self-contained.
    if (editable) bakeTerrainTransform(editable)
    onClose(editable)
  }

  const numInput = (value: number, onChange: (v: number) => void) => (
    <input
      type="number"
      step={0.1}
      value={value}
      onChange={(e) => onChange(parseFloat(e.target.value) || 0)}
      style={{ width: '70px', fontSize: '11px' }}
    />
  )

  return (
    <div style={overlayStyle}>
      <div style={modalStyle}>
        <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '8px' }}>
          <h3 style={{ margin: 0 }}>Terrain Editor</h3>
          <div style={{ display: 'flex', gap: '6px' }}>
            <button onClick={handleApply}>Apply</button>
            <button onClick={() => onClose(null)}>Cancel</button>
          </div>
        </div>
        <TerrainViewport terrain={editable} />
        <div style={{ display: 'flex', gap: '16px', flexWrap: 'wrap', padding: '8px 0', fontSize: '12px', color: '#aaa' }}>
          <div>
            <div>Crop min (x y z):</div>
            <div style={{ display: 'flex', gap: '4px' }}>
              {cropMin.map((v, i) => (
                <span key={i}>{numInput(v, (nv) => setCropMin(prev => { const n = [...prev] as typeof prev; n[i] = nv; return n }))}</span>
              ))}
            </div>
            <div>Crop max (x y z):</div>
            <div style={{ display: 'flex', gap: '4px' }}>
              {cropMax.map((v, i) => (
                <span key={i}>{numInput(v, (nv) => setCropMax(prev => { const n = [...prev] as typeof prev; n[i] = nv; return n }))}</span>
              ))}
            </div>
            <button style={{ marginTop: '4px' }} onClick={handleCrop}>Crop</button>
          </div>
          <div>
            <div>Downsample ratio ({Math.round(ratio * 100)}% removed):</div>
            <input
              type="range"
              min={0.05}
              max={0.9}
              step={0.05}
              value={ratio}
              onChange={(e) => setRatio(parseFloat(e.target.value))}
            />
            <button onClick={handleDownsample}>Downsample</button>
          </div>
          <div style={{ color: '#888' }}>
            Drag gizmo to move/rotate terrain (T = translate, R = rotate). Right-drag to orbit.
          </div>
        </div>
      </div>
    </div>
  )
}

function TerrainViewport({ terrain }: { terrain: THREE.Group | null }) {
  const mountRef = useRef<HTMLDivElement>(null)

  useEffect(() => {
    const el = mountRef.current
    if (!el) return

    const scene = new THREE.Scene()
    scene.background = new THREE.Color(0x1a1a2e)

    const camera = new THREE.PerspectiveCamera(50, el.clientWidth / el.clientHeight, 0.01, 1000)
    camera.up.set(0, 0, 1)
    camera.position.set(5, -5, 5)
    camera.lookAt(0, 0, 0)

    const renderer = new THREE.WebGLRenderer({ antialias: true })
    renderer.setSize(el.clientWidth, el.clientHeight)
    el.appendChild(renderer.domElement)

    const orbitControls = new OrbitControls(camera, renderer.domElement)
    orbitControls.target.set(0, 0, 0)

    const ambient = new THREE.AmbientLight(0x404060, 2)
    scene.add(ambient)
    const dir = new THREE.DirectionalLight(0xffffff, 2)
    dir.position.set(5, 5, 10)
    scene.add(dir)

    // Z-up grid (GridHelper is XZ by default; rotate into XY plane).
    const grid = new THREE.GridHelper(10, 10, 0x444466, 0x222244)
    grid.rotation.x = Math.PI / 2
    scene.add(grid)

    let transformControls: TransformControls | null = null
    if (terrain) {
      scene.add(terrain)
      transformControls = new TransformControls(camera, renderer.domElement)
      transformControls.attach(terrain)
      scene.add(transformControls as unknown as THREE.Object3D)
      transformControls.addEventListener('dragging-changed', (event) => {
        orbitControls.enabled = !(event.target as TransformControls).dragging
      })
    }

    const onKeyDown = (e: KeyboardEvent) => {
      if (!transformControls) return
      if (e.key === 't' || e.key === 'T') transformControls.setMode('translate')
      if (e.key === 'r' || e.key === 'R') transformControls.setMode('rotate')
    }
    window.addEventListener('keydown', onKeyDown)

    let running = true
    function animate() {
      if (!running) return
      requestAnimationFrame(animate)
      orbitControls.update()
      renderer.render(scene, camera)
    }
    animate()

    const onResize = () => {
      camera.aspect = el.clientWidth / el.clientHeight
      camera.updateProjectionMatrix()
      renderer.setSize(el.clientWidth, el.clientHeight)
    }
    window.addEventListener('resize', onResize)

    return () => {
      running = false
      window.removeEventListener('resize', onResize)
      window.removeEventListener('keydown', onKeyDown)
      orbitControls.dispose()
      transformControls?.dispose()
      renderer.dispose()
      el.removeChild(renderer.domElement)
    }
  }, [terrain])

  return <div ref={mountRef} style={{ width: '100%', height: '400px' }} />
}

const overlayStyle: React.CSSProperties = {
  position: 'fixed', top: 0, left: 0, right: 0, bottom: 0,
  background: 'rgba(0,0,0,0.7)', display: 'flex', alignItems: 'center', justifyContent: 'center', zIndex: 200,
}
const modalStyle: React.CSSProperties = {
  background: '#1e1e2e', border: '1px solid #444', borderRadius: '8px',
  width: '80vw', maxWidth: '900px', padding: '16px',
}
