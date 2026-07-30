import { useState, useEffect, useRef } from 'react'
import * as THREE from 'three'
import { TransformControls } from 'three/examples/jsm/controls/TransformControls.js'
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js'

interface TerrainEditorProps {
  terrain: THREE.Group | null
  onClose: (modified: THREE.Group | null) => void
}

export function TerrainEditor({ terrain, onClose }: TerrainEditorProps) {
  const [isOpen, setIsOpen] = useState(true)
  const [terrainRef] = useState<{ current: THREE.Group | null }>(() => ({ current: terrain ? terrain.clone() : null }))

  if (!isOpen) {
    return null
  }

  return (
    <div style={overlayStyle}>
      <div style={modalStyle}>
        <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '8px' }}>
          <h3 style={{ margin: 0 }}>Terrain Editor</h3>
          <div style={{ display: 'flex', gap: '6px' }}>
            <button onClick={() => { setIsOpen(false); onClose(terrainRef.current) }}>Apply</button>
            <button onClick={() => { setIsOpen(false); onClose(terrain) }}>Cancel</button>
          </div>
        </div>
        <TerrainViewport terrain={terrainRef.current} />
        <div style={{ padding: '8px 0', display: 'flex', gap: '8px', fontSize: '12px', color: '#888' }}>
          Use gizmo to move/rotate terrain. Right-click to orbit.
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

    const grid = new THREE.GridHelper(10, 10, 0x444466, 0x222244)
    scene.add(grid)

    let controlsTarget: THREE.Group | null = null
    let transformControls: TransformControls | null = null

    if (terrain) {
      controlsTarget = terrain.clone()
      scene.add(controlsTarget)

      transformControls = new TransformControls(camera, renderer.domElement)
      transformControls.attach(controlsTarget)
      scene.add(transformControls as unknown as THREE.Object3D)

      transformControls.addEventListener('dragging-changed', (event) => {
        orbitControls.enabled = !(event.target as TransformControls).dragging
      })
    }

    function animate() {
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
      window.removeEventListener('resize', onResize)
      orbitControls.dispose()
      if (transformControls) { transformControls.dispose() }
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
