import { useEffect, useRef, useState, useCallback } from 'react'
import * as THREE from 'three'
import URDFLoader from 'urdf-loader'
import { Viewport } from './three/Viewport'
import { RobotModel } from './three/RobotModel'
import { setViewport } from './three/viewportContext'
import { LeftPanel } from './components/LeftPanel'
import { TargetPanel } from './components/TargetPanel'
import { StatePanel } from './components/StatePanel'
import { OpsPanel } from './components/OpsPanel'
import { Timeline } from './components/Timeline'
import { PinnedBanner } from './components/PinnedBanner'
import { TerrainEditor } from './components/TerrainEditor'
import { useStore } from './state/store'
import { getRobotUrdfUrl, getAssetUrl } from './api/client'

function App() {
  const viewportRef = useRef<HTMLDivElement>(null)
  const viewportInstance = useRef<Viewport | null>(null)
  const [terrain, setTerrain] = useState<THREE.Group | null>(null)
  const [showTerrainEditor, setShowTerrainEditor] = useState(false)
  const [urdfRelPath, setUrdfRelPath] = useState<string | undefined>(undefined)
  const {
    trajectory, trajectoryVersion, currentFrame, pinnedLink,
    setRobotModelLoaded, setPinnedLink, setSegment,
  } = useStore()

  // Create the 3D viewport exactly once.
  useEffect(() => {
    if (!viewportRef.current) return
    const vp = new Viewport(viewportRef.current)
    viewportInstance.current = vp
    setViewport(vp)
    ;(window as unknown as { __viewport?: Viewport }).__viewport = vp
    return () => {
      setViewport(null)
      viewportInstance.current = null
      vp.dispose()
    }
  }, [])

  // (Re)load the robot only when the URDF source changes, not on trajectory load.
  useEffect(() => {
    let cancelled = false
    getRobotUrdfUrl(urdfRelPath).then(url => {
      const loader = new URDFLoader()
      loader.packages = (pkg: string) => getAssetUrl(pkg)
      loader.load(url,
        (robot) => {
          if (cancelled || !viewportInstance.current) return
          const model = new RobotModel(robot, useStore.getState().trajectory.jointNames)
          viewportInstance.current.setRobotModel(model)
          setRobotModelLoaded(true)
        },
        undefined,
        (err) => { console.error('[URDF] Load error:', err) },
      )
    })
    return () => { cancelled = true }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [urdfRelPath])

  // Rebind trajectory joint names to the loaded robot when the trajectory changes.
  useEffect(() => {
    viewportInstance.current?.robotModel?.bindJoints(trajectory.jointNames)
  }, [trajectory])

  // Aim the camera at the trajectory's first base position so the robot is in view.
  useEffect(() => {
    const vp = viewportInstance.current
    if (!vp || trajectory.frameCount === 0) return
    const frame = trajectory.getFrame(0)
    const [x, y, z] = [frame.basePoseW[0], frame.basePoseW[1], frame.basePoseW[2]]
    if (![x, y, z].every(Number.isFinite)) return
    vp.controls.target.set(x, y, z)
    vp.camera.position.set(x + 3, y - 3, z + 2)
    vp.controls.update()
  }, [trajectory])

  // Pose the robot from the current frame on any trajectory/frame change.
  useEffect(() => {
    const vp = viewportInstance.current
    if (!vp?.robotModel || trajectory.frameCount === 0) return
    const frame = trajectory.getFrame(currentFrame)
    vp.robotModel.applyFrame(frame.jointPos, frame.basePoseW, frame.baseQuatW)
  }, [trajectory, trajectoryVersion, currentFrame])

  // Reflect pin state in the robot model.
  useEffect(() => {
    const vp = viewportInstance.current
    if (!vp?.robotModel) return
    vp.robotModel.setPinned(pinnedLink)
  }, [pinnedLink])

  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      if (e.key !== 'Escape') return
      if (useStore.getState().pinnedLink) setPinnedLink(null)
      else setSegment(null, null)
    }
    window.addEventListener('keydown', handleKeyDown)
    return () => window.removeEventListener('keydown', handleKeyDown)
  }, [setPinnedLink, setSegment])

  const handleLoadTerrain = useCallback((group: THREE.Group) => {
    setTerrain(group)
    viewportInstance.current?.setTerrain(group)
  }, [])

  const handleTerrainEditorClose = useCallback((modified: THREE.Group | null) => {
    setShowTerrainEditor(false)
    if (modified) {
      setTerrain(modified)
      viewportInstance.current?.setTerrain(modified)
    }
  }, [])

  return (
    <div style={rootStyle}>
      <div style={mainRow}>
        <div style={leftPanelStyle}>
          <LeftPanel
            onLoadTerrain={handleLoadTerrain}
            terrain={terrain}
            onEditTerrain={() => setShowTerrainEditor(true)}
            onRobotUploaded={setUrdfRelPath}
          />
          <TargetPanel />
        </div>
        <div style={viewportWrapperStyle}>
          <PinnedBanner />
          <div ref={viewportRef} style={viewportStyle} />
        </div>
        <div style={rightPanelStyle}>
          <OpsPanel />
          <StatePanel />
        </div>
      </div>
      <div style={timelineStyle}>
        <Timeline />
      </div>
      {showTerrainEditor && (
        <TerrainEditor terrain={terrain} onClose={handleTerrainEditorClose} />
      )}
    </div>
  )
}

export default App

const rootStyle: React.CSSProperties = {
  width: '100vw',
  height: '100vh',
  display: 'flex',
  flexDirection: 'column',
  background: '#12121e',
  color: '#ccc',
  fontFamily: 'monospace',
}

const mainRow: React.CSSProperties = {
  display: 'flex',
  flex: 1,
  overflow: 'hidden',
}

const leftPanelStyle: React.CSSProperties = {
  width: '220px',
  background: '#1a1a2e',
  borderRight: '1px solid #333',
  overflowY: 'auto',
}

const viewportWrapperStyle: React.CSSProperties = {
  flex: 1,
  position: 'relative',
}

const viewportStyle: React.CSSProperties = {
  width: '100%',
  height: '100%',
  position: 'relative',
}

const rightPanelStyle: React.CSSProperties = {
  width: '280px',
  background: '#1a1a2e',
  borderLeft: '1px solid #333',
  display: 'flex',
  flexDirection: 'column',
  overflowY: 'auto',
}

const timelineStyle: React.CSSProperties = {
  height: '120px',
  background: '#1a1a2e',
  borderTop: '1px solid #333',
  flexShrink: 0,
}
