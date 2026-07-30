import { useEffect, useRef, useState, useCallback } from 'react'
import * as THREE from 'three'
import URDFLoader from 'urdf-loader'
import { Viewport } from './three/Viewport'
import { RobotModel } from './three/RobotModel'
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
  const { trajectory, currentFrame, pinnedLink, setRobotModelLoaded, setPinnedLink } = useStore()

  const loadRobotModel = useCallback((urdfPath?: string) => {
    const vp = viewportInstance.current
    if (!vp) return

    getRobotUrdfUrl(urdfPath)
      .then(url => {
        console.log('[URDF] Loading from:', url)
        const loader = new URDFLoader()
        loader.packages = (pkg: string) => getAssetUrl(pkg)
        loader.load(url,
          (robot) => {
            console.log('[URDF] Robot loaded, joints:', robot.joints?.length)
            const jointNames = trajectory.jointNames.length > 0 ? trajectory.jointNames : []
            const model = new RobotModel(robot, jointNames)
            vp.setRobotModel(model)
            setRobotModelLoaded(true)
          },
          undefined,
          (err) => {
            console.error('[URDF] Load error:', err)
          }
        )
      })
      .catch((err) => { console.error('[URDF] URL fetch error:', err) })
  }, [trajectory.jointNames, setRobotModelLoaded])

  useEffect(() => {
    if (!viewportRef.current) return
    const vp = new Viewport(viewportRef.current)
    viewportInstance.current = vp
    loadRobotModel()
    return () => vp.dispose()
  }, [loadRobotModel])

  useEffect(() => {
    const vp = viewportInstance.current
    if (!vp?.robotModel) return
    const frame = trajectory.getFrame(currentFrame)
    vp.robotModel.applyFrame(frame.jointPos, frame.basePoseW, frame.baseQuatW)
  }, [trajectory, currentFrame])

  useEffect(() => {
    const vp = viewportInstance.current
    if (!vp?.robotModel) return
    vp.robotModel.setPinned(pinnedLink)
  }, [pinnedLink])

  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      if (e.key === 'Escape') setPinnedLink(null)
    }
    window.addEventListener('keydown', handleKeyDown)
    return () => window.removeEventListener('keydown', handleKeyDown)
  }, [setPinnedLink])

  const handleLoadTerrain = useCallback((group: THREE.Group) => {
    setTerrain(group)
    viewportInstance.current?.setTerrain(group)
  }, [])

  return (
    <div style={rootStyle}>
      <div style={mainRow}>
        <div style={leftPanelStyle}>
          <LeftPanel onLoadTerrain={handleLoadTerrain} terrain={terrain} onEditTerrain={() => setShowTerrainEditor(true)} onRobotUploaded={loadRobotModel} />
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
        <TerrainEditor terrain={terrain} onClose={(modified) => { setTerrain(modified); setShowTerrainEditor(false) }} />
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
