import { useEffect, useRef, useState, useCallback } from 'react'
import * as THREE from 'three'
import URDFLoader from 'urdf-loader'
import { Viewport } from './three/Viewport'
import { RobotModel } from './three/RobotModel'
import { setViewport } from './three/viewportContext'
import { LeftPanel } from './components/LeftPanel'
import { TargetPanel } from './components/TargetPanel'
import { TransformPanel } from './components/TransformPanel'
import { StatePanel } from './components/StatePanel'
import { OpsPanel } from './components/OpsPanel'
import { Timeline } from './components/Timeline'
import { PinnedBanner } from './components/PinnedBanner'
import { TerrainEditor } from './components/TerrainEditor'
import { useStore } from './state/store'
import { getRobotUrdfUrl, getAssetUrl, getConfig, saveTrajectoryFile } from './api/client'

function App() {
  const viewportRef = useRef<HTMLDivElement>(null)
  const viewportInstance = useRef<Viewport | null>(null)
  const [terrain, setTerrain] = useState<THREE.Group | null>(null)
  const [showTerrainEditor, setShowTerrainEditor] = useState(false)
  const [urdfRelPath, setUrdfRelPath] = useState<string | undefined>(undefined)
  const [leftWidth, setLeftWidth] = useState(280)
  const [rightWidth, setRightWidth] = useState(280)
  const resizeRef = useRef<{ side: 'left' | 'right'; startX: number; startWidth: number } | null>(null)
  const {
    trajectory, trajectoryVersion, currentFrame, pinnedLinks,
    setRobotModelLoaded, clearPinnedLinks, setSegment, setNudgeStep, setFrameStep, undo,
  } = useStore()

  // Drag-resize the left/right panels by their vertical divider handles.
  const handleResizeMove = useCallback((e: PointerEvent) => {
    const drag = resizeRef.current
    if (!drag) return
    // The left divider sits to the right of its panel; the right divider sits to
    // its left, so the sign flips so both handles move the boundary under the
    // cursor (drag right = left panel wider, right panel narrower).
    const delta = drag.side === 'left' ? e.clientX - drag.startX : drag.startX - e.clientX
    const width = drag.startWidth + delta
    if (drag.side === 'left') setLeftWidth(Math.min(420, Math.max(200, width)))
    else setRightWidth(Math.min(520, Math.max(200, width)))
  }, [])

  const endResize = useCallback(() => {
    resizeRef.current = null
    document.body.style.userSelect = ''
    window.removeEventListener('pointermove', handleResizeMove)
    window.removeEventListener('pointerup', endResize)
  }, [handleResizeMove])

  const startResize = useCallback((side: 'left' | 'right') => (e: React.PointerEvent<HTMLDivElement>) => {
    e.preventDefault()
    document.body.style.userSelect = 'none'
    resizeRef.current = { side, startX: e.clientX, startWidth: side === 'left' ? leftWidth : rightWidth }
    window.addEventListener('pointermove', handleResizeMove)
    window.addEventListener('pointerup', endResize)
  }, [leftWidth, rightWidth, handleResizeMove, endResize])

  // Pull launch-time config (e.g. arrow-key nudge/frame steps) from the backend.
  useEffect(() => {
    getConfig()
      .then(cfg => {
        if (typeof cfg.nudge_step === 'number') setNudgeStep(cfg.nudge_step)
        if (typeof cfg.frame_step === 'number') setFrameStep(cfg.frame_step)
      })
      .catch(() => { /* keep the default steps when the backend has no config endpoint */ })
  }, [setNudgeStep, setFrameStep])

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
    vp.robotModel.setPinned(pinnedLinks)
  }, [pinnedLinks])

  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      if (e.key === 'Escape') {
        if (viewportInstance.current?.dragController.clearSelectedLink()) return
        if (useStore.getState().pinnedLinks.length > 0) clearPinnedLinks()
        else setSegment(null, null)
        return
      }
      if ((e.ctrlKey || e.metaKey) && (e.code === 'KeyS' || e.key.toLowerCase() === 's')) {
        e.preventDefault()
        if (!e.repeat) {
          void saveTrajectoryFile(useStore.getState().trajectory.toJSON())
            .catch(error => console.error('Failed to save trajectory:', error))
        }
        return
      }
      // Timeline stepping always takes precedence over a focused form control.
      // In particular, prevent range inputs from consuming Left/Right.
      if (!e.ctrlKey && !e.metaKey && !e.altKey && (
        e.key === 'ArrowLeft' || e.key === 'ArrowRight'
      )) {
        const s = useStore.getState()
        if (s.trajectory.frameCount > 0) {
          e.preventDefault()
          s.stepFrame(e.key === 'ArrowRight' ? 1 : -1)
          return
        }
      }
      // A select keeps focus after choosing an Edit Target, so handle target
      // cycling before the general form-control guard. Do not intercept comma
      // or decimal entry in text/number inputs.
      const keyTarget = e.target as HTMLElement | null
      const enteringText = keyTarget && (
        ['INPUT', 'TEXTAREA'].includes(keyTarget.tagName) || keyTarget.isContentEditable
      )
      if (!enteringText && !e.ctrlKey && !e.metaKey && !e.altKey && (
        e.key === ',' || e.key === '.'
      )) {
        const s = useStore.getState()
        if (s.selectedChannel) {
          e.preventDefault()
          s.cycleSelectedChannel(e.key === '.' ? 1 : -1)
          return
        }
      }
      // Let form controls keep their other native key behavior (typing, Up/Down).
      const target = e.target as HTMLElement | null
      const editingText = target && (
        ['INPUT', 'SELECT', 'TEXTAREA'].includes(target.tagName) || target.isContentEditable
      )
      if (editingText) return
      if ((e.ctrlKey || e.metaKey) && (e.code === 'KeyZ' || e.key.toLowerCase() === 'z')) {
        e.preventDefault()
        useStore.getState().undo()
        return
      }
      if (e.ctrlKey || e.metaKey || e.altKey) return
      const s = useStore.getState()
      if (e.key === 'ArrowUp' || e.key === 'ArrowDown') {
        if (!s.selectedChannel || s.trajectory.frameCount === 0) return
        e.preventDefault()
        s.nudgeSelectedChannel(e.key === 'ArrowUp' ? 1 : -1)
      } else if (e.key === ' ') {
        if (s.trajectory.frameCount === 0) return
        e.preventDefault()
        s.setIsPlaying(!s.isPlaying)
      } else if (e.code === 'KeyI') {
        if (s.trajectory.frameCount === 0) return
        e.preventDefault()
        s.setSegmentStart(s.currentFrame)
      } else if (e.code === 'KeyO') {
        if (s.trajectory.frameCount === 0) return
        e.preventDefault()
        s.setSegmentEnd(s.currentFrame)
      } else if (e.code === 'KeyU') {
        e.preventDefault()
        s.setSegment(null, null)
      }
    }
    window.addEventListener('keydown', handleKeyDown, true)
    return () => window.removeEventListener('keydown', handleKeyDown, true)
  }, [clearPinnedLinks, setSegment, undo])

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
        <div style={{ ...leftPanelStyle, width: leftWidth }}>
          <LeftPanel
            onLoadTerrain={handleLoadTerrain}
            terrain={terrain}
            onEditTerrain={() => setShowTerrainEditor(true)}
            onRobotUploaded={setUrdfRelPath}
          />
          <TargetPanel />
          <TransformPanel />
          <OpsPanel />
        </div>
        <div style={handleStyle} onPointerDown={startResize('left')} />
        <div style={viewportWrapperStyle}>
          <PinnedBanner />
          <div ref={viewportRef} style={viewportStyle} />
        </div>
        <div style={handleStyle} onPointerDown={startResize('right')} />
        <div style={{ ...rightPanelStyle, width: rightWidth }}>
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
  background: '#1a1a2e',
  borderRight: '1px solid #333',
  overflowX: 'hidden',
  overflowY: 'auto',
  scrollbarGutter: 'stable',
  flexShrink: 0,
}

const handleStyle: React.CSSProperties = {
  width: '5px',
  flexShrink: 0,
  cursor: 'col-resize',
  background: '#333',
  touchAction: 'none',
}

const viewportWrapperStyle: React.CSSProperties = {
  flex: 1,
  position: 'relative',
  minWidth: 0,
}

const viewportStyle: React.CSSProperties = {
  width: '100%',
  height: '100%',
  position: 'relative',
}

const rightPanelStyle: React.CSSProperties = {
  background: '#1a1a2e',
  borderLeft: '1px solid #333',
  display: 'flex',
  flexDirection: 'column',
  flexShrink: 0,
}

const timelineStyle: React.CSSProperties = {
  height: '120px',
  background: '#1a1a2e',
  borderTop: '1px solid #333',
  flexShrink: 0,
}
