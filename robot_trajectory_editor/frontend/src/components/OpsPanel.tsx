import { useStore } from '../state/store'
import { useCallback, useEffect, useRef, useState } from 'react'
import { CartesianInterpolateModal } from './CartesianInterpolateModal'
import { SegmentsModal } from './SegmentsModal'
import { runTrajectoryOptimize } from '../state/trajectoryOptimize'

export function OpsPanel({ urdfPath }: { urdfPath?: string }) {
  const { fillRange, smoothRange, interpolateRange, isPlaying, setIsPlaying, trajectory, robotModelLoaded } = useStore()
  const timerRef = useRef<number>(0)
  const [showCartesianInterpolate, setShowCartesianInterpolate] = useState(false)
  const [showSegments, setShowSegments] = useState(false)

  const [friction, setFriction] = useState(0.7)
  const [optimizing, setOptimizing] = useState(false)
  const [optimizeMessage, setOptimizeMessage] = useState('')
  const currentUrdfPath = useRef(urdfPath)
  currentUrdfPath.current = urdfPath

  const handleOptimize = async () => {
    setOptimizing(true)
    setOptimizeMessage('Optimizing the entire trajectory…')
    try {
      const result = await runTrajectoryOptimize(urdfPath, () => currentUrdfPath.current === urdfPath, friction)
      setOptimizeMessage(`Trajectory optimization complete. Validated ${result.validation_samples_per_interval} samples per interval. ` +
        Object.entries(result.residuals).map(([name, residual]) =>
          `${name}: ${residual.toExponential(2)} (tolerance ${result.tolerances[name]})`).join('; '))
    } catch (error) {
      setOptimizeMessage(error instanceof Error ? error.message : String(error))
    } finally {
      setOptimizing(false)
    }
  }

  const togglePlay = useCallback(() => {
    setIsPlaying(!isPlaying)
  }, [isPlaying, setIsPlaying])

  useEffect(() => {
    if (!isPlaying) return
    const interval = 1000 / trajectory.framerate
    timerRef.current = window.setInterval(() => {
      const store = useStore.getState()
      const next = store.currentFrame + 1
      if (next >= store.trajectory.frameCount) {
        store.setIsPlaying(false)
        store.setCurrentFrame(0)
      } else {
        store.setCurrentFrame(next)
      }
    }, interval)
    return () => window.clearInterval(timerRef.current)
  }, [isPlaying, trajectory.framerate])

  return (
    <div style={panelStyle}>
      <h3>Operations</h3>
      <button onClick={fillRange}>Fill</button>
      <button onClick={smoothRange}>Smooth</button>
      <button onClick={interpolateRange}>Interpolate</button>
      <button onClick={() => setShowCartesianInterpolate(true)}>Cartesian Interpolate</button>
      <button onClick={() => setShowSegments(true)}>Segments</button>
      <label style={{ fontSize: '12px' }}>
        Terrain friction μ
        <input aria-label="Terrain friction coefficient" type="number" min="0" step="0.1"
          value={friction} disabled={optimizing} style={{ width: '60px', marginLeft: '6px' }}
          onChange={event => setFriction(event.target.valueAsNumber)} />
      </label>
      <button disabled={optimizing || !Number.isFinite(friction) || friction < 0 || !robotModelLoaded || trajectory.frameCount < 2} onClick={handleOptimize}>
        {optimizing ? 'Optimizing…' : 'Trajectory Optimize'}
      </button>
      {optimizeMessage && <div role="status" style={{ fontSize: '11px', overflowWrap: 'anywhere' }}>{optimizeMessage}</div>}
      <button onClick={togglePlay}>{isPlaying ? 'Pause' : 'Play'}</button>
      {showCartesianInterpolate && (
        <CartesianInterpolateModal onClose={() => setShowCartesianInterpolate(false)} />
      )}
      {showSegments && (
        <SegmentsModal onClose={() => setShowSegments(false)} />
      )}
    </div>
  )
}

const panelStyle: React.CSSProperties = {
  padding: '8px',
  display: 'flex',
  flexDirection: 'column',
  gap: '6px',
  borderBottom: '1px solid #333',
}
