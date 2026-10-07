import { useStore } from '../state/store'
import { useCallback, useEffect, useRef, useState } from 'react'
import { CartesianInterpolateModal } from './CartesianInterpolateModal'
import { SegmentsModal } from './SegmentsModal'

export function OpsPanel() {
  const { fillRange, smoothRange, interpolateRange, isPlaying, setIsPlaying, trajectory } = useStore()
  const timerRef = useRef<number>(0)
  const [showCartesianInterpolate, setShowCartesianInterpolate] = useState(false)
  const [showSegments, setShowSegments] = useState(false)

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
