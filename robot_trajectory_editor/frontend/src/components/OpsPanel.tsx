import { useStore } from '../state/store'
import { useCallback, useEffect, useRef, useState } from 'react'
import { CartesianInterpolateModal } from './CartesianInterpolateModal'
import { SegmentsModal } from './SegmentsModal'
import { TrajectoryOptimizeModal } from './TrajectoryOptimizeModal'
import { interpolatedFrame, playbackPosition } from '../state/playback'
import { getViewport } from '../three/viewportContext'

export function OpsPanel({ urdfPath }: { urdfPath?: string }) {
  const { fillRange, smoothRange, interpolateRange, isPlaying, setIsPlaying, trajectory, robotModelLoaded } = useStore()
  const timerRef = useRef<number>(0)
  const [showCartesianInterpolate, setShowCartesianInterpolate] = useState(false)
  const [showSegments, setShowSegments] = useState(false)

  const [showOptimize, setShowOptimize] = useState(false)

  const togglePlay = useCallback(() => {
    setIsPlaying(!isPlaying)
  }, [isPlaying, setIsPlaying])

  // Playback follows the wall clock at the loaded trajectory's own frame rate,
  // whatever the display refresh rate, and draws the pose between frames.
  useEffect(() => {
    if (!isPlaying) return
    const playing = useStore.getState().trajectory
    const startFrame = useStore.getState().currentFrame
    const start = startFrame >= playing.frameCount - 1 ? 0 : startFrame
    const startedAt = performance.now()
    const tick = (now: number) => {
      const store = useStore.getState()
      const position = playbackPosition(start, (now - startedAt) / 1000, playing.framerate)
      if (position >= playing.frameCount - 1) {
        store.setIsPlaying(false)
        store.setCurrentFrame(0)
        return
      }
      const frame = Math.floor(position)
      if (frame !== store.currentFrame) store.setCurrentFrame(frame)
      const robot = getViewport()?.robotModel
      if (robot) {
        const pose = interpolatedFrame(playing, position)
        robot.applyFrame(pose.jointPos, pose.basePoseW, pose.baseQuatW)
      }
      timerRef.current = requestAnimationFrame(tick)
    }
    timerRef.current = requestAnimationFrame(tick)
    return () => cancelAnimationFrame(timerRef.current)
  }, [isPlaying, trajectory])

  return (
    <div style={panelStyle}>
      <h3>Operations</h3>
      <button onClick={fillRange}>Fill</button>
      <button onClick={smoothRange}>Smooth</button>
      <button onClick={interpolateRange}>Interpolate</button>
      <button onClick={() => setShowCartesianInterpolate(true)}>Cartesian Interpolate</button>
      <button onClick={() => setShowSegments(true)}>Segments</button>
      <button disabled={!robotModelLoaded || trajectory.frameCount < 2} onClick={() => setShowOptimize(true)}>
        Trajectory Optimize
      </button>
      <button onClick={togglePlay}>{isPlaying ? 'Pause' : 'Play'}</button>
      {showCartesianInterpolate && (
        <CartesianInterpolateModal onClose={() => setShowCartesianInterpolate(false)} />
      )}
      {showSegments && (
        <SegmentsModal onClose={() => setShowSegments(false)} />
      )}
      {showOptimize && (
        <TrajectoryOptimizeModal urdfPath={urdfPath} onClose={() => setShowOptimize(false)} />
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
