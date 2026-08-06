import { useStore } from '../state/store'
import { useCallback } from 'react'
import { getViewport } from '../three/viewportContext'

/** Natural euler ranges (radians) matching quatToEuler: roll/yaw wrap to ±π,
 *  pitch is clamped to ±π/2. */
const EULER_RANGES: [number, number][] = [
  [-Math.PI, Math.PI],
  [-Math.PI / 2, Math.PI / 2],
  [-Math.PI, Math.PI],
]

export function StatePanel() {
  const { trajectory, currentFrame, beginTrajectoryEdit, touchTrajectory, robotModelLoaded } = useStore()
  const frame = trajectory.getFrame(currentFrame)
  // Reading limits imperatively; robotModelLoaded subscription triggers re-render on robot load.
  void robotModelLoaded
  const robotModel = getViewport()?.robotModel ?? null

  const handleJointChange = useCallback((index: number, value: number) => {
    if (isNaN(value)) return
    beginTrajectoryEdit()
    trajectory.setJointValue(currentFrame, index, value)
    touchTrajectory()
  }, [trajectory, currentFrame, beginTrajectoryEdit, touchTrajectory])

  const handleBasePosChange = useCallback((axis: number, value: number) => {
    if (isNaN(value)) return
    beginTrajectoryEdit()
    const newPos = new Float32Array(frame.basePoseW)
    newPos[axis] = value
    trajectory.setBasePose(currentFrame, newPos, frame.baseQuatW)
    touchTrajectory()
  }, [trajectory, currentFrame, beginTrajectoryEdit, touchTrajectory, frame])

  const handleBaseQuatChange = useCallback((axis: number, value: number) => {
    if (isNaN(value)) return
    beginTrajectoryEdit()
    const [roll, pitch, yaw] = trajectory.getQuatEuler(currentFrame)
    const r = axis === 0 ? value : roll
    const p = axis === 1 ? value : pitch
    const ya = axis === 2 ? value : yaw
    trajectory.setQuatEuler(currentFrame, r, p, ya)
    touchTrajectory()
  }, [trajectory, currentFrame, beginTrajectoryEdit, touchTrajectory])

  /** Slider range covering the whole base-position sequence on an axis, so the
   *  current frame's value is always reachable with room to adjust. */
  const basePosRange = (axis: number): [number, number] => {
    let lo = Infinity
    let hi = -Infinity
    for (let f = 0; f < trajectory.frameCount; f++) {
      const v = trajectory.basePoseW[f * 3 + axis]
      if (v < lo) lo = v
      if (v > hi) hi = v
    }
    if (!isFinite(lo)) return [-5, 5]
    if (lo === hi) { lo -= 1; hi += 1 }
    const margin = (hi - lo) * 0.2
    return [lo - margin, hi + margin]
  }

  return (
    <div style={panelStyle}>
      <h3>Robot State</h3>
      {trajectory.frameCount === 0 ? (
        <p style={{ color: '#888', fontSize: '12px' }}>No trajectory loaded.</p>
      ) : (
      <>
      <p>Frame: {currentFrame} / {Math.max(trajectory.frameCount - 1, 0)}</p>
      <h4>Joints</h4>
      {trajectory.jointNames.map((name, i) => {
        const limit = robotModel?.getJointLimit(name)
        const lower = limit?.lower ?? -Math.PI
        const upper = limit?.upper ?? Math.PI
        const step = (upper - lower) / 200 || 0.01
        return (
          <div key={i} style={rowStyle}>
            <span style={jointNameStyle}>{name}</span>
            <input
              type="range"
              min={lower}
              max={upper}
              step={step}
              value={frame.jointPos[i]}
              onChange={e => handleJointChange(i, parseFloat(e.target.value))}
              style={sliderStyle}
            />
            <input
              type="number"
              value={frame.jointPos[i]}
              onChange={e => handleJointChange(i, parseFloat(e.target.value))}
              style={jointValueStyle}
              step={0.01}
            />
          </div>
        )
      })}
      <h4>Base Position (xyz)</h4>
      {['x', 'y', 'z'].map((label, i) => {
        const [lo, hi] = basePosRange(i)
        return (
          <div key={i} style={rowStyle}>
            <span style={nameStyle}>{label}:</span>
            <input
              type="range"
              min={lo}
              max={hi}
              step={0.01}
              value={frame.basePoseW[i]}
              onChange={e => handleBasePosChange(i, parseFloat(e.target.value))}
              style={sliderStyle}
            />
            <input
              type="number"
              value={frame.basePoseW[i]}
              onChange={e => handleBasePosChange(i, parseFloat(e.target.value))}
              style={flexValueStyle}
              step={0.01}
            />
          </div>
        )
      })}
      <h4>Base Orientation (roll/pitch/yaw)</h4>
      {(() => {
        const [roll, pitch, yaw] = trajectory.getQuatEuler(currentFrame)
        const euler = [roll, pitch, yaw]
        return ['roll', 'pitch', 'yaw'].map((label, i) => {
          const [lo, hi] = EULER_RANGES[i]
          return (
            <div key={i} style={rowStyle}>
              <span style={nameStyle}>{label}:</span>
              <input
                type="range"
                min={lo}
                max={hi}
                step={0.01}
                value={euler[i]}
                onChange={e => handleBaseQuatChange(i, parseFloat(e.target.value))}
                style={sliderStyle}
              />
              <input
                type="number"
                value={euler[i]}
                onChange={e => handleBaseQuatChange(i, parseFloat(e.target.value))}
                style={flexValueStyle}
                step={0.01}
              />
            </div>
          )
        })
      })()}
      </>
      )}
    </div>
  )
}

const panelStyle: React.CSSProperties = {
  padding: '8px',
  overflowX: 'hidden',
  overflowY: 'auto',
  scrollbarGutter: 'stable',
  flex: 1,
}

const rowStyle: React.CSSProperties = {
  display: 'flex',
  gap: '4px',
  alignItems: 'center',
}

/** Joint name grows with the panel and wraps, so the entire name stays visible. */
const jointNameStyle: React.CSSProperties = {
  flex: 1,
  minWidth: 60,
  fontSize: '11px',
  wordBreak: 'break-word',
}

const nameStyle: React.CSSProperties = {
  width: '45px',
  fontSize: '11px',
  flexShrink: 0,
}

const sliderStyle: React.CSSProperties = {
  flex: 1,
  minWidth: 60,
}

const jointValueStyle: React.CSSProperties = {
  width: '90px',
  fontSize: '11px',
  flexShrink: 0,
}

const flexValueStyle: React.CSSProperties = {
  flex: 1,
  minWidth: 80,
  fontSize: '11px',
}
