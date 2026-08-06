import { useStore } from '../state/store'
import { useCallback } from 'react'
import { getViewport } from '../three/viewportContext'

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
        return (
          <div key={i} style={{ display: 'flex', gap: '4px', alignItems: 'center' }}>
            <span style={{ flex: 1, minWidth: 60, fontSize: '11px', overflow: 'hidden', textOverflow: 'ellipsis', whiteSpace: 'nowrap' }}>{name}</span>
            <input
              type="range"
              min={lower}
              max={upper}
              step={(upper - lower) / 200 || 0.01}
              value={frame.jointPos[i]}
              onChange={e => handleJointChange(i, parseFloat(e.target.value))}
              style={{ flex: 1, minWidth: 60 }}
            />
            <input
              type="number"
              value={frame.jointPos[i]}
              onChange={e => handleJointChange(i, parseFloat(e.target.value))}
              style={{ width: '80px', fontSize: '11px' }}
              step={0.01}
            />
          </div>
        )
      })}
      <h4>Base Position (xyz)</h4>
      {['x', 'y', 'z'].map((label, i) => (
        <div key={i} style={{ display: 'flex', gap: '4px', alignItems: 'center' }}>
          <span style={{ width: '45px', fontSize: '11px' }}>{label}:</span>
          <input
            type="number"
            value={frame.basePoseW[i]}
            onChange={e => handleBasePosChange(i, parseFloat(e.target.value))}
            style={{ flex: 1, minWidth: 80, fontSize: '11px' }}
            step={0.01}
          />
        </div>
      ))}
      <h4>Base Orientation (roll/pitch/yaw)</h4>
      {(() => {
        const [roll, pitch, yaw] = trajectory.getQuatEuler(currentFrame)
        const euler = [roll, pitch, yaw]
        return ['roll', 'pitch', 'yaw'].map((label, i) => (
          <div key={i} style={{ display: 'flex', gap: '4px', alignItems: 'center' }}>
            <span style={{ width: '45px', fontSize: '11px' }}>{label}:</span>
            <input
              type="number"
              value={euler[i]}
              onChange={e => handleBaseQuatChange(i, parseFloat(e.target.value))}
              style={{ flex: 1, minWidth: 80, fontSize: '11px' }}
              step={0.01}
            />
          </div>
        ))
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
