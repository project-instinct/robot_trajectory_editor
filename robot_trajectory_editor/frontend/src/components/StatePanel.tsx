import { useStore } from '../state/store'
import { useCallback } from 'react'

export function StatePanel() {
  const { trajectory, currentFrame, setTrajectory } = useStore()
  const frame = trajectory.getFrame(currentFrame)

  const handleJointChange = useCallback((index: number, value: number) => {
    trajectory.setJointValue(currentFrame, index, value)
    setTrajectory(trajectory)
  }, [trajectory, currentFrame, setTrajectory])

  const handleBasePosChange = useCallback((axis: number, value: number) => {
    const newPos = new Float32Array(frame.basePoseW)
    newPos[axis] = value
    trajectory.setBasePose(currentFrame, newPos, frame.baseQuatW)
    setTrajectory(trajectory)
  }, [trajectory, currentFrame, setTrajectory, frame])

  const handleBaseQuatChange = useCallback((axis: number, value: number) => {
    const newQuat = new Float32Array(frame.baseQuatW)
    newQuat[axis] = value
    trajectory.setBasePose(currentFrame, frame.basePoseW, newQuat)
    setTrajectory(trajectory)
  }, [trajectory, currentFrame, setTrajectory, frame])

  return (
    <div style={panelStyle}>
      <h3>Robot State</h3>
      <p>Frame: {currentFrame} / {trajectory.frameCount - 1}</p>
      <h4>Joints</h4>
      {trajectory.jointNames.map((name, i) => (
        <div key={i} style={{ display: 'flex', gap: '4px', alignItems: 'center' }}>
          <span style={{ width: '60px', fontSize: '11px' }}>{name}</span>
          <input
            type="range"
            min={-Math.PI}
            max={Math.PI}
            step={0.01}
            value={frame.jointPos[i]}
            onChange={e => handleJointChange(i, parseFloat(e.target.value))}
            style={{ flex: 1 }}
          />
          <input
            type="number"
            value={frame.jointPos[i]}
            onChange={e => handleJointChange(i, parseFloat(e.target.value))}
            style={{ width: '60px', fontSize: '11px' }}
            step={0.01}
          />
        </div>
      ))}
      <h4>Base Position (xyz)</h4>
      {['x', 'y', 'z'].map((label, i) => (
        <div key={i} style={{ display: 'flex', gap: '4px', alignItems: 'center' }}>
          <span>{label}:</span>
          <input
            type="number"
            value={frame.basePoseW[i]}
            onChange={e => handleBasePosChange(i, parseFloat(e.target.value))}
            style={{ width: '80px', fontSize: '11px' }}
            step={0.01}
          />
        </div>
      ))}
      <h4>Base Orientation (wxyz)</h4>
      {['w', 'x', 'y', 'z'].map((label, i) => (
        <div key={i} style={{ display: 'flex', gap: '4px', alignItems: 'center' }}>
          <span>{label}:</span>
          <input
            type="number"
            value={frame.baseQuatW[i]}
            onChange={e => handleBaseQuatChange(i, parseFloat(e.target.value))}
            style={{ width: '80px', fontSize: '11px' }}
            step={0.01}
          />
        </div>
      ))}
    </div>
  )
}

const panelStyle: React.CSSProperties = {
  padding: '8px',
  overflowY: 'auto',
  flex: 1,
}
