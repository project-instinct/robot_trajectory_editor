import { useStore } from '../state/store'

const basePosLabels = ['x', 'y', 'z']
const baseQuatLabels = ['roll', 'pitch', 'yaw']

export function TargetPanel() {
  const { trajectory, selectedChannel, setSelectedChannel } = useStore()

  const handleJointSelect = (e: React.ChangeEvent<HTMLSelectElement>) => {
    const idx = parseInt(e.target.value)
    if (isNaN(idx)) { setSelectedChannel(null); return }
    setSelectedChannel({ kind: 'joint', index: idx })
  }

  const handleBasePosSelect = (e: React.ChangeEvent<HTMLSelectElement>) => {
    const axis = parseInt(e.target.value)
    if (isNaN(axis)) { setSelectedChannel(null); return }
    setSelectedChannel({ kind: 'basePos', axis })
  }

  const handleBaseQuatSelect = (e: React.ChangeEvent<HTMLSelectElement>) => {
    const axis = parseInt(e.target.value)
    if (isNaN(axis)) { setSelectedChannel(null); return }
    setSelectedChannel({ kind: 'baseQuat', axis })
  }

  return (
    <div style={panelStyle}>
      <h3>Edit Target</h3>
      <label>Joint:</label>
      <select onChange={handleJointSelect} value={selectedChannel?.kind === 'joint' ? selectedChannel.index : ''}>
        <option value="">--</option>
        {trajectory.jointNames.map((name, i) => (
          <option key={i} value={i}>{name}</option>
        ))}
      </select>
      <label>Base Position:</label>
      <select onChange={handleBasePosSelect} value={selectedChannel?.kind === 'basePos' ? selectedChannel.axis : ''}>
        <option value="">--</option>
        {basePosLabels.map((label, i) => (
          <option key={i} value={i}>{label}</option>
        ))}
      </select>
      <label>Base Orientation:</label>
      <select onChange={handleBaseQuatSelect} value={selectedChannel?.kind === 'baseQuat' ? selectedChannel.axis : ''}>
        <option value="">--</option>
        {baseQuatLabels.map((label, i) => (
          <option key={i} value={i}>{label}</option>
        ))}
      </select>
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
