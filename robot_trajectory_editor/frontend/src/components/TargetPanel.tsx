import { useStore } from '../state/store'

export function TargetPanel() {
  const { trajectory, selectedChannel, setSelectedChannel } = useStore()

  const handleJointSelect = (e: React.ChangeEvent<HTMLSelectElement>) => {
    const idx = parseInt(e.target.value)
    if (isNaN(idx)) { setSelectedChannel(null); return }
    setSelectedChannel({ kind: 'joint', index: idx })
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
      <button onClick={() => setSelectedChannel({ kind: 'basePos' })}>Base Position</button>
      <button onClick={() => setSelectedChannel({ kind: 'baseQuat' })}>Base Orientation</button>
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
