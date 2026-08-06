import { useStore } from '../state/store'

const TRANS_MIN = -5
const TRANS_MAX = 5
const TRANS_STEP = 0.01
const ANG_MIN_DEG = -180
const ANG_MAX_DEG = 180
const ANG_STEP_DEG = 0.5
const RAD2DEG = 180 / Math.PI

const translationAxes = ['tx', 'ty', 'tz'] as const
const rotationAxes = ['roll', 'pitch', 'yaw'] as const

/**
 * Adjusts the x-y-z (and roll/pitch/yaw) coordinate of the entire trajectory,
 * affecting only the robot base position and orientation. Dragging a slider
 * transforms the whole trajectory live, so the 3D view and the timeline update
 * immediately; Apply confirms the session, and Ctrl+Z reverts the whole
 * adjustment in a single undo step.
 */
export function TransformPanel() {
  const { trajectory, baseTransform, setBaseTransform, applyBaseTransform } = useStore()
  const empty = trajectory.frameCount === 0
  const t = baseTransform ?? { tx: 0, ty: 0, tz: 0, roll: 0, pitch: 0, yaw: 0 }

  return (
    <div style={panelStyle}>
      <h3>Whole Trajectory Transform</h3>
      {translationAxes.map(axis => (
        <SliderRow
          key={axis}
          label={axis.toUpperCase()}
          min={TRANS_MIN}
          max={TRANS_MAX}
          step={TRANS_STEP}
          value={t[axis]}
          disabled={empty}
          onChange={value => setBaseTransform({ ...t, [axis]: value })}
        />
      ))}
      {rotationAxes.map(axis => (
        <SliderRow
          key={axis}
          label={axis}
          min={ANG_MIN_DEG}
          max={ANG_MAX_DEG}
          step={ANG_STEP_DEG}
          value={t[axis] * RAD2DEG}
          disabled={empty}
          onChange={value => setBaseTransform({ ...t, [axis]: value / RAD2DEG })}
        />
      ))}
      <button onClick={applyBaseTransform} disabled={empty || !baseTransform}>Apply</button>
    </div>
  )
}

interface SliderRowProps {
  label: string
  min: number
  max: number
  step: number
  value: number
  disabled: boolean
  onChange: (value: number) => void
}

function SliderRow({ label, min, max, step, value, disabled, onChange }: SliderRowProps) {
  return (
    <div style={sliderRowStyle}>
      <span style={labelStyle}>{label}</span>
      <input
        type="range"
        min={min}
        max={max}
        step={step}
        value={value}
        disabled={disabled}
        onChange={e => onChange(parseFloat(e.target.value))}
        style={{ flex: 1 }}
      />
      <input
        type="number"
        min={min}
        max={max}
        step={step}
        value={value}
        disabled={disabled}
        onChange={e => {
          const v = parseFloat(e.target.value)
          if (!isNaN(v)) onChange(Math.min(max, Math.max(min, v)))
        }}
        style={valueStyle}
      />
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

const sliderRowStyle: React.CSSProperties = {
  display: 'flex',
  gap: '4px',
  alignItems: 'center',
}

const labelStyle: React.CSSProperties = {
  width: '30px',
  fontSize: '11px',
}

const valueStyle: React.CSSProperties = {
  width: '64px',
  fontSize: '11px',
  textAlign: 'right',
  background: '#2a2a4e',
  color: '#ccc',
  border: '1px solid #444',
  borderRadius: '3px',
  padding: '1px 2px',
}
