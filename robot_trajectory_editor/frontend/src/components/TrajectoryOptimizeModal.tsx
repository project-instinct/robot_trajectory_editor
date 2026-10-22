import { useState } from 'react'
import { runTrajectoryOptimize } from '../state/trajectoryOptimize'
import { useStore } from '../state/store'

interface Props {
  urdfPath?: string
  onClose: () => void
}

function residualsText(result: { residuals?: Record<string, number>; tolerances?: Record<string, number> }) {
  return Object.entries(result.residuals ?? {})
    .map(([name, residual]) => `${name}: ${residual.toExponential(2)} (tolerance ${result.tolerances?.[name]})`)
    .join('; ')
}

export function TrajectoryOptimizeModal({ urdfPath, onClose }: Props) {
  const [friction, setFriction] = useState(0.7)
  const [maxEffortScale, setMaxEffortScale] = useState(1)
  const [busy, setBusy] = useState(false)
  const [message, setMessage] = useState('')
  const [result, setResult] = useState<Awaited<ReturnType<typeof runTrajectoryOptimize>> | null>(null)

  const previewing = result?.preview === true
  const validInputs = Number.isFinite(friction) && friction >= 0 &&
    Number.isFinite(maxEffortScale) && maxEffortScale >= 1

  const run = async () => {
    setBusy(true)
    setResult(null)
    setMessage('Optimizing the entire trajectory…')
    try {
      const outcome = await runTrajectoryOptimize(urdfPath, () => true, friction, maxEffortScale)
      setResult(outcome)
      if (outcome.success === false) {
        setMessage(outcome.error ?? 'Trajectory optimization failed.')
      } else {
        setMessage(`Trajectory optimization complete. Validated ${outcome.validation_samples_per_interval} samples per interval. ` +
          residualsText(outcome))
      }
    } catch (error) {
      setMessage(error instanceof Error ? error.message : String(error))
    } finally {
      setBusy(false)
    }
  }

  const dump = () => {
    useStore.getState().undo()
    onClose()
  }

  return (
    <div style={overlayStyle}>
      <div style={panelStyle}>
        <h3 style={{ margin: 0 }}>Trajectory Optimize</h3>
        <label style={labelStyle}>
          Max effort scale
          <input aria-label="Maximum effort scale" type="number" min="1" step="0.05"
            value={maxEffortScale} disabled={busy}
            onChange={event => setMaxEffortScale(event.target.valueAsNumber)} />
        </label>
        <div style={hintStyle}>
          Multiplies the URDF actuator effort limits for this optimization only.
        </div>
        <label style={labelStyle}>
          Terrain friction μ
          <input aria-label="Terrain friction coefficient" type="number" min="0" step="0.1"
            value={friction} disabled={busy}
            onChange={event => setFriction(event.target.valueAsNumber)} />
        </label>
        <div style={{ display: 'flex', gap: '6px', marginTop: '4px' }}>
          {!result && (
            <button disabled={busy || !validInputs} onClick={run}>
              {busy ? 'Optimizing…' : 'Run'}
            </button>
          )}
          {previewing && (
            <>
              <button onClick={onClose}>Keep</button>
              <button onClick={dump}>Dump</button>
            </>
          )}
          {!previewing && !busy && (
            <button onClick={onClose}>{result ? 'Close' : 'Cancel'}</button>
          )}
        </div>
        {message && <div role="status" style={messageStyle}>{message}</div>}
      </div>
    </div>
  )
}

const overlayStyle: React.CSSProperties = {
  position: 'fixed', inset: 0, background: 'rgba(0,0,0,.55)', zIndex: 1000,
  display: 'flex', alignItems: 'center', justifyContent: 'center',
}
const panelStyle: React.CSSProperties = {
  width: 'min(520px, 92vw)', background: '#1e1e1e', color: '#ddd',
  border: '1px solid #444', borderRadius: '6px', padding: '14px',
  display: 'flex', flexDirection: 'column', gap: '8px',
}
const labelStyle: React.CSSProperties = {
  display: 'flex', alignItems: 'center', justifyContent: 'space-between',
  fontSize: '12px', gap: '8px',
}
const hintStyle: React.CSSProperties = { fontSize: '11px', color: '#999' }
const messageStyle: React.CSSProperties = {
  fontSize: '11px', overflowWrap: 'anywhere', whiteSpace: 'pre-wrap',
  maxHeight: '160px', overflowY: 'auto',
}
