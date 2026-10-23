import { useState } from 'react'
import { DEFAULT_TRAJECTORY_OPTIMIZE_OPTIONS } from '../api/client'
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
  const [friction, setFriction] = useState(DEFAULT_TRAJECTORY_OPTIMIZE_OPTIONS.friction)
  const [maxEffortScale, setMaxEffortScale] = useState(DEFAULT_TRAJECTORY_OPTIMIZE_OPTIONS.maxEffortScale)
  const [smoothingHz, setSmoothingHz] = useState(DEFAULT_TRAJECTORY_OPTIMIZE_OPTIONS.smoothingHz)
  const [glitchSpeedScale, setGlitchSpeedScale] = useState(DEFAULT_TRAJECTORY_OPTIMIZE_OPTIONS.glitchSpeedScale)
  const [busy, setBusy] = useState(false)
  const [message, setMessage] = useState('')
  const [copied, setCopied] = useState(false)
  const [result, setResult] = useState<Awaited<ReturnType<typeof runTrajectoryOptimize>> | null>(null)

  const previewing = result?.preview === true
  const validInputs = Number.isFinite(friction) && friction >= 0 &&
    Number.isFinite(maxEffortScale) && maxEffortScale >= 1 &&
    Number.isFinite(smoothingHz) && smoothingHz >= 0 &&
    Number.isFinite(glitchSpeedScale) && glitchSpeedScale >= 0

  const run = async () => {
    setBusy(true)
    setResult(null)
    setCopied(false)
    setMessage('Optimizing the entire trajectory…')
    try {
      const outcome = await runTrajectoryOptimize(urdfPath, () => true,
        { friction, maxEffortScale, smoothingHz, glitchSpeedScale })
      setResult(outcome)
      if (outcome.report) {
        setMessage('')
      } else if (outcome.success === false) {
        const details = residualsText(outcome)
        setMessage(`${outcome.error ?? 'Trajectory optimization failed.'}${details ? ` Residuals: ${details}` : ''}`)
      } else {
        setMessage(`Trajectory optimization complete. ${residualsText(outcome)}`)
      }
    } catch (error) {
      setMessage(error instanceof Error ? error.message : String(error))
    } finally {
      setBusy(false)
    }
  }

  const copyReport = async () => {
    if (!result?.report) return
    await navigator.clipboard.writeText(result.report)
    setCopied(true)
  }

  const dump = () => {
    useStore.getState().undo()
    onClose()
  }

  return (
    <div style={overlayStyle}>
      <div style={panelStyle}>
        <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between' }}>
          <h3 style={{ margin: 0 }}>Trajectory Optimize</h3>
          {!previewing && (
            <button aria-label="Close Trajectory Optimize" onClick={onClose} disabled={busy}
              style={{ padding: '0 8px', fontSize: '16px', lineHeight: '22px' }}>×</button>
          )}
        </div>
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
        <label style={labelStyle}>
          Target smoothing (Hz)
          <input aria-label="Target smoothing cutoff" type="number" min="0" step="1"
            value={smoothingHz} disabled={busy}
            onChange={event => setSmoothingHz(event.target.valueAsNumber)} />
        </label>
        <div style={hintStyle}>
          Low-pass cutoff of the motion the optimizer stays close to. Lower is smoother; 0 keeps every
          frame's detail, including jitter.
        </div>
        <label style={labelStyle}>
          Glitch speed threshold (× URDF limit)
          <input aria-label="Glitch speed threshold" type="number" min="0" step="0.5"
            value={glitchSpeedScale} disabled={busy}
            onChange={event => setGlitchSpeedScale(event.target.valueAsNumber)} />
        </label>
        <div style={hintStyle}>
          Joint jumps faster than this multiple of the URDF velocity limit are interpolated away before
          optimizing; 0 disables. Keyframes are never changed.
        </div>
        <div style={{ display: 'flex', gap: '6px', marginTop: '4px' }}>
          {!result && (
            <button disabled={busy || !validInputs} onClick={run}>
              {busy ? 'Optimizing…' : 'Apply'}
            </button>
          )}
          {previewing && (
            <>
              <button onClick={onClose}>Keep anyway</button>
              <button onClick={dump}>Dump</button>
            </>
          )}
          {result && !previewing && (
            <button onClick={onClose}>Close</button>
          )}
          {!result && !busy && (
            <button onClick={onClose}>Cancel</button>
          )}
        </div>
        <div style={hintStyle}>
          Apply starts optimization. A validated result is applied as one undo step;
          if it fails, keep or dump the preview.
        </div>
        {result?.success === false && (
          <div style={warningStyle}>Optimization failed — this preview is not physically validated.</div>
        )}
        {message && <div role="status" style={messageStyle}>{message}</div>}
        {result?.report && (
          <>
            <pre role="status" aria-label="Trajectory Optimize report" style={reportStyle}>{result.report}</pre>
            <button onClick={copyReport} style={{ alignSelf: 'flex-start' }}>
              {copied ? 'Report copied' : 'Copy report'}
            </button>
          </>
        )}
      </div>
    </div>
  )
}

const overlayStyle: React.CSSProperties = {
  position: 'fixed', inset: 0, background: 'rgba(0,0,0,.55)', zIndex: 1000,
  display: 'flex', alignItems: 'center', justifyContent: 'center',
}
const panelStyle: React.CSSProperties = {
  width: 'min(720px, 94vw)', background: '#1e1e1e', color: '#ddd',
  border: '1px solid #444', borderRadius: '6px', padding: '14px',
  display: 'flex', flexDirection: 'column', gap: '8px',
  maxHeight: '90vh', overflowY: 'auto',
}
const labelStyle: React.CSSProperties = {
  display: 'flex', alignItems: 'center', justifyContent: 'space-between',
  fontSize: '12px', gap: '8px',
}
const hintStyle: React.CSSProperties = { fontSize: '11px', color: '#999' }
const warningStyle: React.CSSProperties = {
  fontSize: '12px', color: '#ff9a9a', fontWeight: 600,
}
const reportStyle: React.CSSProperties = {
  fontSize: '11px', margin: 0, padding: '6px', background: '#151515', border: '1px solid #333',
  whiteSpace: 'pre-wrap', overflowWrap: 'anywhere', maxHeight: '260px', overflowY: 'auto', userSelect: 'text',
}
const messageStyle: React.CSSProperties = {
  fontSize: '11px', overflowWrap: 'anywhere', whiteSpace: 'pre-wrap',
  maxHeight: '160px', overflowY: 'auto',
}
