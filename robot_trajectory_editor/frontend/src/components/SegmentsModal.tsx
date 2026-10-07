import { useState } from 'react'
import { useStore } from '../state/store'
import { downloadTrajectory } from '../api/client'
import { SegmentsPreview } from './SegmentsPreview'

interface SegmentDraft {
  start: number | null
  end: number | null
}

interface SegmentsModalProps {
  onClose: () => void
}

/**
 * Select one or more frame ranges and chop the trajectory into those segments.
 * A single segment replaces the active trajectory; multiple segments are each
 * downloaded as a separate .npz file named `<original>_segment_<n>.npz`.
 */
export function SegmentsModal({ onClose }: SegmentsModalProps) {
  const { trajectory, currentFrame, replaceTrajectory, beginTrajectoryEdit, trajectorySourceName } = useStore()
  const [segments, setSegments] = useState<SegmentDraft[]>([{ start: null, end: null }])
  const [warning, setWarning] = useState('')
  const [saving, setSaving] = useState(false)

  const empty = trajectory.frameCount === 0
  const lastFrame = trajectory.frameCount - 1

  const setField = (index: number, field: 'start' | 'end', value: number | null) => {
    setSegments(current => current.map((segment, i) => (
      i === index ? { ...segment, [field]: value } : segment
    )))
  }

  const addSegment = () => {
    setSegments(current => [...current, { start: null, end: null }])
  }

  const removeSegment = (index: number) => {
    setSegments(current => current.length === 1
      ? [{ start: null, end: null }]
      : current.filter((_, i) => i !== index))
  }

  const apply = async () => {
    if (trajectory.frameCount === 0) {
      setWarning('Load a trajectory before applying segments.')
      return
    }

    const normalized: Array<[number, number]> = []
    for (let i = 0; i < segments.length; i++) {
      const { start, end } = segments[i]
      if (start === null || end === null) {
        setWarning(`Segment ${i + 1}: set both a start and an end frame.`)
        return
      }
      if (!Number.isInteger(start) || !Number.isInteger(end)) {
        setWarning(`Segment ${i + 1}: frames must be whole numbers.`)
        return
      }
      const s = Math.min(start, end)
      const e = Math.max(start, end)
      if (s < 0 || e > lastFrame) {
        setWarning(`Segment ${i + 1} is outside the trajectory frame range 0-${lastFrame}.`)
        return
      }
      normalized.push([s, e])
    }

    const slices = normalized.map(([s, e]) => trajectory.slice(s, e))
    const baseName = trajectorySourceName || 'trajectory'

    beginTrajectoryEdit()
    if (slices.length > 1) {
      setSaving(true)
      try {
        for (let i = 0; i < slices.length; i++) {
          await downloadTrajectory(slices[i].toJSON(), `${baseName}_segment_${i + 1}.npz`)
        }
      } catch (error) {
        setWarning(error instanceof Error ? error.message : String(error))
        setSaving(false)
        return
      }
      setSaving(false)
    }

    replaceTrajectory(slices[0])
    onClose()
  }

  return (
    <div style={overlayStyle} onClick={onClose}>
      <div style={modalStyle} onClick={event => event.stopPropagation()}>
        <h3>Segments</h3>
        <SegmentsPreview segments={segments} />
        <div style={infoStyle}>
          <span>Current frame: {currentFrame}</span>
          <span>{empty ? 'No trajectory loaded' : `Trajectory frames: 0-${lastFrame}`}</span>
        </div>
        {segments.map((segment, index) => (
          <div key={index} style={segmentRowStyle}>
            <span style={segmentLabelStyle}>Segment {index + 1}</span>
            <label style={frameFieldStyle}>
              Start
              <input
                type="number"
                min={0}
                max={lastFrame}
                step={1}
                value={segment.start ?? ''}
                placeholder="-"
                onChange={event => setField(index, 'start', event.target.value === '' ? null : Number(event.target.value))}
              />
            </label>
            <label style={frameFieldStyle}>
              End
              <input
                type="number"
                min={0}
                max={lastFrame}
                step={1}
                value={segment.end ?? ''}
                placeholder="-"
                onChange={event => setField(index, 'end', event.target.value === '' ? null : Number(event.target.value))}
              />
            </label>
            <button onClick={() => setField(index, 'start', currentFrame)} disabled={empty}>Set Start Frame</button>
            <button onClick={() => setField(index, 'end', currentFrame)} disabled={empty}>Set End Frame</button>
            <button onClick={() => removeSegment(index)}>Remove Segment</button>
          </div>
        ))}
        <div style={addRowStyle}>
          <button onClick={addSegment}>Add Segment</button>
        </div>
        {warning && <div style={warningStyle}>{warning}</div>}
        <div style={actionsStyle}>
          <button onClick={onClose} disabled={saving}>Cancel</button>
          <button onClick={apply} disabled={saving || empty}>{saving ? 'Saving...' : 'Apply'}</button>
        </div>
      </div>
    </div>
  )
}

const overlayStyle: React.CSSProperties = {
  position: 'fixed', inset: 0, zIndex: 220,
  display: 'flex', alignItems: 'center', justifyContent: 'center',
  background: 'rgba(0, 0, 0, 0.7)',
}

const modalStyle: React.CSSProperties = {
  width: '720px', maxWidth: '92vw', maxHeight: '92vh', overflowY: 'auto',
  padding: '16px', background: '#1e1e2e', border: '1px solid #444', borderRadius: '8px',
}

const infoStyle: React.CSSProperties = {
  display: 'flex', justifyContent: 'space-between', gap: '12px',
  fontSize: '12px', color: '#88aacc', marginBottom: '8px',
}

const segmentRowStyle: React.CSSProperties = {
  display: 'flex', alignItems: 'center', gap: '8px', padding: '6px 0',
  fontSize: '12px', borderBottom: '1px solid #333',
}

const segmentLabelStyle: React.CSSProperties = {
  width: '64px', color: '#88aacc',
}

const frameFieldStyle: React.CSSProperties = {
  display: 'flex', alignItems: 'center', gap: '4px',
}

const addRowStyle: React.CSSProperties = {
  marginTop: '8px',
}

const warningStyle: React.CSSProperties = {
  marginTop: '10px', padding: '8px', color: '#f99', background: '#3a2028', borderRadius: '4px',
}

const actionsStyle: React.CSSProperties = {
  display: 'flex', justifyContent: 'flex-end', gap: '8px', marginTop: '12px',
}
