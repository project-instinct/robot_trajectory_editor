import { useMemo, useState } from 'react'
import { useStore } from '../state/store'
import { getViewport } from '../three/viewportContext'
import {
  computeCartesianInterpolation,
  type CartesianInterpolationMethod,
  type CartesianLinkConstraint,
} from '../three/CartesianInterpolation'

interface CartesianInterpolateModalProps {
  onClose: () => void
}

export function CartesianInterpolateModal({ onClose }: CartesianInterpolateModalProps) {
  const linkNames = useMemo(
    () => getViewport()?.robotModel?.getCartesianTargetLinkNames() ?? [],
    [],
  )
  const [selectedLinks, setSelectedLinks] = useState<string[]>([])
  const [constraints, setConstraints] = useState<Record<string, CartesianLinkConstraint>>({})
  const [method, setMethod] = useState<CartesianInterpolationMethod>('linear')
  const [warning, setWarning] = useState('')

  const toggleLink = (linkName: string) => {
    setSelectedLinks(current => current.includes(linkName)
      ? current.filter(name => name !== linkName)
      : [...current, linkName])
  }

  const apply = () => {
    const store = useStore.getState()
    const robot = getViewport()?.robotModel
    if (!robot) {
      setWarning('Load a robot before applying Cartesian interpolation.')
      return
    }
    if (store.segmentStart === null || store.segmentEnd === null) {
      setWarning('Select a timeline segment before applying Cartesian interpolation.')
      return
    }

    const result = computeCartesianInterpolation(
      store.trajectory,
      robot,
      selectedLinks.map(linkName => ({
        linkName,
        constraint: constraints[linkName] ?? 'pose',
      })),
      store.segmentStart,
      store.segmentEnd,
      method,
      store.currentFrame,
    )
    if (!result.ok) {
      setWarning(result.message)
      return
    }

    store.beginTrajectoryEdit()
    store.trajectory.jointPos.set(result.jointPos)
    store.trajectory.basePoseW.set(result.basePoseW)
    store.trajectory.baseQuatW.set(result.baseQuatW)
    store.touchTrajectory()
    onClose()
  }

  return (
    <div style={overlayStyle} onClick={onClose}>
      <div style={modalStyle} onClick={event => event.stopPropagation()}>
        <h3>Cartesian Interpolate</h3>
        <label style={fieldStyle}>
          <span>Method</span>
          <select value={method} onChange={event => setMethod(event.target.value as CartesianInterpolationMethod)}>
            <option value="linear">Linear</option>
            <option value="cubic">Cubic</option>
          </select>
        </label>
        <div style={{ marginTop: '10px' }}>
          <h4>Links</h4>
          <div style={linkListStyle}>
            {linkNames.map(linkName => (
              <div key={linkName} style={linkStyle}>
                <label style={{ display: 'flex', alignItems: 'center', gap: '8px', flex: 1 }}>
                  <input
                    type="checkbox"
                    checked={selectedLinks.includes(linkName)}
                    onChange={() => toggleLink(linkName)}
                  />
                  <span>{linkName}</span>
                </label>
                <select
                  value={constraints[linkName] ?? 'pose'}
                  disabled={!selectedLinks.includes(linkName)}
                  onChange={event => setConstraints(current => ({
                    ...current,
                    [linkName]: event.target.value as CartesianLinkConstraint,
                  }))}
                >
                  <option value="pose">Position + orientation</option>
                  <option value="position">Position only</option>
                </select>
              </div>
            ))}
            {linkNames.length === 0 && <span style={{ color: '#f88' }}>No IK target links are available.</span>}
          </div>
        </div>
        {warning && <div style={warningStyle}>{warning}</div>}
        <div style={actionsStyle}>
          <button onClick={onClose}>Cancel</button>
          <button onClick={apply} disabled={selectedLinks.length === 0}>Apply</button>
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
  width: '420px', maxWidth: '90vw', padding: '16px',
  background: '#1e1e2e', border: '1px solid #444', borderRadius: '8px',
}

const fieldStyle: React.CSSProperties = {
  display: 'flex', alignItems: 'center', justifyContent: 'space-between', gap: '12px',
}

const linkListStyle: React.CSSProperties = {
  maxHeight: '260px', overflowY: 'auto', padding: '6px',
  border: '1px solid #444', borderRadius: '4px',
}

const linkStyle: React.CSSProperties = {
  display: 'flex', alignItems: 'center', gap: '8px', padding: '3px 0', fontSize: '12px',
}

const warningStyle: React.CSSProperties = {
  marginTop: '10px', padding: '8px', color: '#f99', background: '#3a2028', borderRadius: '4px',
}

const actionsStyle: React.CSSProperties = {
  display: 'flex', justifyContent: 'flex-end', gap: '8px', marginTop: '12px',
}
