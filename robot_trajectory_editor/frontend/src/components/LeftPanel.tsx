import { useStore } from '../state/store'
import { Trajectory } from '../state/Trajectory'
import { parseTrajectory, serializeTrajectory, uploadRobotFolder, getRobotInfo, getRobotUrdfList } from '../api/client'
import { loadTerrainObj, exportTerrainObj } from '../three/Terrain'
import { useEffect, useRef, useState } from 'react'
import * as THREE from 'three'

interface LeftPanelProps {
  onLoadTerrain: (group: THREE.Group) => void
  terrain: THREE.Group | null
  onEditTerrain: () => void
  onRobotUploaded: (urdfPath?: string) => void
}

export function LeftPanel({ onLoadTerrain, terrain, onEditTerrain, onRobotUploaded }: LeftPanelProps) {
  const { setTrajectory, trajectory } = useStore()
  const [showRobotSelect, setShowRobotSelect] = useState(false)
  const [urdfList, setUrdfList] = useState<{ path: string; name: string }[]>([])
  const [selectedUrdf, setSelectedUrdf] = useState('')
  const [loadError, setLoadError] = useState('')
  const fileInputRef = useRef<HTMLInputElement>(null)
  const terrainInputRef = useRef<HTMLInputElement>(null)
  const robotFolderRef = useRef<HTMLInputElement>(null)

  useEffect(() => {
    const input = robotFolderRef.current
    if (input) {
      input.setAttribute('webkitdirectory', '')
      input.setAttribute('directory', '')
    }
  }, [])

  useEffect(() => {
    getRobotInfo().then(info => setShowRobotSelect(!info.has_urdf))
  }, [])

  const handleLoadTrajectory = async () => {
    const input = fileInputRef.current
    if (!input) return
    input.onchange = async () => {
      const file = input.files?.[0]
      if (!file) return
      try {
        const data = await parseTrajectory(file)
        setTrajectory(Trajectory.fromJSON(data))
        setLoadError('')
      } catch (e) {
        setLoadError(e instanceof Error ? e.message : String(e))
        console.error('Failed to load trajectory:', e)
      }
      input.value = ''
    }
    input.click()
  }

  const handleSaveTrajectory = async () => {
    const json = trajectory.toJSON()
    try {
      const blob = await serializeTrajectory(json)
      const url = URL.createObjectURL(blob)
      const a = document.createElement('a')
      a.href = url
      a.download = 'trajectory.npz'
      a.click()
      URL.revokeObjectURL(url)
    } catch (e) {
      console.error('Failed to save trajectory:', e)
    }
  }

  const handleLoadTerrain = () => {
    terrainInputRef.current?.click()
  }

  const handleTerrainFileChange = async (e: React.ChangeEvent<HTMLInputElement>) => {
    const file = e.target.files?.[0]
    if (!file) return
    const text = await file.text()
    const group = await loadTerrainObj(text)
    onLoadTerrain(group)
    e.target.value = ''
  }

  const handleSaveTerrain = () => {
    if (!terrain) return
    const obj = exportTerrainObj(terrain)
    const blob = new Blob([obj], { type: 'text/plain' })
    const url = URL.createObjectURL(blob)
    const a = document.createElement('a')
    a.href = url
    a.download = 'terrain.obj'
    a.click()
    URL.revokeObjectURL(url)
  }

  const handleLoadRobot = () => {
    robotFolderRef.current?.click()
  }

  const handleRobotFolderChange = async (e: React.ChangeEvent<HTMLInputElement>) => {
    const files = e.target.files
    if (!files) return
    try {
      await uploadRobotFolder(Array.from(files))
      const result = await getRobotUrdfList()
      setUrdfList(result.urdfs)
      if (result.urdfs.length === 1) {
        setShowRobotSelect(false)
        onRobotUploaded(result.urdfs[0].path)
      }
    } catch (err) {
      console.error('Failed to upload robot:', err)
    }
    e.target.value = ''
  }

  const handleUrdfSelect = () => {
    if (selectedUrdf) {
      setShowRobotSelect(false)
      setUrdfList([])
      onRobotUploaded(selectedUrdf)
    }
  }

  return (
    <div style={panelStyle}>
      <h3>Files</h3>
      <button onClick={handleLoadTrajectory}>Load Trajectory</button>
      <button onClick={handleSaveTrajectory}>Save Trajectory</button>
      <button onClick={handleLoadTerrain}>Load Terrain</button>
      <button onClick={handleSaveTerrain}>Save Terrain</button>
      <button onClick={onEditTerrain}>Edit Terrain</button>
      {showRobotSelect && (
        <button onClick={handleLoadRobot}>Load Robot URDF</button>
      )}
      {loadError && (
        <div style={{ color: '#f66', fontSize: '11px', marginTop: '4px' }}>{loadError}</div>
      )}
      {urdfList.length > 1 && (
        <div style={{ display: 'flex', flexDirection: 'column', gap: '4px', marginTop: '4px', padding: '6px', background: '#2a2a4e', borderRadius: '4px' }}>
          <span style={{ fontSize: '11px', color: '#88aacc' }}>Select URDF:</span>
          <select value={selectedUrdf} onChange={e => setSelectedUrdf(e.target.value)} style={{ width: '100%' }}>
            <option value="">--</option>
            {urdfList.map(u => (
              <option key={u.path} value={u.path}>{u.name}</option>
            ))}
          </select>
          <button onClick={handleUrdfSelect} disabled={!selectedUrdf}>Confirm</button>
        </div>
      )}
      <input ref={fileInputRef} type="file" accept=".npz" style={{ display: 'none' }} />
      <input ref={terrainInputRef} type="file" accept=".obj" style={{ display: 'none' }} onChange={handleTerrainFileChange} />
      <input ref={robotFolderRef} type="file" style={{ display: 'none' }} onChange={handleRobotFolderChange} />
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
