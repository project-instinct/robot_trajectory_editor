import { useStore } from '../state/store'
import { Trajectory } from '../state/Trajectory'
import { parseTrajectory, saveTrajectoryFile, uploadRobotFolder, getRobotInfo, getRobotUrdfList } from '../api/client'
import { loadTerrain, exportTerrainObj } from '../three/Terrain'
import { HelpModal } from './HelpModal'
import { useEffect, useRef, useState } from 'react'
import * as THREE from 'three'

interface LeftPanelProps {
  onLoadTerrain: (group: THREE.Group) => void
  terrain: THREE.Group | null
  onEditTerrain: () => void
  onRobotUploaded: (urdfPath?: string) => void
}

interface LoadedTrajectory {
  name: string
  /** File name without the .npz extension, used for exported segment files. */
  fileName: string
  trajectory: Trajectory
}

export function LeftPanel({ onLoadTerrain, terrain, onEditTerrain, onRobotUploaded }: LeftPanelProps) {
  const { setTrajectory, trajectory, setTrajectorySourceName } = useStore()
  const [loadedTrajectories, setLoadedTrajectories] = useState<LoadedTrajectory[]>([])
  const [selectedTrajectoryIndex, setSelectedTrajectoryIndex] = useState(0)
  const [showRobotSelect, setShowRobotSelect] = useState(false)
  const [urdfList, setUrdfList] = useState<{ path: string; name: string }[]>([])
  const [selectedUrdf, setSelectedUrdf] = useState('')
  const [loadError, setLoadError] = useState('')
  const [showHelp, setShowHelp] = useState(false)
  const [terrainName, setTerrainName] = useState<string | null>(null)
  const fileInputRef = useRef<HTMLInputElement>(null)
  const trajectoryFolderRef = useRef<HTMLInputElement>(null)
  const terrainInputRef = useRef<HTMLInputElement>(null)
  const robotFolderRef = useRef<HTMLInputElement>(null)

  useEffect(() => {
    for (const input of [trajectoryFolderRef.current, robotFolderRef.current]) {
      if (!input) continue
      input.setAttribute('webkitdirectory', '')
      input.setAttribute('directory', '')
    }
  }, [])

  useEffect(() => {
    getRobotInfo().then(info => setShowRobotSelect(!info.has_urdf))
  }, [])

  const loadTrajectoryFiles = async (files: File[]) => {
    const npzFiles = files.filter(file => file.name.toLowerCase().endsWith('.npz'))
    if (npzFiles.length === 0) {
      setLoadError('No .npz trajectory files selected.')
      return
    }

    const loaded: LoadedTrajectory[] = []
    const errors: string[] = []
    for (const file of npzFiles) {
      try {
        const data = await parseTrajectory(file)
        loaded.push({
          name: file.webkitRelativePath || file.name,
          fileName: baseNameOf(file.name),
          trajectory: Trajectory.fromJSON(data),
        })
      } catch (e) {
        errors.push(`${file.name}: ${e instanceof Error ? e.message : String(e)}`)
        console.error('Failed to load trajectory:', e)
      }
    }

    if (loaded.length > 0) {
      setLoadedTrajectories(loaded)
      setSelectedTrajectoryIndex(0)
      setTrajectory(loaded[0].trajectory)
      setTrajectorySourceName(loaded[0].fileName)
    }
    setLoadError(errors.join('\n'))
  }

  const handleTrajectoryFileChange = async (e: React.ChangeEvent<HTMLInputElement>) => {
    const files = Array.from(e.target.files ?? [])
    e.target.value = ''
    if (files.length === 0) return
    await loadTrajectoryFiles(files)
  }

  const handleTrajectorySelect = (e: React.ChangeEvent<HTMLSelectElement>) => {
    const nextIndex = Number(e.target.value)
    const nextTrajectory = loadedTrajectories[nextIndex]
    if (!nextTrajectory || nextIndex === selectedTrajectoryIndex) return

    // Edits are made in place, except Undo replaces the active trajectory with
    // a snapshot. Capture the current store object before switching so either
    // kind of edit is retained when this file is selected again.
    const currentTrajectory = useStore.getState().trajectory
    setLoadedTrajectories(entries => entries.map((entry, index) => (
      index === selectedTrajectoryIndex ? { ...entry, trajectory: currentTrajectory } : entry
    )))
    setSelectedTrajectoryIndex(nextIndex)
    setTrajectory(nextTrajectory.trajectory)
    setTrajectorySourceName(nextTrajectory.fileName)
  }

  const handleSaveTrajectory = async () => {
    try {
      await saveTrajectoryFile(trajectory.toJSON())
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
    const isStl = file.name.toLowerCase().endsWith('.stl')
    const data = isStl ? await file.arrayBuffer() : await file.text()
    const group = await loadTerrain(data, file.name)
    onLoadTerrain(group)
    setTerrainName(file.name)
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
      <button onClick={() => fileInputRef.current?.click()}>Load Trajectory</button>
      <button onClick={() => trajectoryFolderRef.current?.click()}>Load Trajectory Folder</button>
      {loadedTrajectories.length > 1 && (
        <div style={trajectoryListStyle}>
          <span style={{ fontSize: '11px', color: '#88aacc' }}>Loaded trajectories:</span>
          <select
            aria-label="Loaded trajectories"
            value={selectedTrajectoryIndex}
            onChange={handleTrajectorySelect}
            size={Math.min(loadedTrajectories.length, 6)}
            style={{ width: '100%' }}
          >
            {loadedTrajectories.map((entry, index) => (
              <option key={`${entry.name}-${index}`} value={index}>{entry.name}</option>
            ))}
          </select>
        </div>
      )}
      <button onClick={handleSaveTrajectory}>Save Trajectory</button>
      <button onClick={handleLoadTerrain}>Load Terrain</button>
      {terrainName && (
        <div style={{ fontSize: '11px', color: '#88aacc', wordBreak: 'break-all' }}>{terrainName}</div>
      )}
      <button onClick={handleSaveTerrain}>Save Terrain</button>
      <button onClick={onEditTerrain}>Edit Terrain</button>
      <button onClick={() => setShowHelp(true)}>?</button>
      {showHelp && <HelpModal onClose={() => setShowHelp(false)} />}
      {showRobotSelect && (
        <button onClick={handleLoadRobot}>Load Robot URDF</button>
      )}
      {loadError && (
        <div style={{ color: '#f66', fontSize: '11px', marginTop: '4px', whiteSpace: 'pre-wrap' }}>{loadError}</div>
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
      <input ref={fileInputRef} type="file" accept=".npz" multiple style={{ display: 'none' }} onChange={handleTrajectoryFileChange} />
      <input ref={trajectoryFolderRef} type="file" style={{ display: 'none' }} onChange={handleTrajectoryFileChange} />
      <input ref={terrainInputRef} type="file" accept=".obj,.stl" style={{ display: 'none' }} onChange={handleTerrainFileChange} />
      <input ref={robotFolderRef} type="file" style={{ display: 'none' }} onChange={handleRobotFolderChange} />
    </div>
  )
}

function baseNameOf(fileName: string): string {
  const base = fileName.split(/[\\/]/).pop() ?? fileName
  return base.replace(/\.npz$/i, '')
}

const panelStyle: React.CSSProperties = {
  padding: '8px',
  display: 'flex',
  flexDirection: 'column',
  gap: '6px',
  borderBottom: '1px solid #333',
}

const trajectoryListStyle: React.CSSProperties = {
  display: 'flex',
  flexDirection: 'column',
  gap: '4px',
  padding: '6px',
  background: '#2a2a4e',
  borderRadius: '4px',
}
