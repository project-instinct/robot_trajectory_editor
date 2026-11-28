import { useStore } from '../state/store'
import { Trajectory } from '../state/Trajectory'
import {
  parseTrajectory,
  saveTrajectoryFile,
  uploadRobotFolder,
  getRobotInfo,
  getRobotUrdfList,
  loadServerTrajectories,
  saveServerTrajectory,
  loadServerTerrain,
  saveServerTerrain,
  selectServerRobotUrdf,
} from '../api/client'
import { loadTerrain, exportTerrainObj } from '../three/Terrain'
import { HelpModal } from './HelpModal'
import { ServerFileModal, type ServerFileMode } from './ServerFileModal'
import {
  resizeTrajectoryListBox,
  TRAJECTORY_LIST_MAX_HEIGHT,
  TRAJECTORY_LIST_MAX_WIDTH,
  TRAJECTORY_LIST_PANEL_CHROME,
  type TrajectoryListResizeAxis,
  type TrajectoryListSize,
} from './trajectoryListResize'
import { useEffect, useRef, useState } from 'react'
import * as THREE from 'three'

interface LeftPanelProps {
  onLoadTerrain: (group: THREE.Group) => void
  terrain: THREE.Group | null
  onEditTerrain: () => void
  onRobotUploaded: (urdfPath?: string) => void
  onTrajectoryListWidthChange?: (requiredPanelWidth: number) => void
}

interface LoadedTrajectory {
  name: string
  /** File name without the .npz extension, used for exported segment files. */
  fileName: string
  trajectory: Trajectory
}

interface ServerPickerRequest {
  action: 'load-trajectory' | 'load-trajectory-folder' | 'save-trajectory' | 'load-terrain' | 'save-terrain' | 'load-robot'
  title: string
  mode: ServerFileMode
  extensions?: string[]
  defaultFileName?: string
}

export function LeftPanel({ onLoadTerrain, terrain, onEditTerrain, onRobotUploaded, onTrajectoryListWidthChange }: LeftPanelProps) {
  const { setTrajectory, trajectory, setTrajectorySourceName, trajectorySourceName, fileExplorerSide } = useStore()
  const [loadedTrajectories, setLoadedTrajectories] = useState<LoadedTrajectory[]>([])
  const [selectedTrajectoryIndex, setSelectedTrajectoryIndex] = useState(0)
  const [listSize, setListSize] = useState<{ width: number | null; height: number | null }>({ width: null, height: null })
  const [showRobotSelect, setShowRobotSelect] = useState(false)
  const [urdfList, setUrdfList] = useState<{ path: string; name: string }[]>([])
  const [selectedUrdf, setSelectedUrdf] = useState('')
  const [loadError, setLoadError] = useState('')
  const [showHelp, setShowHelp] = useState(false)
  const [terrainName, setTerrainName] = useState<string | null>(null)
  const [serverPicker, setServerPicker] = useState<ServerPickerRequest | null>(null)
  const fileInputRef = useRef<HTMLInputElement>(null)
  const trajectoryFolderRef = useRef<HTMLInputElement>(null)
  const terrainInputRef = useRef<HTMLInputElement>(null)
  const robotFolderRef = useRef<HTMLInputElement>(null)
  const trajectoryListRef = useRef<HTMLDivElement>(null)
  const listResizeCleanupRef = useRef<(() => void) | null>(null)

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

  useEffect(() => {
    if (loadedTrajectories.length <= 1) {
      onTrajectoryListWidthChange?.(0)
    }
  }, [loadedTrajectories.length, onTrajectoryListWidthChange])

  useEffect(() => () => {
    listResizeCleanupRef.current?.()
  }, [])

  useEffect(() => {
    const onSaveEvent = () => {
      if (useStore.getState().fileExplorerSide !== 'server') return
      const defaultName = `${useStore.getState().trajectorySourceName || 'trajectory'}.npz`
      setServerPicker({
        action: 'save-trajectory',
        title: 'Save Trajectory',
        mode: 'save-file',
        extensions: ['.npz'],
        defaultFileName: defaultName,
      })
    }
    window.addEventListener('rte:save-trajectory', onSaveEvent)
    return () => window.removeEventListener('rte:save-trajectory', onSaveEvent)
  }, [])

  const startListResize = (axis: TrajectoryListResizeAxis) => (e: React.PointerEvent<HTMLDivElement>) => {
    const box = trajectoryListRef.current
    if (!box) return
    e.preventDefault()
    e.stopPropagation()

    listResizeCleanupRef.current?.()
    const rect = box.getBoundingClientRect()
    const start: TrajectoryListSize = { width: rect.width, height: rect.height }
    const startX = e.clientX
    const startY = e.clientY
    const maxWidth = Math.min(TRAJECTORY_LIST_MAX_WIDTH, window.innerWidth - 360)
    const maxHeight = Math.min(TRAJECTORY_LIST_MAX_HEIGHT, window.innerHeight - 200)
    document.body.style.userSelect = 'none'

    const handleMove = (moveEvent: PointerEvent) => {
      const next = resizeTrajectoryListBox(
        start,
        moveEvent.clientX - startX,
        moveEvent.clientY - startY,
        axis,
        maxWidth,
        maxHeight,
      )
      setListSize(prev => ({
        width: axis === 'y' ? prev.width : next.width,
        height: axis === 'x' ? prev.height : next.height,
      }))
      if (axis !== 'y') {
        onTrajectoryListWidthChange?.(next.width + TRAJECTORY_LIST_PANEL_CHROME)
      }
    }

    const cleanup = () => {
      document.body.style.userSelect = ''
      window.removeEventListener('pointermove', handleMove)
      window.removeEventListener('pointerup', cleanup)
      window.removeEventListener('pointercancel', cleanup)
      listResizeCleanupRef.current = null
    }

    listResizeCleanupRef.current = cleanup
    window.addEventListener('pointermove', handleMove)
    window.addEventListener('pointerup', cleanup)
    window.addEventListener('pointercancel', cleanup)
  }

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

  const handleLoadTrajectoryClick = () => {
    if (fileExplorerSide === 'server') {
      setServerPicker({
        action: 'load-trajectory',
        title: 'Load Trajectory',
        mode: 'open-files',
        extensions: ['.npz'],
      })
      return
    }
    fileInputRef.current?.click()
  }

  const handleLoadTrajectoryFolderClick = () => {
    if (fileExplorerSide === 'server') {
      setServerPicker({
        action: 'load-trajectory-folder',
        title: 'Load Trajectory Folder',
        mode: 'open-dir',
      })
      return
    }
    trajectoryFolderRef.current?.click()
  }

  const handleSaveTrajectory = async () => {
    if (fileExplorerSide === 'server') {
      setServerPicker({
        action: 'save-trajectory',
        title: 'Save Trajectory',
        mode: 'save-file',
        extensions: ['.npz'],
        defaultFileName: `${trajectorySourceName || 'trajectory'}.npz`,
      })
      return
    }
    try {
      await saveTrajectoryFile(trajectory.toJSON())
    } catch (e) {
      console.error('Failed to save trajectory:', e)
    }
  }

  const handleLoadTerrain = () => {
    if (fileExplorerSide === 'server') {
      setServerPicker({
        action: 'load-terrain',
        title: 'Load Terrain',
        mode: 'open-file',
        extensions: ['.obj', '.stl'],
      })
      return
    }
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
    if (fileExplorerSide === 'server') {
      setServerPicker({
        action: 'save-terrain',
        title: 'Save Terrain',
        mode: 'save-file',
        extensions: ['.obj'],
        defaultFileName: terrainName ? terrainName.replace(/\.(obj|stl)$/i, '.obj') : 'terrain.obj',
      })
      return
    }
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
    if (fileExplorerSide === 'server') {
      setServerPicker({
        action: 'load-robot',
        title: 'Load Robot URDF',
        mode: 'open-file',
        extensions: ['.urdf'],
      })
      return
    }
    robotFolderRef.current?.click()
  }

  const handleServerPickerConfirm = async (paths: string[]) => {
    const picker = serverPicker
    setServerPicker(null)
    if (!picker || paths.length === 0) return

    setLoadError('')
    try {
      if (picker.action === 'load-trajectory' || picker.action === 'load-trajectory-folder') {
        const res = picker.action === 'load-trajectory'
          ? await loadServerTrajectories({ paths })
          : await loadServerTrajectories({ directory: paths[0] })
        const loaded: LoadedTrajectory[] = res.trajectories.map(item => ({
          name: item.name,
          fileName: baseNameOf(item.name),
          trajectory: Trajectory.fromJSON(item.data),
        }))
        if (loaded.length > 0) {
          setLoadedTrajectories(loaded)
          setSelectedTrajectoryIndex(0)
          setTrajectory(loaded[0].trajectory)
          setTrajectorySourceName(loaded[0].fileName)
        } else if (res.errors.length === 0) {
          setLoadError('No .npz trajectory files found.')
        }
        if (res.errors.length > 0) {
          setLoadError(res.errors.join('\n'))
        }
      } else if (picker.action === 'save-trajectory') {
        await saveServerTrajectory(paths[0], useStore.getState().trajectory.toJSON())
      } else if (picker.action === 'load-terrain') {
        const fileName = paths[0].split(/[\\/]/).pop() ?? paths[0]
        const data = await loadServerTerrain(paths[0])
        const group = await loadTerrain(data, fileName)
        onLoadTerrain(group)
        setTerrainName(fileName)
      } else if (picker.action === 'save-terrain') {
        if (!terrain) return
        await saveServerTerrain(paths[0], exportTerrainObj(terrain))
      } else if (picker.action === 'load-robot') {
        await selectServerRobotUrdf(paths[0])
        setShowRobotSelect(false)
        onRobotUploaded(undefined)
      }
    } catch (err) {
      setLoadError(err instanceof Error ? err.message : String(err))
      console.error('Server file action failed:', err)
    }
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
      <button onClick={handleLoadTrajectoryClick}>Load Trajectory</button>
      <button onClick={handleLoadTrajectoryFolderClick}>Load Trajectory Folder</button>
      {loadedTrajectories.length > 1 && (
        <div
          ref={trajectoryListRef}
          style={{
            ...trajectoryListStyle,
            width: listSize.width !== null ? `${listSize.width}px` : '100%',
            height: listSize.height !== null ? `${listSize.height}px` : undefined,
          }}
        >
          <span style={{ fontSize: '11px', color: '#88aacc', flexShrink: 0 }}>Loaded trajectories:</span>
          <select
            aria-label="Loaded trajectories"
            value={selectedTrajectoryIndex}
            onChange={handleTrajectorySelect}
            size={Math.min(loadedTrajectories.length, 6)}
            style={{ width: '100%', flex: 1, minHeight: 0 }}
          >
            {loadedTrajectories.map((entry, index) => (
              <option key={`${entry.name}-${index}`} value={index}>{entry.name}</option>
            ))}
          </select>
          <div
            aria-label="Resize trajectory list width"
            onPointerDown={startListResize('x')}
            style={listRightHandleStyle}
          />
          <div
            aria-label="Resize trajectory list height"
            onPointerDown={startListResize('y')}
            style={listBottomHandleStyle}
          />
          <div
            aria-label="Resize trajectory list"
            title="Drag to resize width and height"
            onPointerDown={startListResize('xy')}
            style={listCornerHandleStyle}
          />
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
      {serverPicker && (
        <ServerFileModal
          title={serverPicker.title}
          mode={serverPicker.mode}
          extensions={serverPicker.extensions}
          defaultFileName={serverPicker.defaultFileName}
          onConfirm={handleServerPickerConfirm}
          onCancel={() => setServerPicker(null)}
        />
      )}
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
  position: 'relative',
  display: 'flex',
  flexDirection: 'column',
  gap: '4px',
  padding: '6px 10px 10px 6px',
  background: '#2a2a4e',
  borderRadius: '4px',
  flexShrink: 0,
}

const listRightHandleStyle: React.CSSProperties = {
  position: 'absolute',
  top: 0,
  right: 0,
  bottom: '12px',
  width: '6px',
  cursor: 'ew-resize',
  touchAction: 'none',
}

const listBottomHandleStyle: React.CSSProperties = {
  position: 'absolute',
  left: 0,
  right: '12px',
  bottom: 0,
  height: '6px',
  cursor: 'ns-resize',
  touchAction: 'none',
}

const listCornerHandleStyle: React.CSSProperties = {
  position: 'absolute',
  right: '2px',
  bottom: '2px',
  width: '12px',
  height: '12px',
  cursor: 'nwse-resize',
  touchAction: 'none',
  background: 'repeating-linear-gradient(-45deg, transparent 0 2px, #88aacc 2px 3.5px)',
  clipPath: 'polygon(100% 0, 100% 100%, 0 100%)',
  opacity: 0.75,
}
