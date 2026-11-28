import { useCallback, useEffect, useState } from 'react'
import { listServerDir, type ServerFsEntry } from '../api/client'

export type ServerFileMode = 'open-file' | 'open-files' | 'open-dir' | 'save-file'

interface ServerFileModalProps {
  title: string
  mode: ServerFileMode
  extensions?: string[]
  defaultFileName?: string
  onConfirm: (paths: string[]) => void
  onCancel: () => void
}

let lastVisitedDir = ''

export function ServerFileModal({
  title,
  mode,
  extensions,
  defaultFileName = '',
  onConfirm,
  onCancel,
}: ServerFileModalProps) {
  const [currentDir, setCurrentDir] = useState(lastVisitedDir)
  const [pathInput, setPathInput] = useState(lastVisitedDir)
  const [parentDir, setParentDir] = useState<string | null>(null)
  const [entries, setEntries] = useState<ServerFsEntry[]>([])
  const [selectedPaths, setSelectedPaths] = useState<string[]>([])
  const [fileName, setFileName] = useState(defaultFileName)
  const [loading, setLoading] = useState(false)
  const [error, setError] = useState('')

  const browse = useCallback(async (targetPath?: string) => {
    setLoading(true)
    setError('')
    try {
      const res = await listServerDir(targetPath, mode === 'open-dir' ? undefined : extensions)
      setCurrentDir(res.path)
      setPathInput(res.path)
      setParentDir(res.parent)
      setEntries(mode === 'open-dir' ? res.entries.filter(e => e.is_dir) : res.entries)
      setSelectedPaths([])
      lastVisitedDir = res.path
    } catch (err) {
      setError(err instanceof Error ? err.message : String(err))
    } finally {
      setLoading(false)
    }
  }, [extensions, mode])

  useEffect(() => {
    void browse(lastVisitedDir || undefined)
  }, [browse])

  useEffect(() => {
    const onKeyDown = (e: KeyboardEvent) => {
      if (e.key === 'Escape') {
        e.stopPropagation()
        onCancel()
      }
    }
    window.addEventListener('keydown', onKeyDown, true)
    return () => window.removeEventListener('keydown', onKeyDown, true)
  }, [onCancel])

  const handleEntryClick = (entry: ServerFsEntry, e: React.MouseEvent) => {
    if (entry.is_dir) {
      if (mode === 'open-dir') {
        setSelectedPaths([entry.path])
      }
      return
    }
    if (mode === 'save-file') {
      setFileName(entry.name)
      setSelectedPaths([entry.path])
      return
    }
    if (mode === 'open-files' && (e.ctrlKey || e.metaKey)) {
      setSelectedPaths(prev => (
        prev.includes(entry.path) ? prev.filter(p => p !== entry.path) : [...prev, entry.path]
      ))
    } else if (mode === 'open-files' && e.shiftKey && selectedPaths.length > 0) {
      const fileEntries = entries.filter(item => !item.is_dir)
      const lastSelected = selectedPaths[selectedPaths.length - 1]
      const i1 = fileEntries.findIndex(item => item.path === lastSelected)
      const i2 = fileEntries.findIndex(item => item.path === entry.path)
      if (i1 !== -1 && i2 !== -1) {
        const [start, end] = i1 < i2 ? [i1, i2] : [i2, i1]
        setSelectedPaths(fileEntries.slice(start, end + 1).map(item => item.path))
        return
      }
      setSelectedPaths([entry.path])
    } else {
      setSelectedPaths([entry.path])
    }
  }

  const handleEntryDoubleClick = (entry: ServerFsEntry) => {
    if (entry.is_dir) {
      void browse(entry.path)
    } else if (mode === 'open-file' || mode === 'open-files') {
      onConfirm([entry.path])
    } else if (mode === 'save-file') {
      setFileName(entry.name)
    }
  }

  const handleConfirm = () => {
    if (mode === 'open-dir') {
      onConfirm([selectedPaths[0] || currentDir])
      return
    }
    if (mode === 'save-file') {
      const trimmed = fileName.trim()
      if (!trimmed) return
      const fullPath = trimmed.startsWith('/')
        ? trimmed
        : `${currentDir.replace(/\/+$/, '')}/${trimmed}`
      onConfirm([fullPath])
      return
    }
    if (selectedPaths.length > 0) {
      onConfirm(selectedPaths)
    }
  }

  const canConfirm = mode === 'open-dir'
    ? Boolean(currentDir)
    : mode === 'save-file'
      ? Boolean(currentDir && fileName.trim())
      : selectedPaths.length > 0

  const confirmLabel = mode === 'save-file'
    ? 'Save'
    : mode === 'open-dir'
      ? (selectedPaths.length > 0 ? 'Select Folder' : 'Use Current Folder')
      : selectedPaths.length > 1
        ? `Open (${selectedPaths.length})`
        : 'Open'

  return (
    <div style={backdropStyle} onClick={onCancel}>
      <div style={modalStyle} onClick={e => e.stopPropagation()}>
        <div style={headerStyle}>
          <span style={{ fontWeight: 'bold', fontSize: '13px', color: '#eef' }}>{title} (Server)</span>
          <button onClick={onCancel} style={smallBtnStyle}>✕</button>
        </div>

        <form
          onSubmit={e => {
            e.preventDefault()
            void browse(pathInput)
          }}
          style={{ display: 'flex', gap: '4px' }}
        >
          <button
            type="button"
            disabled={!parentDir || loading}
            onClick={() => parentDir && void browse(parentDir)}
            style={smallBtnStyle}
            title="Go to parent directory"
          >
            ↑ Up
          </button>
          <input
            type="text"
            value={pathInput}
            onChange={e => setPathInput(e.target.value)}
            style={pathInputStyle}
            placeholder="/path/on/server"
          />
          <button type="submit" disabled={loading} style={smallBtnStyle}>Go</button>
        </form>

        {error && <div style={{ color: '#f66', fontSize: '11px' }}>{error}</div>}

        <div style={listStyle}>
          {loading ? (
            <div style={{ padding: '12px', color: '#889', fontSize: '12px' }}>Loading...</div>
          ) : entries.length === 0 ? (
            <div style={{ padding: '12px', color: '#889', fontSize: '12px' }}>
              {extensions && extensions.length > 0
                ? `No folders or ${extensions.join(' / ')} files in this directory.`
                : 'Empty directory.'}
            </div>
          ) : (
            entries.map(entry => {
              const selected = selectedPaths.includes(entry.path)
              return (
                <div
                  key={entry.path}
                  onClick={e => handleEntryClick(entry, e)}
                  onDoubleClick={() => handleEntryDoubleClick(entry)}
                  style={{
                    ...rowStyle,
                    background: selected ? '#355480' : 'transparent',
                    color: entry.is_dir ? '#9cd' : '#dde',
                  }}
                >
                  <span style={{ overflow: 'hidden', textOverflow: 'ellipsis', whiteSpace: 'nowrap' }}>
                    {entry.is_dir ? '📁 ' : '📄 '}{entry.name}
                  </span>
                  {!entry.is_dir && typeof entry.size === 'number' && (
                    <span style={{ fontSize: '10px', color: '#889', flexShrink: 0, marginLeft: '8px' }}>
                      {formatSize(entry.size)}
                    </span>
                  )}
                </div>
              )
            })
          )}
        </div>

        {mode === 'open-files' && (
          <div style={{ fontSize: '10px', color: '#889' }}>
            Tip: Double-click a folder to enter; Ctrl/Shift+click to select multiple files.
          </div>
        )}

        {mode === 'save-file' && (
          <div style={{ display: 'flex', alignItems: 'center', gap: '6px' }}>
            <span style={{ fontSize: '12px', color: '#aab' }}>File name:</span>
            <input
              type="text"
              value={fileName}
              onChange={e => setFileName(e.target.value)}
              onKeyDown={e => { if (e.key === 'Enter') handleConfirm() }}
              style={pathInputStyle}
            />
          </div>
        )}

        <div style={{ display: 'flex', justifyContent: 'flex-end', gap: '6px' }}>
          <button onClick={onCancel} style={smallBtnStyle}>Cancel</button>
          <button onClick={handleConfirm} disabled={!canConfirm} style={primaryBtnStyle}>
            {confirmLabel}
          </button>
        </div>
      </div>
    </div>
  )
}

function formatSize(bytes: number): string {
  if (bytes < 1024) return `${bytes} B`
  if (bytes < 1024 * 1024) return `${(bytes / 1024).toFixed(1)} KB`
  return `${(bytes / (1024 * 1024)).toFixed(1)} MB`
}

const backdropStyle: React.CSSProperties = {
  position: 'fixed',
  inset: 0,
  background: 'rgba(0, 0, 0, 0.65)',
  display: 'flex',
  alignItems: 'center',
  justifyContent: 'center',
  zIndex: 1000,
}

const modalStyle: React.CSSProperties = {
  width: 'min(640px, 92vw)',
  maxHeight: '80vh',
  background: '#1e1e32',
  border: '1px solid #445',
  borderRadius: '6px',
  padding: '12px',
  display: 'flex',
  flexDirection: 'column',
  gap: '8px',
  color: '#ddd',
  fontFamily: 'monospace',
}

const headerStyle: React.CSSProperties = {
  display: 'flex',
  alignItems: 'center',
  justifyContent: 'space-between',
}

const pathInputStyle: React.CSSProperties = {
  flex: 1,
  background: '#121220',
  color: '#eef',
  border: '1px solid #445',
  borderRadius: '3px',
  padding: '4px 6px',
  fontFamily: 'monospace',
  fontSize: '12px',
}

const listStyle: React.CSSProperties = {
  flex: 1,
  minHeight: '260px',
  maxHeight: '45vh',
  overflowY: 'auto',
  background: '#141424',
  border: '1px solid #334',
  borderRadius: '4px',
  padding: '4px',
}

const rowStyle: React.CSSProperties = {
  display: 'flex',
  alignItems: 'center',
  justifyContent: 'space-between',
  padding: '4px 6px',
  borderRadius: '3px',
  cursor: 'pointer',
  fontSize: '12px',
  userSelect: 'none',
}

const smallBtnStyle: React.CSSProperties = {
  padding: '4px 8px',
  background: '#2c2c44',
  color: '#dde',
  border: '1px solid #445',
  borderRadius: '3px',
  cursor: 'pointer',
  fontFamily: 'monospace',
  fontSize: '12px',
}

const primaryBtnStyle: React.CSSProperties = {
  ...smallBtnStyle,
  background: '#2f5d96',
  borderColor: '#4a80c4',
  color: '#fff',
}
