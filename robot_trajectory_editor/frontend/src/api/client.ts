import type { TrajectoryJSON } from '../state/Trajectory'

const BASE = '/api'

export async function getConfig(): Promise<{ nudge_step: number; frame_step: number }> {
  const res = await fetch(`${BASE}/config`)
  return res.json()
}

export async function getRobotInfo(): Promise<{ has_urdf: boolean; urdf_path: string | null }> {
  const res = await fetch(`${BASE}/robot/info`)
  return res.json()
}

export async function getRobotUrdfUrl(relPath?: string): Promise<string> {
  const params = relPath ? `?path=${encodeURIComponent(relPath)}` : ''
  return `${BASE}/robot/urdf${params}`
}

export async function getRobotUrdfList(): Promise<{ urdfs: { path: string; name: string }[]; root_dir: string }> {
  const res = await fetch(`${BASE}/robot/urdfs`)
  return res.json()
}

export function getAssetUrl(assetPath: string): string {
  return `${BASE}/robot/assets/${assetPath}`
}

export async function uploadRobotFolder(files: File[]): Promise<void> {
  const formData = new FormData()
  for (const f of files) {
    formData.append('files', f, f.webkitRelativePath || f.name)
  }
  await fetch(`${BASE}/robot/upload`, { method: 'POST', body: formData })
}

export async function parseTrajectory(file: File): Promise<TrajectoryJSON> {
  const formData = new FormData()
  formData.append('file', file)
  const res = await fetch(`${BASE}/trajectory/parse`, { method: 'POST', body: formData })
  if (!res.ok) {
    const body = await res.json().catch(() => ({}))
    throw new Error(body.error || 'Failed to parse trajectory')
  }
  return res.json()
}

export async function serializeTrajectory(payload: TrajectoryJSON): Promise<Blob> {
  const res = await fetch(`${BASE}/trajectory/serialize`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(payload),
  })
  if (!res.ok) throw new Error('Failed to serialize trajectory')
  return res.blob()
}

interface SaveFileHandle {
  createWritable: () => Promise<{
    write: (data: Blob) => Promise<void>
    close: () => Promise<void>
  }>
}

interface SaveFilePickerWindow extends Window {
  showSaveFilePicker?: (options: {
    suggestedName: string
    types: { description: string; accept: Record<string, string[]> }[]
  }) => Promise<SaveFileHandle>
}

/** Serialize and save a trajectory through the native picker when supported. */
export async function saveTrajectoryFile(payload: TrajectoryJSON, suggestedName = 'trajectory.npz'): Promise<void> {
  const pickerWindow = window as SaveFilePickerWindow
  if (pickerWindow.showSaveFilePicker) {
    try {
      // Open the picker before awaiting serialization so the browser still
      // considers this part of the user's click or keyboard gesture.
      const handle = await pickerWindow.showSaveFilePicker({
        suggestedName,
        types: [{
          description: 'NumPy trajectory',
          accept: { 'application/octet-stream': ['.npz'] },
        }],
      })
      const blob = await serializeTrajectory(payload)
      const writable = await handle.createWritable()
      await writable.write(blob)
      await writable.close()
    } catch (error) {
      if (error instanceof DOMException && error.name === 'AbortError') return
      throw error
    }
    return
  }

  await downloadTrajectory(payload, suggestedName)
}

/** Serialize and download a trajectory as a file with the given name, without
 *  showing a save picker. Used when several files are produced at once. */
export async function downloadTrajectory(payload: TrajectoryJSON, fileName: string): Promise<void> {
  const blob = await serializeTrajectory(payload)
  const url = URL.createObjectURL(blob)
  const anchor = document.createElement('a')
  anchor.href = url
  anchor.download = fileName
  anchor.click()
  URL.revokeObjectURL(url)
}
