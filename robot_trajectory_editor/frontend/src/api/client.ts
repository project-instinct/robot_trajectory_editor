import type { TrajectoryJSON } from '../state/Trajectory'

const BASE = '/api'

export async function getConfig(): Promise<{
  nudge_step: number
  frame_step: number
  file_explorer_side?: 'client' | 'server'
}> {
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

export interface TrajectoryOptimizeResult {
  trajectory: TrajectoryJSON
  residuals: Record<string, number>
  tolerances: Record<string, number>
  validation_samples_per_interval: number
  iterations: number
  friction?: number
  max_effort_scale?: number
  smoothing_hz?: number
  glitch_speed_scale?: number
  repaired_joint_samples?: number
  seconds?: number
  success?: boolean
  error?: string
  /** Plain-text summary of the run, meant to be copied verbatim. */
  report?: string
}

/** User parameters of one Trajectory Optimize request. */
export interface TrajectoryOptimizeOptions {
  /** Terrain friction coefficient μ. */
  friction: number
  /** Multiplier on the URDF actuator effort limits (at least 1). */
  maxEffortScale: number
  /** Low-pass cutoff of the target motion in Hz; 0 keeps every frame's detail. */
  smoothingHz: number
  /** Joint jumps faster than this multiple of the URDF velocity limit are repaired; 0 disables. */
  glitchSpeedScale: number
}

export const DEFAULT_TRAJECTORY_OPTIMIZE_OPTIONS: TrajectoryOptimizeOptions = {
  friction: 0.7, maxEffortScale: 1, smoothingHz: 8, glitchSpeedScale: 1,
}

export async function optimizeTrajectory(
  trajectory: TrajectoryJSON, keyframes: number[], terrainObj: string | null,
  urdfPath?: string, options: TrajectoryOptimizeOptions = DEFAULT_TRAJECTORY_OPTIMIZE_OPTIONS,
): Promise<TrajectoryOptimizeResult> {
  const response = await fetch(`${BASE}/trajectory/optimize`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
      trajectory, keyframes, terrain_obj: terrainObj, urdf_path: urdfPath,
      friction: options.friction, max_effort_scale: options.maxEffortScale,
      smoothing_hz: options.smoothingHz, glitch_speed_scale: options.glitchSpeedScale,
    }),
  })
  const result = await response.json()
  if (!response.ok && !result.trajectory) throw new Error(result.error || 'Trajectory Optimize failed')
  return result
}

export interface ServerFsEntry {
  name: string
  path: string
  is_dir: boolean
  size?: number
}

export interface ServerFsListResponse {
  path: string
  parent: string | null
  entries: ServerFsEntry[]
}

export async function listServerDir(path?: string, exts?: string[]): Promise<ServerFsListResponse> {
  const params = new URLSearchParams()
  if (path) params.set('path', path)
  if (exts && exts.length > 0) params.set('ext', exts.join(','))
  const qs = params.toString()
  const res = await fetch(`${BASE}/fs/list${qs ? `?${qs}` : ''}`)
  const body = await res.json().catch(() => ({}))
  if (!res.ok) throw new Error(body.error || 'Failed to list server directory')
  return body
}

export async function loadServerTrajectories(
  request: { paths: string[] } | { directory: string },
): Promise<{ trajectories: { path: string; name: string; data: TrajectoryJSON }[]; errors: string[] }> {
  const res = await fetch(`${BASE}/fs/trajectory/load`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(request),
  })
  const body = await res.json().catch(() => ({}))
  if (!res.ok) throw new Error(body.error || 'Failed to load server trajectories')
  return body
}

export async function saveServerTrajectory(path: string, trajectory: TrajectoryJSON): Promise<{ path: string }> {
  const res = await fetch(`${BASE}/fs/trajectory/save`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ path, trajectory }),
  })
  const body = await res.json().catch(() => ({}))
  if (!res.ok) throw new Error(body.error || 'Failed to save trajectory on server')
  return body
}

export async function loadServerTerrain(path: string): Promise<string | ArrayBuffer> {
  const res = await fetch(`${BASE}/fs/terrain?path=${encodeURIComponent(path)}`)
  if (!res.ok) {
    const body = await res.json().catch(() => ({}))
    throw new Error(body.error || 'Failed to load server terrain')
  }
  return path.toLowerCase().endsWith('.stl') ? res.arrayBuffer() : res.text()
}

export async function saveServerTerrain(path: string, content: string): Promise<{ path: string }> {
  const res = await fetch(`${BASE}/fs/terrain/save`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ path, content }),
  })
  const body = await res.json().catch(() => ({}))
  if (!res.ok) throw new Error(body.error || 'Failed to save terrain on server')
  return body
}

export async function selectServerRobotUrdf(path: string): Promise<{ urdf_path: string }> {
  const res = await fetch(`${BASE}/fs/robot/select`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ path }),
  })
  const body = await res.json().catch(() => ({}))
  if (!res.ok) throw new Error(body.error || 'Failed to select server URDF')
  return body
}
