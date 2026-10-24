import { DEFAULT_TRAJECTORY_OPTIMIZE_OPTIONS, optimizeTrajectory, type TrajectoryOptimizeOptions } from '../api/client'
import { getViewport } from '../three/viewportContext'
import { exportTerrainObj } from '../three/Terrain'
import { Trajectory } from './Trajectory'
import { useStore } from './store'

/** Apply only a validated result for the exact trajectory and scene submitted. */
export async function runTrajectoryOptimize(urdfPath?: string, isCurrentRobot = () => true,
                                           options: TrajectoryOptimizeOptions = DEFAULT_TRAJECTORY_OPTIMIZE_OPTIONS) {
  const store = useStore.getState()
  const viewport = getViewport()
  if (!viewport?.robotModel || !store.robotModelLoaded) throw new Error('Load a robot before Trajectory Optimize.')
  const source = store.trajectory
  const snapshot = source.toJSON()
  const keyframes = [...source.keyframes].sort((a, b) => a - b)
  const robot = viewport.robotModel
  const terrain = () => viewport.terrainGroup.children.length ? exportTerrainObj(viewport.terrainGroup) : null
  const terrainObj = terrain()
  store.setIsPlaying(false)
  const result = await optimizeTrajectory(snapshot, keyframes, terrainObj, urdfPath, options)
  const current = useStore.getState()
  if (current.trajectory !== source || JSON.stringify(source.toJSON()) !== JSON.stringify(snapshot) ||
      JSON.stringify([...source.keyframes].sort((a, b) => a - b)) !== JSON.stringify(keyframes) ||
      viewport.robotModel !== robot || !current.robotModelLoaded || !isCurrentRobot() || terrain() !== terrainObj) {
    throw new Error('Trajectory or scene changed during Trajectory Optimize. Result discarded; run Trajectory Optimize again.')
  }
  const processed = Trajectory.fromJSON(result.trajectory)
  if (processed.frameCount !== source.frameCount || processed.framerate !== source.framerate ||
      JSON.stringify(processed.jointNames) !== JSON.stringify(source.jointNames)) {
    throw new Error('Trajectory Optimize returned incompatible trajectory metadata.')
  }
  // Keyframes are exact. Without keyframes the first and last frames keep
  // everything but their base height, which may move onto the terrain.
  const endpointsOnly = keyframes.length === 0
  const fixedFrames = endpointsOnly ? [0, source.frameCount - 1] : keyframes
  for (const frame of fixedFrames) {
    for (const field of ['joint_pos', 'base_pos_w', 'base_quat_w'] as const) {
      const value = result.trajectory[field][frame]
      const expected = snapshot[field][frame]
      const kept = endpointsOnly && field === 'base_pos_w'
        ? value[0] === expected[0] && value[1] === expected[1]
        : JSON.stringify(value) === JSON.stringify(expected)
      if (!kept) {
        const kind = endpointsOnly ? 'endpoint' : 'keyframe'
        throw new Error(`Trajectory Optimize changed fixed ${kind} ${frame}; result discarded.`)
      }
    }
  }
  processed.keyframes = new Set(keyframes)
  if (result.success === false) {
    // Show the failed candidate as a preview. The original is pushed to the
    // undo stack so "Dump" can restore it and "Keep" leaves one undo step.
    current.beginTrajectoryEdit()
    current.replaceTrajectory(processed)
    return { ...result, preview: true }
  }
  current.applyProcessedTrajectory(processed)
  return { ...result, preview: false }
}
