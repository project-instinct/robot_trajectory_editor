import { optimizeTrajectory } from '../api/client'
import { getViewport } from '../three/viewportContext'
import { exportTerrainObj } from '../three/Terrain'
import { Trajectory } from './Trajectory'
import { useStore } from './store'

/** Apply only a validated result for the exact trajectory and scene submitted. */
export async function runTrajectoryOptimize(urdfPath?: string, isCurrentRobot = () => true, friction = 0.7) {
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
  const result = await optimizeTrajectory(snapshot, keyframes, terrainObj, urdfPath, friction)
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
  for (const frame of keyframes) {
    for (const field of ['joint_pos', 'base_pos_w', 'base_quat_w'] as const) {
      if (JSON.stringify(result.trajectory[field][frame]) !== JSON.stringify(snapshot[field][frame])) {
        throw new Error(`Trajectory Optimize changed fixed keyframe ${frame}; result discarded.`)
      }
    }
  }
  processed.keyframes = new Set(keyframes)
  current.applyProcessedTrajectory(processed)
  return result
}
