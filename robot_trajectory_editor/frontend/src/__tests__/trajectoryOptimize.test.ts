import { beforeEach, describe, expect, it, vi } from 'vitest'
import * as THREE from 'three'
import { optimizeTrajectory } from '../api/client'
import { runTrajectoryOptimize } from '../state/trajectoryOptimize'
import { Trajectory } from '../state/Trajectory'
import { useStore } from '../state/store'
import { getViewport } from '../three/viewportContext'

vi.mock('../api/client', () => ({ optimizeTrajectory: vi.fn() }))
vi.mock('../three/viewportContext', () => ({ getViewport: vi.fn() }))

const api = vi.mocked(optimizeTrajectory)
let terrain: THREE.Group

function response() {
  const data = useStore.getState().trajectory.toJSON()
  data.base_pos_w[0][2] = 3
  data.base_pos_w[2][2] = 4
  return { trajectory: data, residuals: { dynamics: 0 }, tolerances: { dynamics: 1e-3 },
    validation_samples_per_interval: 17, iterations: 1 }
}

beforeEach(() => {
  vi.resetAllMocks()
  const trajectory = Trajectory.fromJSON({ framerate: 30, joint_names: ['joint'],
    joint_pos: [[0], [1], [2]], base_pos_w: [[0, 0, 1], [0, 0, 1], [0, 0, 1]],
    base_quat_w: [[1, 0, 0, 0], [1, 0, 0, 0], [1, 0, 0, 0]],
  })
  trajectory.keyframes.add(1)
  useStore.setState({ trajectory, trajectoryVersion: 0, undoStack: [], currentFrame: 1,
    robotModelLoaded: true, isPlaying: false, segmentStart: 1, segmentEnd: 1,
    baseTransform: null, baseTransformSource: null, baseTransformOrigin: null })
  terrain = new THREE.Group()
  vi.mocked(getViewport).mockReturnValue({ robotModel: {}, terrainGroup: terrain } as ReturnType<typeof getViewport>)
})

describe('Trajectory Optimize', () => {
  it('submits the configured terrain friction', async () => {
    api.mockResolvedValue(response())
    await runTrajectoryOptimize('robot.urdf', () => true, 0.5, 1.25)
    expect(api.mock.calls[0][4]).toBe(0.5)
    expect(api.mock.calls[0][5]).toBe(1.25)
  })

  it('submits all frames and anchors, applies once, preserves cursor and supports undo', async () => {
    const before = useStore.getState().trajectory.clone()
    api.mockResolvedValue(response())
    await runTrajectoryOptimize('robot.urdf')
    expect(api).toHaveBeenCalledWith(before.toJSON(), [1], null, 'robot.urdf', 0.7, 1)
    const after = useStore.getState()
    expect(after.trajectory.basePoseW[2]).toBe(3)
    expect(after.trajectory.basePoseW[8]).toBe(4)
    expect([...after.trajectory.keyframes]).toEqual([1])
    expect(after.currentFrame).toBe(1)
    expect(after.undoStack).toHaveLength(1)
    expect(after.trajectoryVersion).toBe(1)
    after.undo()
    expect(useStore.getState().trajectory.toJSON()).toEqual(before.toJSON())
  })

  it('previews a failed candidate and restores it on dump', async () => {
    const before = useStore.getState().trajectory.toJSON()
    const result = { ...response(), success: false, error: 'Validation failed: dynamics residual 1' }
    api.mockResolvedValue(result)
    const outcome = await runTrajectoryOptimize()
    expect(outcome.preview).toBe(true)
    expect(useStore.getState().trajectory.basePoseW[2]).toBe(3)
    expect(useStore.getState().undoStack).toHaveLength(1)
    useStore.getState().undo()
    expect(useStore.getState().trajectory.toJSON()).toEqual(before)
  })

  it('leaves data and undo history unchanged on solver failure', async () => {
    const before = useStore.getState().trajectory.toJSON()
    api.mockRejectedValue(new Error('dynamics residual 0.1'))
    await expect(runTrajectoryOptimize()).rejects.toThrow('dynamics residual 0.1')
    expect(useStore.getState().trajectory.toJSON()).toEqual(before)
    expect(useStore.getState().undoStack).toHaveLength(0)
  })

  it('rejects results after edits made while waiting', async () => {
    api.mockImplementation(async () => {
      const result = response()
      useStore.getState().trajectory.jointPos[0] = 8
      return result
    })
    await expect(runTrajectoryOptimize()).rejects.toThrow('changed during Trajectory Optimize')
    expect(useStore.getState().trajectory.jointPos[0]).toBe(8)
    expect(useStore.getState().undoStack).toHaveLength(0)
  })

  it('rejects results after a keyframe or model selection changes', async () => {
    api.mockImplementation(async () => {
      const result = response()
      useStore.getState().trajectory.keyframes.add(2)
      return result
    })
    await expect(runTrajectoryOptimize()).rejects.toThrow('changed during Trajectory Optimize')
    api.mockResolvedValue(response())
    await expect(runTrajectoryOptimize('robot.urdf', () => false)).rejects.toThrow('changed during Trajectory Optimize')
    expect(useStore.getState().undoStack).toHaveLength(0)
  })

  it('exports the currently transformed terrain', async () => {
    const mesh = new THREE.Mesh(new THREE.PlaneGeometry(1, 1))
    mesh.position.z = 2
    terrain.add(mesh)
    api.mockResolvedValue(response())
    await runTrajectoryOptimize()
    expect(api.mock.calls[0][2]).toContain('v -0.5 0.5 2')
  })

  it('rejects a result that changes a fixed configuration', async () => {
    const result = response()
    result.trajectory.joint_pos[1][0] = 99
    api.mockResolvedValue(result)
    await expect(runTrajectoryOptimize()).rejects.toThrow('fixed keyframe 1')
    expect(useStore.getState().undoStack).toHaveLength(0)
  })
})
