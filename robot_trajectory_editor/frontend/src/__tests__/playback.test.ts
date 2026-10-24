import { describe, expect, it } from 'vitest'
import { interpolatedFrame, playbackPosition, slerp } from '../state/playback'
import { Trajectory } from '../state/Trajectory'

function trajectory(framerate: number) {
  const half = Math.SQRT1_2
  return Trajectory.fromJSON({ framerate, joint_names: ['joint'],
    joint_pos: [[0], [1], [3]], base_pos_w: [[0, 0, 1], [1, 0, 1], [1, 2, 1]],
    base_quat_w: [[1, 0, 0, 0], [half, 0, 0, half], [0, 0, 0, 1]] })
}

describe('playback', () => {
  it('follows the trajectory frame rate in real time', () => {
    expect(playbackPosition(0, 1, 90)).toBe(90)
    expect(playbackPosition(10, .5, 30)).toBe(25)
    expect(playbackPosition(5, -1, 30)).toBe(5)
  })

  it('draws the pose between neighbouring frames without changing them', () => {
    const t = trajectory(90)
    const before = t.toJSON()
    const pose = interpolatedFrame(t, 1.25)
    expect(pose.jointPos[0]).toBeCloseTo(1.5)
    expect([...pose.basePoseW]).toEqual([1, 0.5, 1])
    // A quarter of the way from a 90 degree to a 180 degree yaw: 112.5 degrees.
    const angle = 2 * Math.atan2(pose.baseQuatW[3], pose.baseQuatW[0])
    expect(angle).toBeCloseTo(Math.PI * 112.5 / 180)
    expect(t.toJSON()).toEqual(before)
    expect([...interpolatedFrame(t, 2).jointPos]).toEqual([3])
  })

  it('interpolates orientation along the shorter arc', () => {
    const q = slerp(new Float32Array([1, 0, 0, 0]), new Float32Array([-1, 0, 0, 0]), .5)
    expect(Math.abs(q[0])).toBeCloseTo(1)
  })
})
