import { describe, it, expect } from 'vitest'
import { Trajectory, type ChannelKind } from '../state/Trajectory'

function makeTrajectory(frameCount = 10, jointCount = 3): Trajectory {
  const jointNames = Array.from({ length: jointCount }, (_, i) => `joint_${i}`)
  const jointPos = new Float32Array(frameCount * jointCount)
  const basePoseW = new Float32Array(frameCount * 3)
  const baseQuatW = new Float32Array(frameCount * 4)

  for (let f = 0; f < frameCount; f++) {
    for (let j = 0; j < jointCount; j++) {
      jointPos[f * jointCount + j] = f * 0.1 + j * 0.01
    }
    basePoseW[f * 3] = f * 0.01
    basePoseW[f * 3 + 1] = 0
    basePoseW[f * 3 + 2] = 1.0
    baseQuatW[f * 4] = 1
  }

  return new Trajectory(30, jointNames, jointPos, basePoseW, baseQuatW)
}

describe('Trajectory', () => {
  it('should create empty trajectory', () => {
    const t = Trajectory.empty()
    expect(t.frameCount).toBe(0)
    expect(t.jointCount).toBe(0)
  })

  it('should roundtrip JSON', () => {
    const t = makeTrajectory(5, 2)
    const json = t.toJSON()
    expect(json.framerate).toBe(30)
    expect(json.joint_names.length).toBe(2)
    expect(json.joint_pos.length).toBe(5)

    const t2 = Trajectory.fromJSON(json)
    expect(t2.framerate).toBe(t.framerate)
    expect(t2.frameCount).toBe(t.frameCount)
    expect(t2.jointCount).toBe(t.jointCount)
    for (let i = 0; i < t.jointPos.length; i++) {
      expect(t2.jointPos[i]).toBeCloseTo(t.jointPos[i])
    }
  })

  it('should get and set frame', () => {
    const t = makeTrajectory(5, 2)
    t.setJointValue(2, 0, 0.5)
    const frame = t.getFrame(2)
    expect(frame.jointPos[0]).toBeCloseTo(0.5)
    expect(frame.basePoseW[0]).toBeCloseTo(0.02)
  })

  it('should set base pose', () => {
    const t = makeTrajectory(5, 1)
    t.setBasePose(1, new Float32Array([1, 2, 3]), new Float32Array([0, 0, 0, 1]))
    const frame = t.getFrame(1)
    expect(frame.basePoseW[0]).toBeCloseTo(1)
    expect(frame.basePoseW[1]).toBeCloseTo(2)
    expect(frame.basePoseW[2]).toBeCloseTo(3)
  })

  it('should fill range', () => {
    const t = makeTrajectory(5, 2)
    t.setJointValue(0, 0, 9.99)
    const sourceState = t.getFrame(0)
    t.fillRange(1, 3, sourceState)

    for (let f = 1; f <= 3; f++) {
      const frame = t.getFrame(f)
      expect(frame.jointPos[0]).toBeCloseTo(9.99)
      expect(frame.basePoseW[0]).toBeCloseTo(0)
    }
  })

  it('should smooth range with keyframes', () => {
    const t = makeTrajectory(10, 1)
    t.insertKeyframe(0)
    t.insertKeyframe(5)
    t.insertKeyframe(9)
    t.setJointValue(0, 0, 0)
    t.setJointValue(5, 0, 1.57)
    t.setJointValue(9, 0, 0)

    const channel: ChannelKind = { kind: 'joint', index: 0 }
    t.smoothRange(0, 9, channel)

    for (let f = 0; f < 10; f++) {
      const v = t.jointPos[f]
      expect(isNaN(v)).toBe(false)
      expect(isFinite(v)).toBe(true)
    }

    expect(t.jointPos[0]).toBeCloseTo(0)
    expect(t.jointPos[9]).toBeCloseTo(0)
  })

  it('should preserve keyframes', () => {
    const t = makeTrajectory(5, 1)
    t.insertKeyframe(2)
    expect(t.hasKeyframe(2)).toBe(true)
    expect(t.hasKeyframe(0)).toBe(false)
    t.removeKeyframe(2)
    expect(t.hasKeyframe(2)).toBe(false)
  })
})
