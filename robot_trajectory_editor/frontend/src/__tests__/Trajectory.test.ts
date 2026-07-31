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
    // keyframe anchor preserved
    expect(t.jointPos[5]).toBeCloseTo(1.57)
  })

  it('should remove high-frequency jitter but keep the low-frequency trend', () => {
    const t = makeTrajectory(60, 1)
    for (let f = 0; f < 60; f++) {
      // linear trend + frame-to-frame (Nyquist) jitter
      t.setJointValue(f, 0, f * 0.01 + (f % 2 === 0 ? 0.1 : -0.1))
    }

    const channel: ChannelKind = { kind: 'joint', index: 0 }
    t.smoothRange(0, 59, channel)

    // away from the tapered edges the jitter is gone and the trend is intact
    for (let f = 8; f < 52; f++) {
      expect(t.jointPos[f]).toBeCloseTo(f * 0.01, 2)
    }
  })

  it('should preserve the segment shape instead of interpolating between endpoints', () => {
    const t = makeTrajectory(10, 1)
    t.setJointValue(0, 0, 0)
    t.setJointValue(9, 0, 1)
    for (let f = 1; f < 9; f++) t.setJointValue(f, 0, 42) // constant interior offset

    const channel: ChannelKind = { kind: 'joint', index: 0 }
    t.smoothRange(0, 9, channel)

    expect(t.jointPos[0]).toBeCloseTo(0)
    expect(t.jointPos[9]).toBeCloseTo(1)
    // interior stays near its original value (42) — the old buggy behavior
    // replaced it with a start-to-end interpolation (~0.44 at frame 4)
    expect(t.jointPos[4]).toBeGreaterThan(40)
  })

  it('should only smooth the selected segment and channel', () => {
    const t = makeTrajectory(20, 2)
    for (let f = 0; f < 20; f++) {
      t.setJointValue(f, 0, f % 2 === 0 ? 1 : -1) // jitter on joint 0
      t.setJointValue(f, 1, f * 0.05)             // reference data on joint 1
    }
    const joint0Before = Array.from({ length: 20 }, (_, f) => t.jointPos[f * 2])
    const joint1Before = Array.from({ length: 20 }, (_, f) => t.jointPos[f * 2 + 1])

    t.smoothRange(2, 17, { kind: 'joint', index: 0 })

    // frames outside the segment and the segment endpoints are untouched
    for (const f of [0, 1, 2, 17, 18, 19]) {
      expect(t.jointPos[f * 2]).toBeCloseTo(joint0Before[f])
    }
    // fully-smoothed interior (past the edge taper): jitter essentially gone
    expect(Math.abs(t.jointPos[9 * 2])).toBeLessThan(0.05)
    expect(Math.abs(t.jointPos[10 * 2])).toBeLessThan(0.05)
    // the other channel is untouched everywhere
    for (let f = 0; f < 20; f++) {
      expect(t.jointPos[f * 2 + 1]).toBeCloseTo(joint1Before[f])
    }
  })

  it('should keep unit quaternions when smoothing base orientation', () => {
    const t = makeTrajectory(10, 1)
    for (let f = 0; f < 10; f++) {
      // alternate hemispheres to test continuity handling
      const s = f % 2 === 0 ? 1 : -1
      t.setBasePose(f, new Float32Array([0, 0, 1]), new Float32Array([s, 0, 0, 0]))
    }
    const channel: ChannelKind = { kind: 'baseQuat', axis: 2 }
    t.smoothRange(0, 9, channel)
    for (let f = 0; f < 10; f++) {
      const q = t.getFrame(f).baseQuatW
      const n = Math.hypot(q[0], q[1], q[2], q[3])
      expect(n).toBeCloseTo(1, 3)
    }
  })

  it('should derive frame count from base data when there are no joints', () => {
    const t = new Trajectory(
      30,
      [],
      new Float32Array(0),
      new Float32Array([0, 0, 1, 1, 0, 1]),
      new Float32Array([1, 0, 0, 0, 1, 0, 0, 0]),
    )
    expect(t.frameCount).toBe(2)
    expect(t.jointCount).toBe(0)
  })

  it('should set a single channel value', () => {
    const t = makeTrajectory(5, 2)
    t.setChannelValue(2, { kind: 'joint', index: 1 }, 0.75)
    expect(t.jointPos[2 * 2 + 1]).toBeCloseTo(0.75)
    t.setChannelValue(2, { kind: 'basePos', axis: 2 }, 1.5)
    expect(t.basePoseW[2 * 3 + 2]).toBeCloseTo(1.5)
  })

  it('should fill only the selected channel over the range', () => {
    const t = makeTrajectory(5, 2)
    t.fillChannelRange(1, 3, { kind: 'joint', index: 0 }, 9.99)

    for (let f = 1; f <= 3; f++) {
      expect(t.jointPos[f * 2]).toBeCloseTo(9.99)
      // other channels untouched
      expect(t.jointPos[f * 2 + 1]).toBeCloseTo(f * 0.1 + 0.01)
      expect(t.basePoseW[f * 3]).toBeCloseTo(f * 0.01)
    }
    // frames outside the range untouched
    expect(t.jointPos[0]).toBeCloseTo(0)
    expect(t.jointPos[4 * 2]).toBeCloseTo(0.4)
  })

  it('should fill one base-orientation axis while preserving the others', () => {
    const t = makeTrajectory(4, 1)
    t.setQuatEuler(2, 0.3, -0.2, 0.1)
    t.fillChannelRange(1, 2, { kind: 'baseQuat', axis: 1 }, 0.5)

    for (const f of [1, 2]) {
      const [roll, pitch, yaw] = t.getQuatEuler(f)
      expect(pitch).toBeCloseTo(0.5)
      // roll/yaw keep their per-frame values
      expect(roll).toBeCloseTo(f === 2 ? 0.3 : 0)
      expect(yaw).toBeCloseTo(f === 2 ? 0.1 : 0)
      const q = t.getFrame(f).baseQuatW
      expect(Math.hypot(q[0], q[1], q[2], q[3])).toBeCloseTo(1)
    }
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
