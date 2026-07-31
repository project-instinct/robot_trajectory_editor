import { describe, it, expect, beforeEach } from 'vitest'
import { useStore } from '../state/store'
import { Trajectory } from '../state/Trajectory'

function makeTrajectory(frameCount = 10, jointCount = 2): Trajectory {
  const jointNames = Array.from({ length: jointCount }, (_, i) => `joint_${i}`)
  const jointPos = new Float32Array(frameCount * jointCount)
  const basePoseW = new Float32Array(frameCount * 3)
  const baseQuatW = new Float32Array(frameCount * 4)
  for (let f = 0; f < frameCount; f++) {
    for (let j = 0; j < jointCount; j++) jointPos[f * jointCount + j] = f * 0.1 + j * 0.01
    basePoseW[f * 3 + 2] = 1.0
    baseQuatW[f * 4] = 1
  }
  return new Trajectory(30, jointNames, jointPos, basePoseW, baseQuatW)
}

beforeEach(() => {
  useStore.setState({
    trajectory: makeTrajectory(),
    trajectoryVersion: 0,
    currentFrame: 3,
    selectedChannel: null,
    segmentStart: null,
    segmentEnd: null,
    isPlaying: false,
    nudgeStep: 0.01,
    frameStep: 1,
  })
})

describe('fillRange', () => {
  it('fills only the selected channel over the selected segment with the current value', () => {
    const s = () => useStore.getState()
    s().setSelectedChannel({ kind: 'joint', index: 0 })
    s().setSegment(2, 5)
    // current frame is 3: joint 0 value there is 0.3
    s().fillRange()

    const t = s().trajectory
    for (let f = 2; f <= 5; f++) {
      expect(t.getChannelValue(f, { kind: 'joint', index: 0 })).toBeCloseTo(0.3)
    }
    // outside the segment untouched
    expect(t.getChannelValue(1, { kind: 'joint', index: 0 })).toBeCloseTo(0.1)
    expect(t.getChannelValue(6, { kind: 'joint', index: 0 })).toBeCloseTo(0.6)
    // other channels untouched even inside the segment
    for (let f = 2; f <= 5; f++) {
      expect(t.getChannelValue(f, { kind: 'joint', index: 1 })).toBeCloseTo(f * 0.1 + 0.01)
      expect(t.getChannelValue(f, { kind: 'basePos', axis: 2 })).toBeCloseTo(1.0)
    }
    expect(s().trajectoryVersion).toBe(1)
  })

  it('fills the whole trajectory when no segment is selected', () => {
    const s = () => useStore.getState()
    s().setSelectedChannel({ kind: 'basePos', axis: 2 })
    // give the current frame a distinct value so the fill is observable
    s().trajectory.setChannelValue(3, { kind: 'basePos', axis: 2 }, 2.5)
    s().fillRange()
    for (let f = 0; f < 10; f++) {
      expect(s().trajectory.getChannelValue(f, { kind: 'basePos', axis: 2 })).toBeCloseTo(2.5)
    }
  })

  it('is a no-op without a selected channel', () => {
    const s = () => useStore.getState()
    s().setSegment(2, 5)
    s().fillRange()
    expect(s().trajectoryVersion).toBe(0)
    expect(s().trajectory.getChannelValue(3, { kind: 'joint', index: 0 })).toBeCloseTo(0.3)
  })
})

describe('stepFrame', () => {
  it('moves the current frame by frameStep in both directions', () => {
    const s = () => useStore.getState()
    expect(s().currentFrame).toBe(3)
    s().stepFrame(1)
    expect(s().currentFrame).toBe(4)
    s().stepFrame(-1)
    s().stepFrame(-1)
    expect(s().currentFrame).toBe(2)
  })

  it('respects a custom frameStep from the backend config', () => {
    const s = () => useStore.getState()
    s().setFrameStep(5)
    s().stepFrame(1)
    expect(s().currentFrame).toBe(8)
  })

  it('clamps to the trajectory range', () => {
    const s = () => useStore.getState()
    s().setFrameStep(100)
    s().stepFrame(1)
    expect(s().currentFrame).toBe(9)
    s().stepFrame(-1)
    expect(s().currentFrame).toBe(0)
  })

  it('works without a selected channel and is a no-op on an empty trajectory', () => {
    const s = () => useStore.getState()
    expect(s().selectedChannel).toBeNull()
    s().stepFrame(1)
    expect(s().currentFrame).toBe(4)

    s().setTrajectory(Trajectory.empty())
    s().stepFrame(1)
    expect(s().currentFrame).toBe(0)
  })
})

describe('nudgeSelectedChannel', () => {
  it('nudges the selected joint channel up/down by nudgeStep at the current frame', () => {
    const s = () => useStore.getState()
    s().setSelectedChannel({ kind: 'joint', index: 1 })
    const before = s().trajectory.getChannelValue(3, { kind: 'joint', index: 1 })

    s().nudgeSelectedChannel(1)
    expect(s().trajectory.getChannelValue(3, { kind: 'joint', index: 1 })).toBeCloseTo(before + 0.01)

    s().nudgeSelectedChannel(-1)
    s().nudgeSelectedChannel(-1)
    expect(s().trajectory.getChannelValue(3, { kind: 'joint', index: 1 })).toBeCloseTo(before - 0.01)

    // Other frames and channels untouched.
    expect(s().trajectory.getChannelValue(2, { kind: 'joint', index: 1 })).toBeCloseTo(0.21)
    expect(s().trajectory.getChannelValue(3, { kind: 'joint', index: 0 })).toBeCloseTo(0.3)
    expect(s().trajectoryVersion).toBe(3)
  })

  it('nudges base position and base orientation (euler) channels', () => {
    const s = () => useStore.getState()
    s().setSelectedChannel({ kind: 'basePos', axis: 2 })
    s().nudgeSelectedChannel(1)
    expect(s().trajectory.getChannelValue(3, { kind: 'basePos', axis: 2 })).toBeCloseTo(1.01)

    s().setSelectedChannel({ kind: 'baseQuat', axis: 0 })
    s().nudgeSelectedChannel(1)
    expect(s().trajectory.getChannelValue(3, { kind: 'baseQuat', axis: 0 })).toBeCloseTo(0.01)
    const q = s().trajectory.getFrame(3).baseQuatW
    expect(Math.hypot(q[0], q[1], q[2], q[3])).toBeCloseTo(1)
  })

  it('respects a custom nudgeStep from the backend config', () => {
    const s = () => useStore.getState()
    s().setNudgeStep(0.1)
    s().setSelectedChannel({ kind: 'joint', index: 0 })
    s().nudgeSelectedChannel(1)
    expect(s().trajectory.getChannelValue(3, { kind: 'joint', index: 0 })).toBeCloseTo(0.4)
  })

  it('is a no-op without a selected channel or trajectory', () => {
    const s = () => useStore.getState()
    s().nudgeSelectedChannel(1)
    expect(s().trajectoryVersion).toBe(0)

    s().setSelectedChannel({ kind: 'joint', index: 0 })
    s().setTrajectory(Trajectory.empty())
    s().nudgeSelectedChannel(1)
    expect(s().trajectoryVersion).toBe(0)
  })
})
