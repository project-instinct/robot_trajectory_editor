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
    undoStack: [],
    currentFrame: 3,
    selectedChannel: null,
    segmentStart: null,
    segmentEnd: null,
    isPlaying: false,
    pinnedLinks: [],
    nudgeStep: 0.01,
    frameStep: 1,
    baseTransform: null,
    baseTransformSource: null,
    baseTransformOrigin: null,
    trajectorySourceName: null,
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

  it('keeps working when the segment is reselected from right to left', () => {
    const s = () => useStore.getState()
    const channel = { kind: 'joint', index: 0 } as const
    s().setSelectedChannel(channel)
    s().setSegment(2, 5)
    s().fillRange()

    s().setCurrentFrame(7)
    s().setSegment(8, 6)
    s().fillRange()

    for (let f = 6; f <= 8; f++) {
      expect(s().trajectory.getChannelValue(f, channel)).toBeCloseTo(0.7)
    }
    expect(s().trajectoryVersion).toBe(2)
  })

  it('is a no-op without a selected channel', () => {
    const s = () => useStore.getState()
    s().setSegment(2, 5)
    s().fillRange()
    expect(s().trajectoryVersion).toBe(0)
    expect(s().trajectory.getChannelValue(3, { kind: 'joint', index: 0 })).toBeCloseTo(0.3)
  })
})

describe('segment endpoint selection', () => {
  it('preserves an existing end when the new start is before it', () => {
    const s = () => useStore.getState()
    s().setSegment(null, 7)
    s().setSegmentStart(3)
    expect(s().segmentStart).toBe(3)
    expect(s().segmentEnd).toBe(7)
  })

  it('resets the end to the final frame when the new start is after it', () => {
    const s = () => useStore.getState()
    s().setSegment(null, 4)
    s().setSegmentStart(6)
    expect(s().segmentStart).toBe(6)
    expect(s().segmentEnd).toBe(9)
  })

  it('uses the timeline bounds when either endpoint is selected alone', () => {
    const s = () => useStore.getState()
    s().setSegmentStart(3)
    expect([s().segmentStart, s().segmentEnd]).toEqual([3, 9])

    s().setSegment(null, null)
    s().setSegmentEnd(7)
    expect([s().segmentStart, s().segmentEnd]).toEqual([0, 7])
  })
})

describe('cycleSelectedChannel', () => {
  it('wraps through joints without crossing into base position', () => {
    const s = () => useStore.getState()
    s().setSelectedChannel({ kind: 'joint', index: 1 })
    s().cycleSelectedChannel(1)
    expect(s().selectedChannel).toEqual({ kind: 'joint', index: 0 })

    s().cycleSelectedChannel(-1)
    expect(s().selectedChannel).toEqual({ kind: 'joint', index: 1 })
  })

  it('wraps independently inside both three-axis base blocks', () => {
    const s = () => useStore.getState()
    s().setSelectedChannel({ kind: 'basePos', axis: 2 })
    s().cycleSelectedChannel(1)
    expect(s().selectedChannel).toEqual({ kind: 'basePos', axis: 0 })

    s().setSelectedChannel({ kind: 'baseQuat', axis: 0 })
    s().cycleSelectedChannel(-1)
    expect(s().selectedChannel).toEqual({ kind: 'baseQuat', axis: 2 })
  })

  it('does nothing when no target is selected', () => {
    useStore.getState().cycleSelectedChannel(1)
    expect(useStore.getState().selectedChannel).toBeNull()
  })
})

describe('undo', () => {
  it('reverts the latest trajectory edit, including keyframes', () => {
    const s = () => useStore.getState()
    s().setSelectedChannel({ kind: 'joint', index: 0 })
    const before = s().trajectory.getChannelValue(3, { kind: 'joint', index: 0 })

    s().beginTrajectoryEdit()
    s().trajectory.setChannelValue(3, { kind: 'joint', index: 0 }, 9)
    s().trajectory.insertKeyframe(3)
    s().touchTrajectory()
    s().undo()

    expect(s().trajectory.getChannelValue(3, { kind: 'joint', index: 0 })).toBeCloseTo(before)
    expect(s().trajectory.hasKeyframe(3)).toBe(false)
  })

  it('can undo multiple edits in reverse order', () => {
    const s = () => useStore.getState()
    s().setSelectedChannel({ kind: 'joint', index: 0 })
    const original = s().trajectory.getChannelValue(3, { kind: 'joint', index: 0 })

    s().nudgeSelectedChannel(1)
    s().nudgeSelectedChannel(1)
    s().undo()
    expect(s().trajectory.getChannelValue(3, { kind: 'joint', index: 0 })).toBeCloseTo(original + 0.01)
    s().undo()
    expect(s().trajectory.getChannelValue(3, { kind: 'joint', index: 0 })).toBeCloseTo(original)
  })
})

describe('interpolateRange', () => {
  it('interpolates the selected segment and can be undone', () => {
    const s = () => useStore.getState()
    s().setSelectedChannel({ kind: 'joint', index: 0 })
    s().setSegment(1, 5)
    s().trajectory.setChannelValue(1, { kind: 'joint', index: 0 }, 1)
    s().trajectory.setChannelValue(3, { kind: 'joint', index: 0 }, 99)
    s().trajectory.setChannelValue(5, { kind: 'joint', index: 0 }, 3)

    s().interpolateRange()
    expect(s().trajectory.getChannelValue(3, { kind: 'joint', index: 0 })).toBeCloseTo(2)
    expect(s().trajectoryVersion).toBe(1)

    s().undo()
    expect(s().trajectory.getChannelValue(3, { kind: 'joint', index: 0 })).toBeCloseTo(99)
  })

  it('does nothing without both a selected channel and segment', () => {
    const s = () => useStore.getState()
    s().interpolateRange()
    s().setSelectedChannel({ kind: 'joint', index: 0 })
    s().interpolateRange()
    expect(s().trajectoryVersion).toBe(0)
    expect(s().undoStack).toHaveLength(0)
  })
})

describe('pinned links', () => {
  it('toggles multiple links independently and clears all links', () => {
    const s = () => useStore.getState()
    s().togglePinnedLink('left_foot')
    s().togglePinnedLink('right_foot')
    expect(s().pinnedLinks).toEqual(['left_foot', 'right_foot'])
    s().togglePinnedLink('left_foot')
    expect(s().pinnedLinks).toEqual(['right_foot'])
    s().clearPinnedLinks()
    expect(s().pinnedLinks).toEqual([])
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

  it('nudges every frame inside the selected timeline segment', () => {
    const s = () => useStore.getState()
    s().setSelectedChannel({ kind: 'joint', index: 1 })
    s().setSegment(1, 3)
    const before = [1, 2, 3].map(f => s().trajectory.getChannelValue(f, { kind: 'joint', index: 1 }))

    s().nudgeSelectedChannel(1)
    for (let f = 1; f <= 3; f++) {
      expect(s().trajectory.getChannelValue(f, { kind: 'joint', index: 1 })).toBeCloseTo(before[f - 1] + 0.01)
    }
    // Frames outside the segment are untouched.
    expect(s().trajectory.getChannelValue(0, { kind: 'joint', index: 1 })).toBeCloseTo(0.01)
    expect(s().trajectory.getChannelValue(4, { kind: 'joint', index: 1 })).toBeCloseTo(0.41)
  })

  it('nudges the whole segment when it was dragged right-to-left', () => {
    const s = () => useStore.getState()
    s().setSelectedChannel({ kind: 'joint', index: 1 })
    s().setSegment(3, 1)
    const before = [1, 2, 3].map(f => s().trajectory.getChannelValue(f, { kind: 'joint', index: 1 }))

    s().nudgeSelectedChannel(-1)
    for (let f = 1; f <= 3; f++) {
      expect(s().trajectory.getChannelValue(f, { kind: 'joint', index: 1 })).toBeCloseTo(before[f - 1] - 0.01)
    }
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

describe('base transform', () => {
  it('transforms the whole base sequence live and a single undo reverts the session', () => {
    const s = () => useStore.getState()
    s().setBaseTransform({ tx: 1, ty: 0, tz: 0, roll: 0, pitch: 0, yaw: 0 })

    // every frame moved by the slider offset; joints untouched
    for (let f = 0; f < 10; f++) {
      expect(s().trajectory.getChannelValue(f, { kind: 'basePos', axis: 0 })).toBeCloseTo(1)
    }
    expect(s().trajectory.getChannelValue(3, { kind: 'joint', index: 0 })).toBeCloseTo(0.3)
    expect(s().undoStack).toHaveLength(1)
    expect(s().trajectoryVersion).toBe(1)

    s().undo()
    expect(s().trajectory.getChannelValue(0, { kind: 'basePos', axis: 0 })).toBeCloseTo(0)
    expect(s().baseTransform).toBeNull()
    expect(s().baseTransformSource).toBeNull()
  })

  it('recomputes from the session source so slider values are absolute offsets', () => {
    const s = () => useStore.getState()
    s().setBaseTransform({ tx: 1, ty: 0, tz: 0, roll: 0, pitch: 0, yaw: 0 })
    s().setBaseTransform({ tx: 0, ty: 0, tz: 0, roll: 0, pitch: 0, yaw: 0 })
    // dragging the slider back to 0 restores the original state exactly
    expect(s().trajectory.getChannelValue(0, { kind: 'basePos', axis: 0 })).toBeCloseTo(0)
    expect(s().undoStack).toHaveLength(1)
  })

  it('applying ends the session and a later adjustment becomes a fresh undo step', () => {
    const s = () => useStore.getState()
    s().setBaseTransform({ tx: 1, ty: 0, tz: 0, roll: 0, pitch: 0, yaw: 0 })
    s().applyBaseTransform()
    expect(s().baseTransform).toBeNull()
    expect(s().undoStack).toHaveLength(1)

    s().setBaseTransform({ tx: 0.5, ty: 0, tz: 0, roll: 0, pitch: 0, yaw: 0 })
    expect(s().undoStack).toHaveLength(2)
    // the slider offset is relative to this new session's start (already at +1)
    expect(s().trajectory.getChannelValue(0, { kind: 'basePos', axis: 0 })).toBeCloseTo(1.5)

    s().undo()
    expect(s().trajectory.getChannelValue(0, { kind: 'basePos', axis: 0 })).toBeCloseTo(1)
  })

  it('rotates the trajectory as a rigid body about the session start', () => {
    const s = () => useStore.getState()
    // frame 0 base is (0,0,1); yaw 90 deg about it keeps it fixed
    s().setBaseTransform({ tx: 0, ty: 0, tz: 0, roll: 0, pitch: 0, yaw: Math.PI / 2 })
    const frame0 = s().trajectory.getFrame(0)
    expect(frame0.basePoseW[0]).toBeCloseTo(0)
    expect(frame0.basePoseW[1]).toBeCloseTo(0)
    expect(frame0.basePoseW[2]).toBeCloseTo(1)
    const q = frame0.baseQuatW
    expect(Math.hypot(q[0], q[1], q[2], q[3])).toBeCloseTo(1)
  })

  it('is a no-op on an empty trajectory and apply without a session does nothing', () => {
    const s = () => useStore.getState()
    s().setTrajectory(Trajectory.empty())
    s().setBaseTransform({ tx: 1, ty: 0, tz: 0, roll: 0, pitch: 0, yaw: 0 })
    expect(s().undoStack).toHaveLength(0)
    expect(s().trajectoryVersion).toBe(0)

    s().applyBaseTransform()
    expect(s().baseTransform).toBeNull()
  })
})

describe('replaceTrajectory', () => {
  it('replaces the trajectory while keeping undo history', () => {
    const s = () => useStore.getState()
    s().beginTrajectoryEdit()
    s().replaceTrajectory(s().trajectory.slice(2, 4))

    expect(s().trajectory.frameCount).toBe(3)
    expect(s().currentFrame).toBe(0)
    expect(s().undoStack).toHaveLength(1)
    expect(s().trajectoryVersion).toBe(1)

    s().undo()
    expect(s().trajectory.frameCount).toBe(10)
    expect(s().currentFrame).toBe(0)
  })

  it('stores the source file name used for segment exports', () => {
    const s = () => useStore.getState()
    s().setTrajectorySourceName('walk')
    expect(s().trajectorySourceName).toBe('walk')
    s().setTrajectorySourceName(null)
    expect(s().trajectorySourceName).toBeNull()
  })
})
