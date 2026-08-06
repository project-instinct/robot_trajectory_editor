import { create } from 'zustand'
import { Trajectory, type ChannelKind } from './Trajectory'

/** Slider values of the base-transform panel. Angles are in radians, matching
 *  the euler convention used by Trajectory's baseQuat channels. */
export interface BaseTransform {
  tx: number
  ty: number
  tz: number
  roll: number
  pitch: number
  yaw: number
}

export interface StoreState {
  trajectory: Trajectory
  /** Bumped on every in-place trajectory mutation so subscribers re-render. */
  trajectoryVersion: number
  /** Snapshots captured immediately before user edits. */
  undoStack: Trajectory[]
  currentFrame: number
  selectedChannel: ChannelKind | null
  segmentStart: number | null
  segmentEnd: number | null
  isPlaying: boolean
  pinnedLinks: string[]
  robotModelLoaded: boolean
  /** Value step applied by the Up/Down arrow keys (from backend /api/config). */
  nudgeStep: number
  /** Frame step applied by the Left/Right arrow keys (from backend /api/config). */
  frameStep: number
  /** Active base-transform session slider values, or null when idle. */
  baseTransform: BaseTransform | null
  /** Trajectory snapshot the active base transform is recomputed from, and the
   *  point its rotation pivots around (the first base position at session start). */
  baseTransformSource: Trajectory | null
  baseTransformOrigin: [number, number, number] | null

  /** Load a new trajectory (resets playback position). */
  setTrajectory: (t: Trajectory) => void
  /** Capture the current trajectory before an in-place user edit. */
  beginTrajectoryEdit: () => void
  /** Notify that the current trajectory was mutated in place. */
  touchTrajectory: () => void
  undo: () => void
  setCurrentFrame: (f: number) => void
  setSelectedChannel: (c: ChannelKind | null) => void
  /** Select the previous/next channel within the current channel block, wrapping at its edges. */
  cycleSelectedChannel: (direction: 1 | -1) => void
  setSegment: (start: number | null, end: number | null) => void
  /** Set the segment start, preserving a valid end or falling back to the final frame. */
  setSegmentStart: (frame: number) => void
  /** Set the segment end, falling back to the first frame when no start exists. */
  setSegmentEnd: (frame: number) => void
  setIsPlaying: (p: boolean) => void
  togglePinnedLink: (name: string) => void
  clearPinnedLinks: () => void
  setRobotModelLoaded: (l: boolean) => void
  setNudgeStep: (step: number) => void
  /** Move the selected channel's value at the current frame by ±nudgeStep. */
  nudgeSelectedChannel: (direction: 1 | -1) => void
  setFrameStep: (step: number) => void
  /** Move the current frame by ±frameStep, clamped to the trajectory range. */
  stepFrame: (direction: 1 | -1) => void
  /** Fill the segment (or whole trajectory) of the selected channel with the
   *  current frame's value of that channel. */
  fillRange: () => void
  /** Low-pass smooth of the selected channel over the segment (or whole trajectory):
   *  removes high-frequency jitter, preserves the overall motion shape. */
  smoothRange: () => void
  /** Linearly interpolate the selected channel between selected segment endpoints. */
  interpolateRange: () => void
  /** Start a base-transform session: capture the undo snapshot, the pivot point,
   *  and the source the live transform is recomputed from. */
  beginBaseTransform: () => void
  /** Apply a base transform to the whole trajectory live, recomputed from the
   *  session source so the sliders are absolute offsets from the session start. */
  setBaseTransform: (t: BaseTransform) => void
  /** Confirm the active base-transform session (the transform is already applied
   *  live; this just ends the session so a later edit forms a new undo step). */
  applyBaseTransform: () => void
}

export const useStore = create<StoreState>((set, get) => ({
  trajectory: Trajectory.empty(),
  trajectoryVersion: 0,
  undoStack: [],
  currentFrame: 0,
  selectedChannel: null,
  segmentStart: null,
  segmentEnd: null,
  isPlaying: false,
  pinnedLinks: [],
  robotModelLoaded: false,
  nudgeStep: 0.01,
  frameStep: 1,
  baseTransform: null,
  baseTransformSource: null,
  baseTransformOrigin: null,

  setTrajectory: (t) => set({
    trajectory: t, currentFrame: 0, undoStack: [],
    baseTransform: null, baseTransformSource: null, baseTransformOrigin: null,
  }),
  beginTrajectoryEdit: () => set(s => ({
    undoStack: [...s.undoStack, s.trajectory.clone()].slice(-100),
  })),
  touchTrajectory: () => set(s => ({ trajectoryVersion: s.trajectoryVersion + 1 })),
  undo: () => {
    const { undoStack, currentFrame } = get()
    const previous = undoStack[undoStack.length - 1]
    if (!previous) return
    const nextFrame = previous.frameCount === 0
      ? 0
      : Math.min(currentFrame, previous.frameCount - 1)
    set(s => ({
      trajectory: previous,
      undoStack: s.undoStack.slice(0, -1),
      currentFrame: nextFrame,
      trajectoryVersion: s.trajectoryVersion + 1,
      baseTransform: null, baseTransformSource: null, baseTransformOrigin: null,
    }))
  },
  setCurrentFrame: (f) => set({ currentFrame: f }),
  setSelectedChannel: (c) => set({ selectedChannel: c }),
  cycleSelectedChannel: (direction) => {
    const { trajectory, selectedChannel } = get()
    if (!selectedChannel) return
    const count = selectedChannel.kind === 'joint' ? trajectory.jointCount : 3
    if (count === 0) return
    const current = selectedChannel.kind === 'joint'
      ? selectedChannel.index
      : selectedChannel.axis
    const next = (current + direction + count) % count
    set({
      selectedChannel: selectedChannel.kind === 'joint'
        ? { kind: 'joint', index: next }
        : { kind: selectedChannel.kind, axis: next },
    })
  },
  setSegment: (start, end) => set({ segmentStart: start, segmentEnd: end }),
  setSegmentStart: (frame) => {
    const { trajectory, segmentEnd } = get()
    if (trajectory.frameCount === 0) return
    const end = segmentEnd === null || frame > segmentEnd
      ? trajectory.frameCount - 1
      : segmentEnd
    set({ segmentStart: frame, segmentEnd: end })
  },
  setSegmentEnd: (frame) => {
    const { trajectory, segmentStart } = get()
    if (trajectory.frameCount === 0) return
    set({ segmentStart: segmentStart ?? 0, segmentEnd: frame })
  },
  setIsPlaying: (p) => set({ isPlaying: p }),
  togglePinnedLink: (name) => set(s => ({
    pinnedLinks: s.pinnedLinks.includes(name)
      ? s.pinnedLinks.filter(link => link !== name)
      : [...s.pinnedLinks, name],
  })),
  clearPinnedLinks: () => set({ pinnedLinks: [] }),
  setRobotModelLoaded: (l) => set({ robotModelLoaded: l }),
  setNudgeStep: (step) => set({ nudgeStep: step }),

  nudgeSelectedChannel: (direction) => {
    const { trajectory, selectedChannel, currentFrame, nudgeStep, segmentStart, segmentEnd } = get()
    if (!selectedChannel || trajectory.frameCount === 0) return
    get().beginTrajectoryEdit()
    const delta = direction * nudgeStep
    if (segmentStart !== null && segmentEnd !== null) {
      // Nudge every frame inside the selected timeline segment.
      const start = Math.min(segmentStart, segmentEnd)
      const end = Math.max(segmentStart, segmentEnd)
      for (let f = start; f <= end; f++) {
        trajectory.setChannelValue(f, selectedChannel, trajectory.getChannelValue(f, selectedChannel) + delta)
      }
    } else {
      // No segment: nudge only the current frame.
      trajectory.setChannelValue(currentFrame, selectedChannel, trajectory.getChannelValue(currentFrame, selectedChannel) + delta)
    }
    get().touchTrajectory()
  },

  setFrameStep: (step) => set({ frameStep: step }),

  stepFrame: (direction) => {
    const { trajectory, currentFrame, frameStep } = get()
    if (trajectory.frameCount === 0) return
    const next = Math.min(Math.max(currentFrame + direction * frameStep, 0), trajectory.frameCount - 1)
    if (next !== currentFrame) set({ currentFrame: next })
  },

  fillRange: () => {
    const { trajectory, selectedChannel, currentFrame, segmentStart, segmentEnd } = get()
    if (!selectedChannel || trajectory.frameCount === 0) return
    get().beginTrajectoryEdit()
    const selectedStart = segmentStart ?? 0
    const selectedEnd = segmentEnd ?? trajectory.frameCount - 1
    const start = Math.min(selectedStart, selectedEnd)
    const end = Math.max(selectedStart, selectedEnd)
    const value = trajectory.getChannelValue(currentFrame, selectedChannel)
    trajectory.fillChannelRange(start, end, selectedChannel, value)
    get().touchTrajectory()
  },

  smoothRange: () => {
    const { trajectory, selectedChannel, segmentStart, segmentEnd } = get()
    if (!selectedChannel || trajectory.frameCount === 0) return
    get().beginTrajectoryEdit()
    const selectedStart = segmentStart ?? 0
    const selectedEnd = segmentEnd ?? trajectory.frameCount - 1
    const start = Math.min(selectedStart, selectedEnd)
    const end = Math.max(selectedStart, selectedEnd)
    trajectory.smoothRange(start, end, selectedChannel)
    get().touchTrajectory()
  },

  interpolateRange: () => {
    const { trajectory, selectedChannel, segmentStart, segmentEnd } = get()
    if (!selectedChannel || segmentStart === null || segmentEnd === null || trajectory.frameCount === 0) return
    const start = Math.min(segmentStart, segmentEnd)
    const end = Math.max(segmentStart, segmentEnd)
    if (start === end) return
    get().beginTrajectoryEdit()
    trajectory.interpolateChannelRange(start, end, selectedChannel)
    get().touchTrajectory()
  },

  beginBaseTransform: () => {
    const { trajectory } = get()
    if (trajectory.frameCount === 0 || get().baseTransform) return
    // One snapshot serves both as the undo target and the source the live
    // transform is recomputed from, so dragging sliders around stays a single
    // Ctrl+Z undoable operation.
    const snapshot = trajectory.clone()
    set({
      undoStack: [...get().undoStack, snapshot].slice(-100),
      baseTransformSource: snapshot,
      baseTransformOrigin: [snapshot.basePoseW[0], snapshot.basePoseW[1], snapshot.basePoseW[2]],
      baseTransform: { tx: 0, ty: 0, tz: 0, roll: 0, pitch: 0, yaw: 0 },
    })
  },

  setBaseTransform: (t) => {
    if (!get().baseTransform) get().beginBaseTransform()
    const { baseTransformSource, baseTransformOrigin } = get()
    if (!baseTransformSource || !baseTransformOrigin) return
    const trajectory = get().trajectory
    trajectory.copyBaseFrom(baseTransformSource)
    trajectory.transformBase(t.tx, t.ty, t.tz, t.roll, t.pitch, t.yaw, baseTransformOrigin)
    set({ baseTransform: t })
    get().touchTrajectory()
  },

  applyBaseTransform: () => {
    if (!get().baseTransform) return
    set({ baseTransform: null, baseTransformSource: null, baseTransformOrigin: null })
  },
}))
