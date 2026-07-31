import { create } from 'zustand'
import { Trajectory, type ChannelKind } from './Trajectory'

export interface StoreState {
  trajectory: Trajectory
  /** Bumped on every in-place trajectory mutation so subscribers re-render. */
  trajectoryVersion: number
  currentFrame: number
  selectedChannel: ChannelKind | null
  segmentStart: number | null
  segmentEnd: number | null
  isPlaying: boolean
  pinnedLink: string | null
  robotModelLoaded: boolean
  /** Value step applied by the Up/Down arrow keys (from backend /api/config). */
  nudgeStep: number
  /** Frame step applied by the Left/Right arrow keys (from backend /api/config). */
  frameStep: number

  /** Load a new trajectory (resets playback position). */
  setTrajectory: (t: Trajectory) => void
  /** Notify that the current trajectory was mutated in place. */
  touchTrajectory: () => void
  setCurrentFrame: (f: number) => void
  setSelectedChannel: (c: ChannelKind | null) => void
  setSegment: (start: number | null, end: number | null) => void
  setIsPlaying: (p: boolean) => void
  setPinnedLink: (name: string | null) => void
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
}

export const useStore = create<StoreState>((set, get) => ({
  trajectory: Trajectory.empty(),
  trajectoryVersion: 0,
  currentFrame: 0,
  selectedChannel: null,
  segmentStart: null,
  segmentEnd: null,
  isPlaying: false,
  pinnedLink: null,
  robotModelLoaded: false,
  nudgeStep: 0.01,
  frameStep: 1,

  setTrajectory: (t) => set({ trajectory: t, currentFrame: 0 }),
  touchTrajectory: () => set(s => ({ trajectoryVersion: s.trajectoryVersion + 1 })),
  setCurrentFrame: (f) => set({ currentFrame: f }),
  setSelectedChannel: (c) => set({ selectedChannel: c }),
  setSegment: (start, end) => set({ segmentStart: start, segmentEnd: end }),
  setIsPlaying: (p) => set({ isPlaying: p }),
  setPinnedLink: (name) => set({ pinnedLink: name }),
  setRobotModelLoaded: (l) => set({ robotModelLoaded: l }),
  setNudgeStep: (step) => set({ nudgeStep: step }),

  nudgeSelectedChannel: (direction) => {
    const { trajectory, selectedChannel, currentFrame, nudgeStep } = get()
    if (!selectedChannel || trajectory.frameCount === 0) return
    const value = trajectory.getChannelValue(currentFrame, selectedChannel) + direction * nudgeStep
    trajectory.setChannelValue(currentFrame, selectedChannel, value)
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
    const start = segmentStart ?? 0
    const end = segmentEnd ?? trajectory.frameCount - 1
    const value = trajectory.getChannelValue(currentFrame, selectedChannel)
    trajectory.fillChannelRange(start, end, selectedChannel, value)
    get().touchTrajectory()
  },

  smoothRange: () => {
    const { trajectory, selectedChannel, segmentStart, segmentEnd } = get()
    if (!selectedChannel || trajectory.frameCount === 0) return
    const start = segmentStart ?? 0
    const end = segmentEnd ?? trajectory.frameCount - 1
    trajectory.smoothRange(start, end, selectedChannel)
    get().touchTrajectory()
  },
}))
