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
  /** Fill segment (or whole trajectory) with the current frame's full robot state. */
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

  setTrajectory: (t) => set({ trajectory: t, currentFrame: 0 }),
  touchTrajectory: () => set(s => ({ trajectoryVersion: s.trajectoryVersion + 1 })),
  setCurrentFrame: (f) => set({ currentFrame: f }),
  setSelectedChannel: (c) => set({ selectedChannel: c }),
  setSegment: (start, end) => set({ segmentStart: start, segmentEnd: end }),
  setIsPlaying: (p) => set({ isPlaying: p }),
  setPinnedLink: (name) => set({ pinnedLink: name }),
  setRobotModelLoaded: (l) => set({ robotModelLoaded: l }),

  fillRange: () => {
    const { trajectory, currentFrame, segmentStart, segmentEnd } = get()
    if (trajectory.frameCount === 0) return
    const start = segmentStart ?? 0
    const end = segmentEnd ?? trajectory.frameCount - 1
    const state = trajectory.getFrame(currentFrame)
    trajectory.fillRange(start, end, state)
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
