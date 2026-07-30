import { create } from 'zustand'
import { Trajectory, type ChannelKind } from './Trajectory'

export interface StoreState {
  trajectory: Trajectory
  currentFrame: number
  selectedChannel: ChannelKind | null
  segmentStart: number | null
  segmentEnd: number | null
  isPlaying: boolean
  pinnedLink: string | null
  robotModelLoaded: boolean

  setTrajectory: (t: Trajectory) => void
  setCurrentFrame: (f: number) => void
  setSelectedChannel: (c: ChannelKind | null) => void
  setSegment: (start: number | null, end: number | null) => void
  setIsPlaying: (p: boolean) => void
  setPinnedLink: (name: string | null) => void
  setRobotModelLoaded: (l: boolean) => void
  fillRange: () => void
  smoothRange: () => void
}

export const useStore = create<StoreState>((set, get) => ({
  trajectory: Trajectory.empty(),
  currentFrame: 0,
  selectedChannel: null,
  segmentStart: null,
  segmentEnd: null,
  isPlaying: false,
  pinnedLink: null,
  robotModelLoaded: false,

  setTrajectory: (t) => set({ trajectory: t, currentFrame: 0 }),
  setCurrentFrame: (f) => set({ currentFrame: f }),
  setSelectedChannel: (c) => set({ selectedChannel: c }),
  setSegment: (start, end) => set({ segmentStart: start, segmentEnd: end }),
  setIsPlaying: (p) => set({ isPlaying: p }),
  setPinnedLink: (name) => set({ pinnedLink: name }),
  setRobotModelLoaded: (l) => set({ robotModelLoaded: l }),

  fillRange: () => {
    const { trajectory, currentFrame, selectedChannel, segmentStart, segmentEnd } = get()
    if (!selectedChannel) return
    const start = segmentStart ?? 0
    const end = segmentEnd ?? trajectory.frameCount - 1
    const value = trajectory.getChannelValue(currentFrame, selectedChannel)
    trajectory.fillChannel(start, end, selectedChannel, value)
    set({ trajectory: trajectory })
  },

  smoothRange: () => {
    const { trajectory, selectedChannel, segmentStart, segmentEnd } = get()
    if (!selectedChannel) return
    const start = segmentStart ?? 0
    const end = segmentEnd ?? trajectory.frameCount - 1
    trajectory.smoothRange(start, end, selectedChannel)
    set({ trajectory: trajectory })
  },
}))
