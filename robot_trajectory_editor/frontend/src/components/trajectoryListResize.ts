export interface TrajectoryListSize {
  width: number
  height: number
}

export type TrajectoryListResizeAxis = 'x' | 'y' | 'xy'

export const TRAJECTORY_LIST_MIN_WIDTH = 120
export const TRAJECTORY_LIST_MAX_WIDTH = 720
export const TRAJECTORY_LIST_MIN_HEIGHT = 64
export const TRAJECTORY_LIST_MAX_HEIGHT = 600
// LeftPanel horizontal padding (16px) + left panel scrollbar gutter (~16px).
export const TRAJECTORY_LIST_PANEL_CHROME = 32

export function resizeTrajectoryListBox(
  start: TrajectoryListSize,
  deltaX: number,
  deltaY: number,
  axis: TrajectoryListResizeAxis,
  maxWidth = TRAJECTORY_LIST_MAX_WIDTH,
  maxHeight = TRAJECTORY_LIST_MAX_HEIGHT,
): TrajectoryListSize {
  const effectiveMaxWidth = Math.max(TRAJECTORY_LIST_MIN_WIDTH, maxWidth)
  const effectiveMaxHeight = Math.max(TRAJECTORY_LIST_MIN_HEIGHT, maxHeight)
  const width = axis === 'y'
    ? start.width
    : Math.min(effectiveMaxWidth, Math.max(TRAJECTORY_LIST_MIN_WIDTH, Math.round(start.width + deltaX)))
  const height = axis === 'x'
    ? start.height
    : Math.min(effectiveMaxHeight, Math.max(TRAJECTORY_LIST_MIN_HEIGHT, Math.round(start.height + deltaY)))
  return { width, height }
}
