export interface TimelineView {
  startFrame: number
  endFrame: number
}

export interface ValueRange {
  minValue: number
  maxValue: number
}

const MIN_VISIBLE_FRAME_SPAN = 4
const MIN_VISIBLE_VALUE_FRACTION = 0.02

export function fullTimelineView(frameCount: number): TimelineView {
  return { startFrame: 0, endFrame: Math.max(frameCount - 1, 0) }
}

export function frameAtClientX(
  clientX: number,
  canvasLeft: number,
  canvasWidth: number,
  view: TimelineView,
  frameCount: number,
): number {
  if (frameCount < 2 || canvasWidth <= 0) return 0
  const t = Math.max(0, Math.min((clientX - canvasLeft) / canvasWidth, 1))
  const frame = Math.round(view.startFrame + t * (view.endFrame - view.startFrame))
  return Math.max(0, Math.min(frame, frameCount - 1))
}

export function frameToCanvasX(frame: number, canvasWidth: number, view: TimelineView): number {
  const span = view.endFrame - view.startFrame
  if (span <= 0) return 0
  return ((frame - view.startFrame) / span) * canvasWidth
}

/** Zooms around an anchor and optionally moves that anchor for a translated pinch. */
export function zoomTimelineView(
  view: TimelineView,
  frameCount: number,
  spanFactor: number,
  anchorRatio: number,
  targetAnchorRatio = anchorRatio,
): TimelineView {
  const lastFrame = frameCount - 1
  if (lastFrame <= 0) return fullTimelineView(frameCount)

  const currentSpan = Math.max(view.endFrame - view.startFrame, Number.EPSILON)
  const minSpan = Math.min(MIN_VISIBLE_FRAME_SPAN, lastFrame)
  const nextSpan = Math.max(minSpan, Math.min(currentSpan * spanFactor, lastFrame))
  const sourceRatio = Math.max(0, Math.min(anchorRatio, 1))
  const destinationRatio = Math.max(0, Math.min(targetAnchorRatio, 1))
  const anchorFrame = view.startFrame + sourceRatio * currentSpan
  const unclampedStart = anchorFrame - destinationRatio * nextSpan
  const startFrame = Math.max(0, Math.min(unclampedStart, lastFrame - nextSpan))

  return { startFrame, endFrame: startFrame + nextSpan }
}

export function resizeTimelineView(
  view: TimelineView,
  frameCount: number,
  edge: 'start' | 'end',
  frame: number,
): TimelineView {
  const lastFrame = frameCount - 1
  if (lastFrame <= 0) return fullTimelineView(frameCount)
  const minSpan = Math.min(MIN_VISIBLE_FRAME_SPAN, lastFrame)

  if (edge === 'start') {
    const startFrame = Math.max(0, Math.min(Math.round(frame), view.endFrame - minSpan))
    return { startFrame, endFrame: view.endFrame }
  }

  const endFrame = Math.min(lastFrame, Math.max(Math.round(frame), view.startFrame + minSpan))
  return { startFrame: view.startFrame, endFrame }
}

export function resizeValueRange(
  range: ValueRange,
  globalRange: ValueRange,
  edge: 'min' | 'max',
  value: number,
): ValueRange {
  const globalSpan = globalRange.maxValue - globalRange.minValue
  if (globalSpan <= 0) return globalRange
  const minSpan = globalSpan * MIN_VISIBLE_VALUE_FRACTION

  if (edge === 'min') {
    const minValue = Math.max(
      globalRange.minValue,
      Math.min(value, range.maxValue - minSpan),
    )
    return { minValue, maxValue: range.maxValue }
  }

  const maxValue = Math.min(
    globalRange.maxValue,
    Math.max(value, range.minValue + minSpan),
  )
  return { minValue: range.minValue, maxValue }
}
