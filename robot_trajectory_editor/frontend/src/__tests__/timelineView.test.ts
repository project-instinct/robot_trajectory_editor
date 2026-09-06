import { describe, expect, it } from 'vitest'
import {
  frameAtClientX,
  frameToCanvasX,
  fullTimelineView,
  resizeTimelineView,
  resizeValueRange,
  zoomTimelineView,
} from '../components/timelineView'

describe('timeline view', () => {
  it('maps client positions through the visible frame range', () => {
    const view = { startFrame: 20, endFrame: 60 }

    expect(frameAtClientX(100, 100, 400, view, 100)).toBe(20)
    expect(frameAtClientX(300, 100, 400, view, 100)).toBe(40)
    expect(frameAtClientX(500, 100, 400, view, 100)).toBe(60)
    expect(frameToCanvasX(40, 400, view)).toBe(200)
  })

  it('zooms around the pointer and clamps to the trajectory', () => {
    const full = fullTimelineView(101)
    const zoomed = zoomTimelineView(full, 101, 0.5, 0.25)

    expect(zoomed).toEqual({ startFrame: 12.5, endFrame: 62.5 })
    expect(zoomTimelineView(zoomed, 101, 10, 0.5)).toEqual(full)
  })

  it('moves the visible range with a translated pinch midpoint', () => {
    const view = { startFrame: 20, endFrame: 60 }
    const moved = zoomTimelineView(view, 101, 1, 0.5, 0.25)

    expect(moved).toEqual({ startFrame: 30, endFrame: 70 })
  })

  it('keeps at least five frames visible at maximum zoom', () => {
    const zoomed = zoomTimelineView(fullTimelineView(101), 101, 0.001, 0.5)

    expect(zoomed.endFrame - zoomed.startFrame).toBe(4)
  })

  it('resizes either frame edge without crossing the minimum span', () => {
    expect(resizeTimelineView({ startFrame: 10, endFrame: 50 }, 101, 'start', 20))
      .toEqual({ startFrame: 20, endFrame: 50 })
    expect(resizeTimelineView({ startFrame: 20, endFrame: 50 }, 101, 'end', 22))
      .toEqual({ startFrame: 20, endFrame: 24 })
  })

  it('resizes either value edge within the global value range', () => {
    const globalRange = { minValue: -10, maxValue: 10 }
    expect(resizeValueRange(globalRange, globalRange, 'min', -4))
      .toEqual({ minValue: -4, maxValue: 10 })
    expect(resizeValueRange({ minValue: -4, maxValue: 10 }, globalRange, 'max', -8))
      .toEqual({ minValue: -4, maxValue: -3.6 })
  })
})
