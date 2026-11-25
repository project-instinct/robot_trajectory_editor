import { describe, expect, it } from 'vitest'
import {
  resizeTrajectoryListBox,
  TRAJECTORY_LIST_MAX_HEIGHT,
  TRAJECTORY_LIST_MAX_WIDTH,
  TRAJECTORY_LIST_MIN_HEIGHT,
  TRAJECTORY_LIST_MIN_WIDTH,
} from '../components/trajectoryListResize'

describe('resizeTrajectoryListBox', () => {
  const start = { width: 240, height: 140 }

  it('resizes both width and height larger and smaller on corner drag', () => {
    expect(resizeTrajectoryListBox(start, 80, 60, 'xy')).toEqual({ width: 320, height: 200 })
    expect(resizeTrajectoryListBox(start, -60, -40, 'xy')).toEqual({ width: 180, height: 100 })
  })

  it('resizes only width or only height on single-axis edge drags', () => {
    expect(resizeTrajectoryListBox(start, 50, 90, 'x')).toEqual({ width: 290, height: 140 })
    expect(resizeTrajectoryListBox(start, 50, 90, 'y')).toEqual({ width: 240, height: 230 })
  })

  it('clamps to minimum and maximum bounds', () => {
    expect(resizeTrajectoryListBox(start, -500, -500, 'xy')).toEqual({
      width: TRAJECTORY_LIST_MIN_WIDTH,
      height: TRAJECTORY_LIST_MIN_HEIGHT,
    })
    expect(resizeTrajectoryListBox(start, 2000, 2000, 'xy')).toEqual({
      width: TRAJECTORY_LIST_MAX_WIDTH,
      height: TRAJECTORY_LIST_MAX_HEIGHT,
    })
  })

  it('respects custom viewport-constrained max bounds', () => {
    expect(resizeTrajectoryListBox(start, 400, 400, 'xy', 300, 250)).toEqual({
      width: 300,
      height: 250,
    })
  })
})
