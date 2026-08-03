import { describe, expect, it } from 'vitest'
import { interpolationAlpha } from '../three/CartesianInterpolation'

describe('Cartesian interpolation timing', () => {
  it('uses direct progress for linear interpolation', () => {
    expect(interpolationAlpha(0, 'linear')).toBe(0)
    expect(interpolationAlpha(0.25, 'linear')).toBe(0.25)
    expect(interpolationAlpha(1, 'linear')).toBe(1)
  })

  it('uses a cubic smoothstep with fixed endpoints', () => {
    expect(interpolationAlpha(0, 'cubic')).toBe(0)
    expect(interpolationAlpha(0.25, 'cubic')).toBeCloseTo(0.15625)
    expect(interpolationAlpha(0.5, 'cubic')).toBe(0.5)
    expect(interpolationAlpha(0.75, 'cubic')).toBeCloseTo(0.84375)
    expect(interpolationAlpha(1, 'cubic')).toBe(1)
  })
})
