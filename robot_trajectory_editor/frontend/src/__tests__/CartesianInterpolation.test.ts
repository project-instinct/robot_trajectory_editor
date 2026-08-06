import { describe, expect, it } from 'vitest'
import * as THREE from 'three'
import { interpolationAlpha, localPointToWorld } from '../three/CartesianInterpolation'

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

describe('Cartesian local point', () => {
  it('transforms a link-local point by the link pose', () => {
    const world = localPointToWorld({
      pos: new THREE.Vector3(2, 3, 4),
      quat: new THREE.Quaternion().setFromAxisAngle(
        new THREE.Vector3(0, 0, 1),
        Math.PI / 2,
      ),
    }, [1, 0, 0])

    expect(world.x).toBeCloseTo(2)
    expect(world.y).toBeCloseTo(4)
    expect(world.z).toBeCloseTo(4)
  })
})
