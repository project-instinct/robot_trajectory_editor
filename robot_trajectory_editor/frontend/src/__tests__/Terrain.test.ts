import { describe, expect, it } from 'vitest'
import * as THREE from 'three'
import { scaleTerrain } from '../three/Terrain'

function quadAt(x: number, y: number, z: number): THREE.Group {
  const geom = new THREE.BufferGeometry()
  geom.setAttribute('position', new THREE.Float32BufferAttribute([
    x, y, z, x + 1, y, z, x + 1, y + 1, z,
  ], 3))
  const group = new THREE.Group()
  group.add(new THREE.Mesh(geom, new THREE.MeshStandardMaterial()))
  return group
}

function vertices(group: THREE.Group): number[] {
  const mesh = group.children[0] as THREE.Mesh
  return Array.from(mesh.geometry.getAttribute('position').array as Float32Array)
}

describe('scaleTerrain', () => {
  it('scales vertices about the world origin', () => {
    const group = quadAt(1, 2, 3)
    scaleTerrain(group, 2)
    expect(vertices(group)).toEqual([2, 4, 6, 4, 4, 6, 4, 6, 6])
  })

  it('scales about the origin after a move, not about the mesh pivot', () => {
    const group = quadAt(1, 0, 0)
    group.position.set(2, 0, 0)
    scaleTerrain(group, 2)
    expect(group.position.toArray()).toEqual([0, 0, 0])
    expect(vertices(group)).toEqual([6, 0, 0, 8, 0, 0, 8, 2, 0])
  })
})
