import * as THREE from 'three'
import { OBJLoader } from 'three/examples/jsm/loaders/OBJLoader.js'

export interface TerrainMesh {
  group: THREE.Group
  simplify(ratio: number): void
  crop(xMin: number, xMax: number, yMin: number, yMax: number, zMin: number, zMax: number): void
}

export async function loadTerrainObj(data: string): Promise<THREE.Group> {
  const loader = new OBJLoader()
  const group = loader.parse(data)
  group.traverse((obj) => {
    if ((obj as THREE.Mesh).isMesh) {
      const mesh = obj as THREE.Mesh
      mesh.castShadow = true
      mesh.receiveShadow = true
    }
  })
  return group
}

export function exportTerrainObj(group: THREE.Group): string {
  let obj = '# Terrain exported by Robot Trajectory Editor\n'
  let vi = 1

  group.traverse((object) => {
    if (!(object as THREE.Mesh).isMesh) return
    const mesh = object as THREE.Mesh
    const geom = mesh.geometry
    const pos = geom.getAttribute('position') as THREE.BufferAttribute
    const norm = geom.getAttribute('normal')

    obj += `o ${mesh.name || 'mesh'}\n`
    for (let i = 0; i < pos.count; i++) {
      obj += `v ${pos.getX(i)} ${pos.getY(i)} ${pos.getZ(i)}\n`
    }
    if (norm) {
      for (let i = 0; i < norm.count; i++) {
        obj += `vn ${(norm as THREE.BufferAttribute).getX(i)} ${(norm as THREE.BufferAttribute).getY(i)} ${(norm as THREE.BufferAttribute).getZ(i)}\n`
      }
    }
    if (geom.index) {
      const idx = geom.index
      for (let i = 0; i < idx.count; i += 3) {
        const a = idx.getX(i) + vi
        const b = idx.getX(i + 1) + vi
        const c = idx.getX(i + 2) + vi
        obj += norm ? `f ${a}//${a} ${b}//${b} ${c}//${c}\n` : `f ${a} ${b} ${c}\n`
      }
    }
    vi += pos.count
  })
  return obj
}
