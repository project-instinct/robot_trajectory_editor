import * as THREE from 'three'
import { OBJLoader } from 'three/examples/jsm/loaders/OBJLoader.js'
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js'
import { SimplifyModifier } from 'three/examples/jsm/modifiers/SimplifyModifier.js'

/** Loads a terrain mesh from .obj (text) or .stl (binary or ASCII) file content. */
export async function loadTerrain(data: ArrayBuffer | string, filename: string): Promise<THREE.Group> {
  const ext = filename.toLowerCase().split('.').pop()
  if (ext === 'stl') {
    const buf = typeof data === 'string' ? new TextEncoder().encode(data).buffer as ArrayBuffer : data
    const geometry = new STLLoader().parse(buf)
    geometry.computeVertexNormals()
    const mesh = new THREE.Mesh(
      geometry,
      new THREE.MeshStandardMaterial({ color: 0x7a8a99, side: THREE.DoubleSide }),
    )
    mesh.castShadow = true
    mesh.receiveShadow = true
    const group = new THREE.Group()
    group.add(mesh)
    return group
  }
  const text = typeof data === 'string' ? data : new TextDecoder().decode(data)
  const loader = new OBJLoader()
  const group = loader.parse(text)
  group.traverse((obj) => {
    if ((obj as THREE.Mesh).isMesh) {
      const mesh = obj as THREE.Mesh
      mesh.castShadow = true
      mesh.receiveShadow = true
    }
  })
  return group
}

/**
 * Bakes the current world transforms of all meshes into their geometry and
 * resets mesh/group transforms to identity, so subsequent edits and exports
 * operate on plain world-space vertices.
 */
export function bakeTerrainTransform(group: THREE.Group): void {
  group.updateMatrixWorld(true)
  const meshes: THREE.Mesh[] = []
  group.traverse((obj) => {
    if ((obj as THREE.Mesh).isMesh) meshes.push(obj as THREE.Mesh)
  })
  for (const mesh of meshes) {
    mesh.geometry.applyMatrix4(mesh.matrixWorld)
    mesh.position.set(0, 0, 0)
    mesh.quaternion.identity()
    mesh.scale.set(1, 1, 1)
    mesh.updateMatrix()
  }
  group.position.set(0, 0, 0)
  group.quaternion.identity()
  group.scale.set(1, 1, 1)
  group.updateMatrixWorld(true)
}

/** Keeps only triangles fully inside the given (world-space) box. */
export function cropTerrain(group: THREE.Group, box: THREE.Box3): void {
  bakeTerrainTransform(group)
  group.traverse((obj) => {
    if (!(obj as THREE.Mesh).isMesh) return
    const mesh = obj as THREE.Mesh
    const src = mesh.geometry.index ? mesh.geometry.toNonIndexed() : mesh.geometry
    const pos = src.getAttribute('position') as THREE.BufferAttribute
    const kept: number[] = []
    const v = new THREE.Vector3()
    for (let i = 0; i < pos.count; i += 3) {
      let inside = true
      for (let k = 0; k < 3; k++) {
        v.fromBufferAttribute(pos, i + k)
        if (!box.containsPoint(v)) { inside = false; break }
      }
      if (inside) {
        for (let k = 0; k < 3; k++) kept.push(pos.getX(i + k), pos.getY(i + k), pos.getZ(i + k))
      }
    }
    const geom = new THREE.BufferGeometry()
    geom.setAttribute('position', new THREE.Float32BufferAttribute(kept, 3))
    geom.computeVertexNormals()
    mesh.geometry.dispose()
    mesh.geometry = geom
  })
}

/** Reduces vertex count by the given ratio (0..1) using mesh simplification. */
export function downsampleTerrain(group: THREE.Group, ratio: number): void {
  const modifier = new SimplifyModifier()
  bakeTerrainTransform(group)
  group.traverse((obj) => {
    if (!(obj as THREE.Mesh).isMesh) return
    const mesh = obj as THREE.Mesh
    const vertexCount = mesh.geometry.getAttribute('position').count
    const removeCount = Math.min(
      Math.floor(vertexCount * THREE.MathUtils.clamp(ratio, 0, 0.95)),
      Math.max(vertexCount - 4, 0),
    )
    if (removeCount <= 0) return
    const simplified = modifier.modify(mesh.geometry, removeCount)
    simplified.computeVertexNormals()
    mesh.geometry.dispose()
    mesh.geometry = simplified
  })
}

/** Exports the terrain in world coordinates (transforms baked in). */
export function exportTerrainObj(group: THREE.Group): string {
  group.updateMatrixWorld(true)
  let obj = '# Terrain exported by Robot Trajectory Editor\n'
  let vi = 1

  const v = new THREE.Vector3()
  const n = new THREE.Vector3()

  group.traverse((object) => {
    if (!(object as THREE.Mesh).isMesh) return
    const mesh = object as THREE.Mesh
    const geom = mesh.geometry.index ? mesh.geometry.toNonIndexed() : mesh.geometry
    const pos = geom.getAttribute('position') as THREE.BufferAttribute

    obj += `o ${mesh.name || 'mesh'}\n`
    for (let i = 0; i < pos.count; i++) {
      v.fromBufferAttribute(pos, i).applyMatrix4(mesh.matrixWorld)
      obj += `v ${v.x} ${v.y} ${v.z}\n`
    }
    // Recompute flat-shaded normals per triangle in world space.
    const a = new THREE.Vector3()
    const b = new THREE.Vector3()
    const c = new THREE.Vector3()
    const ab = new THREE.Vector3()
    const ac = new THREE.Vector3()
    for (let i = 0; i < pos.count; i += 3) {
      a.fromBufferAttribute(pos, i).applyMatrix4(mesh.matrixWorld)
      b.fromBufferAttribute(pos, i + 1).applyMatrix4(mesh.matrixWorld)
      c.fromBufferAttribute(pos, i + 2).applyMatrix4(mesh.matrixWorld)
      n.crossVectors(ab.subVectors(b, a), ac.subVectors(c, a)).normalize()
      for (let k = 0; k < 3; k++) obj += `vn ${n.x} ${n.y} ${n.z}\n`
    }
    for (let i = 0; i < pos.count; i += 3) {
      const f0 = i + vi
      const f1 = i + 1 + vi
      const f2 = i + 2 + vi
      obj += `f ${f0}//${f0} ${f1}//${f1} ${f2}//${f2}\n`
    }
    vi += pos.count
  })
  return obj
}
