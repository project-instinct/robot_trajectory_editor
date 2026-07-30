import * as THREE from 'three'

export interface IkSolver {
  solve(chain: IkChain, target: THREE.Matrix4, currentAngles: number[]): number[]
}

export interface IkChain {
  jointNames: string[]
  jointTypes: string[]
  jointOrigins: THREE.Matrix4[]  // parent->joint transforms
  endEffectorOffset: THREE.Matrix4
}

export class CcdIkSolver implements IkSolver {
  maxIterations: number
  tolerance: number

  constructor(maxIterations = 50, tolerance = 0.001) {
    this.maxIterations = maxIterations
    this.tolerance = tolerance
  }

  solve(chain: IkChain, target: THREE.Matrix4, currentAngles: number[]): number[] {
    const angles = [...currentAngles]
    const n = chain.jointNames.length

    const worldTransforms: THREE.Matrix4[] = []
    for (let i = 0; i < n; i++) {
      const m = chain.jointOrigins[i].clone()
      if (chain.jointTypes[i] === 'revolute') m.multiply(makeRevJointTransform(angles[i]))
      worldTransforms.push(m)
    }

    for (let iter = 0; iter < this.maxIterations; iter++) {
      for (let j = n - 1; j >= 0; j--) {
        if (chain.jointTypes[j] !== 'revolute') continue

        const eePos = new THREE.Vector3()
        const targetPos = new THREE.Vector3()
        let composite = new THREE.Matrix4().identity()
        for (let k = 0; k <= n - 1; k++) {
          composite.multiply(worldTransforms[k])
        }
        composite.multiply(chain.endEffectorOffset)
        eePos.setFromMatrixPosition(composite)
        targetPos.setFromMatrixPosition(target)

        let chainStart = new THREE.Matrix4().identity()
        for (let k = 0; k < j; k++) chainStart.multiply(worldTransforms[k])

        const inv = chainStart.clone().invert()
        const localEe = eePos.clone().applyMatrix4(inv)
        const localTarget = targetPos.clone().applyMatrix4(inv)

        const v1 = localEe.clone().normalize()
        const v2 = localTarget.clone().normalize()
        const axis = new THREE.Vector3().crossVectors(v1, v2)
        if (axis.length() < 0.0001) continue
        axis.normalize()

        const cosAngle = Math.max(-1, Math.min(1, v1.dot(v2)))
        let angle = Math.acos(cosAngle)
        if (angle > this.tolerance) {
          angle = Math.min(angle, 0.3)
          angles[j] += angle
          worldTransforms[j] = chain.jointOrigins[j].clone().multiply(makeRevJointTransform(angles[j]))
        }

        if (new THREE.Vector3().subVectors(eePos, targetPos).length() < this.tolerance) break
      }
    }
    return angles
  }
}

function makeRevJointTransform(angle: number): THREE.Matrix4 {
  return new THREE.Matrix4().makeRotationZ(angle)
}
