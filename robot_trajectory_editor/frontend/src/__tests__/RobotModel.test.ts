import { describe, expect, it } from 'vitest'
import * as THREE from 'three'
import type { URDFRobot } from 'urdf-loader'
import { RobotModel } from '../three/RobotModel'
import { eulerToQuat, quatToEuler } from '../state/Trajectory'
import { useStore } from '../state/store'

interface MockJoint extends THREE.Group {
  jointType: string
  angle: number
  jointValue: number[]
  limit?: { lower: number; upper: number }
}

function makeMockUrdfRobot(): URDFRobot {
  const root = new THREE.Group() as THREE.Group & {
    joints: Record<string, MockJoint>
    setJointValue: (name: string, value: number) => boolean
  }

  const baseLink = new THREE.Group() as THREE.Group & { isURDFLink?: boolean }
  baseLink.name = 'pelvis'
  baseLink.isURDFLink = true
  root.add(baseLink)

  const makeJoint = (name: string, jointType: string, initial: number, limit?: { lower: number; upper: number }): MockJoint => {
    const obj = Object.assign(new THREE.Group(), {
      jointType,
      angle: initial,
      jointValue: jointType === 'fixed' ? [] : [initial],
      limit,
    })
    obj.name = name
    return obj
  }

  root.joints = {
    fixed_sensor: makeJoint('fixed_sensor', 'fixed', 0),
    hip_pitch: makeJoint('hip_pitch', 'revolute', 0.25, { lower: -1.5, upper: 1.5 }),
    wheel_spin: makeJoint('wheel_spin', 'continuous', -0.5, { lower: 0, upper: 0 }),
  }

  root.setJointValue = (name: string, value: number) => {
    const joint = root.joints[name]
    if (!joint) return false
    joint.angle = value
    joint.jointValue = [value]
    return true
  }

  return root as unknown as URDFRobot
}

describe('RobotModel live scene state without a trajectory', () => {
  it('reads movable joints and base pose directly from the 3D scene', () => {
    const model = new RobotModel(makeMockUrdfRobot(), [])
    const state = model.getSceneState()

    expect(state.jointNames).toEqual(['hip_pitch', 'wheel_spin'])
    expect(Array.from(state.jointPos)).toEqual([0.25, -0.5])
    expect(Array.from(state.basePoseW)).toEqual([0, 0, 0])
    expect(Array.from(state.baseQuatW)).toEqual([1, 0, 0, 0])
  })

  it('resolves joint limits even when no trajectory joints are bound', () => {
    const model = new RobotModel(makeMockUrdfRobot(), [])

    expect(model.getJointLimit('hip_pitch')).toEqual({ lower: -1.5, upper: 1.5 })
    expect(model.getJointLimit('wheel_spin')).toEqual({ lower: -Math.PI, upper: Math.PI })
    expect(model.getJointLimit('nonexistent')).toBeNull()
  })

  it('updates joint values and base pose directly in the 3D scene', () => {
    const model = new RobotModel(makeMockUrdfRobot(), [])
    model.setJointValue('hip_pitch', 0.75)
    model.setBasePose([1.2, -0.4, 0.85], eulerToQuat(0.1, -0.2, 0.5))

    const state = model.getSceneState()
    expect(state.jointPos[0]).toBeCloseTo(0.75)
    expect(state.basePoseW[0]).toBeCloseTo(1.2)
    expect(state.basePoseW[1]).toBeCloseTo(-0.4)
    expect(state.basePoseW[2]).toBeCloseTo(0.85)
    const [roll, pitch, yaw] = quatToEuler(
      state.baseQuatW[0], state.baseQuatW[1], state.baseQuatW[2], state.baseQuatW[3],
    )
    expect(roll).toBeCloseTo(0.1)
    expect(pitch).toBeCloseTo(-0.2)
    expect(yaw).toBeCloseTo(0.5)
  })

  it('bumps robotPoseVersion in the store when touchRobotPose is called', () => {
    const before = useStore.getState().robotPoseVersion
    useStore.getState().touchRobotPose()
    expect(useStore.getState().robotPoseVersion).toBe(before + 1)
  })
})
