export type ChannelKind = { kind: 'joint'; index: number } | { kind: 'basePos'; axis: number } | { kind: 'baseQuat'; axis: number }

export interface FrameState {
  jointPos: Float32Array
  basePoseW: Float32Array
  baseQuatW: Float32Array
}

export interface TrajectoryJSON {
  framerate: number
  joint_names: string[]
  joint_pos: number[][]
  base_pos_w: number[][]
  base_quat_w: number[][]
}

export class Trajectory {
  framerate: number
  jointNames: string[]
  jointPos: Float32Array
  basePoseW: Float32Array
  baseQuatW: Float32Array
  keyframes: Set<number>
  frameCount: number
  jointCount: number

  constructor(
    framerate: number,
    jointNames: string[],
    jointPos: Float32Array,
    basePoseW: Float32Array,
    baseQuatW: Float32Array,
  ) {
    this.framerate = framerate
    this.jointNames = jointNames
    this.jointCount = jointNames.length
    this.frameCount = jointPos.length / Math.max(jointNames.length, 1)
    this.jointPos = jointPos
    this.basePoseW = basePoseW
    this.baseQuatW = baseQuatW
    this.keyframes = new Set()
  }

  static empty(): Trajectory {
    const jointNames: string[] = []
    return new Trajectory(
      30,
      jointNames,
      new Float32Array(0),
      new Float32Array(0),
      new Float32Array(0),
    )
  }

  static fromJSON(data: TrajectoryJSON): Trajectory {
    const jointNames = data.joint_names
    return new Trajectory(
      data.framerate,
      jointNames,
      new Float32Array(data.joint_pos.flat()),
      new Float32Array(data.base_pos_w.flat()),
      new Float32Array(data.base_quat_w.flat()),
    )
  }

  toJSON(): TrajectoryJSON {
    const jc = this.jointCount
    const joint_pos: number[][] = []
    const base_pos_w: number[][] = []
    const base_quat_w: number[][] = []
    for (let f = 0; f < this.frameCount; f++) {
      const jp: number[] = []
      for (let j = 0; j < jc; j++) jp.push(this.jointPos[f * jc + j])
      joint_pos.push(jp)
      base_pos_w.push([this.basePoseW[f * 3], this.basePoseW[f * 3 + 1], this.basePoseW[f * 3 + 2]])
      base_quat_w.push([this.baseQuatW[f * 4], this.baseQuatW[f * 4 + 1], this.baseQuatW[f * 4 + 2], this.baseQuatW[f * 4 + 3]])
    }
    return {
      framerate: this.framerate,
      joint_names: this.jointNames,
      joint_pos,
      base_pos_w,
      base_quat_w,
    }
  }

  getFrame(frame: number): FrameState {
    const jc = this.jointCount
    const jStart = frame * jc
    return {
      jointPos: this.jointPos.slice(jStart, jStart + jc),
      basePoseW: this.basePoseW.slice(frame * 3, frame * 3 + 3),
      baseQuatW: this.baseQuatW.slice(frame * 4, frame * 4 + 4),
    }
  }

  setJointValue(frame: number, jointIdx: number, value: number): void {
    this.jointPos[frame * this.jointCount + jointIdx] = value
  }

  setBasePose(frame: number, pos: Float32Array | number[], quat: Float32Array | number[]): void {
    this.basePoseW[frame * 3] = pos[0]
    this.basePoseW[frame * 3 + 1] = pos[1]
    this.basePoseW[frame * 3 + 2] = pos[2]
    this.baseQuatW[frame * 4] = quat[0]
    this.baseQuatW[frame * 4 + 1] = quat[1]
    this.baseQuatW[frame * 4 + 2] = quat[2]
    this.baseQuatW[frame * 4 + 3] = quat[3]
  }

  insertKeyframe(frame: number): void { this.keyframes.add(frame) }
  removeKeyframe(frame: number): void { this.keyframes.delete(frame) }
  hasKeyframe(frame: number): boolean { return this.keyframes.has(frame) }

  getChannelValue(frame: number, channel: ChannelKind): number {
    if (channel.kind === 'joint') return this.jointPos[frame * this.jointCount + channel.index]
    if (channel.kind === 'basePos') return this.basePoseW[frame * 3 + channel.axis]
    return this.baseQuatW[frame * 4 + channel.axis]
  }

  getChannelValues(channel: ChannelKind): Float32Array {
    const count = this.frameCount
    if (channel.kind === 'joint') {
      const out = new Float32Array(count)
      for (let f = 0; f < count; f++) out[f] = this.jointPos[f * this.jointCount + channel.index]
      return out
    }
    if (channel.kind === 'basePos') {
      const out = new Float32Array(count)
      for (let f = 0; f < count; f++) out[f] = this.basePoseW[f * 3 + channel.axis]
      return out
    }
    const out = new Float32Array(count)
    for (let f = 0; f < count; f++) out[f] = this.baseQuatW[f * 4 + channel.axis]
    return out
  }

  fillRange(start: number, end: number, state: FrameState): void {
    for (let f = start; f <= end; f++) {
      for (let j = 0; j < this.jointCount; j++) this.jointPos[f * this.jointCount + j] = state.jointPos[j]
      for (let i = 0; i < 3; i++) this.basePoseW[f * 3 + i] = state.basePoseW[i]
      for (let i = 0; i < 4; i++) this.baseQuatW[f * 4 + i] = state.baseQuatW[i]
    }
  }

  fillChannel(start: number, end: number, channel: ChannelKind, value: number): void {
    if (channel.kind === 'joint') {
      for (let f = start; f <= end; f++) this.jointPos[f * this.jointCount + channel.index] = value
    } else if (channel.kind === 'basePos') {
      for (let f = start; f <= end; f++) this.basePoseW[f * 3 + channel.axis] = value
    } else {
      for (let f = start; f <= end; f++) this.baseQuatW[f * 4 + channel.axis] = value
    }
  }

  smoothRange(start: number, end: number, channel: ChannelKind): void {
    if (channel.kind === 'joint') {
      this.smoothJointChannel(start, end, channel.index)
    } else if (channel.kind === 'basePos') {
      this.smoothBaseChannel(start, end, channel.axis, 'pos')
    } else {
      this.smoothBaseChannel(start, end, channel.axis, 'quat')
      for (let f = start; f <= end; f++) {
        const x = this.baseQuatW[f * 4]
        const y = this.baseQuatW[f * 4 + 1]
        const z = this.baseQuatW[f * 4 + 2]
        const w = this.baseQuatW[f * 4 + 3]
        const mag = Math.sqrt(x * x + y * y + z * z + w * w)
        if (mag > 0.001) {
          this.baseQuatW[f * 4] = x / mag
          this.baseQuatW[f * 4 + 1] = y / mag
          this.baseQuatW[f * 4 + 2] = z / mag
          this.baseQuatW[f * 4 + 3] = w / mag
        }
      }
    }
  }

  private smoothJointChannel(start: number, end: number, jointIdx: number): void {
    const values: number[] = []
    const indices: number[] = []
    for (let f = start; f <= end; f++) {
      if (this.keyframes.has(f)) { values.push(this.jointPos[f * this.jointCount + jointIdx]); indices.push(f) }
    }
    if (indices.length < 2) return
    const spline = cubicSpline(indices, values)
    for (let f = start; f <= end; f++) {
      this.jointPos[f * this.jointCount + jointIdx] = spline(f)
    }
  }

  private smoothBaseChannel(start: number, end: number, axis: number, kind: 'pos' | 'quat'): void {
    const stride = kind === 'pos' ? 3 : 4
    const arr = kind === 'pos' ? this.basePoseW : this.baseQuatW
    const values: number[] = []
    const indices: number[] = []
    for (let f = start; f <= end; f++) {
      if (this.keyframes.has(f)) { values.push(arr[f * stride + axis]); indices.push(f) }
    }
    if (indices.length < 2) return
    const spline = cubicSpline(indices, values)
    for (let f = start; f <= end; f++) {
      arr[f * stride + axis] = spline(f)
    }
  }
}

function cubicSpline(xs: number[], ys: number[]): (x: number) => number {
  const n = xs.length
  const a = ys.slice()
  const b = new Array(n).fill(0)
  const d = new Array(n).fill(0)
  const h: number[] = []
  for (let i = 0; i < n - 1; i++) h.push(xs[i + 1] - xs[i])

  const alpha: number[] = [0]
  for (let i = 1; i < n - 1; i++) {
    alpha.push((3 / h[i]) * (a[i + 1] - a[i]) - (3 / h[i - 1]) * (a[i] - a[i - 1]))
  }
  alpha.push(0)

  const l: number[] = [1]
  const mu: number[] = [0]
  const z: number[] = [0]

  for (let i = 1; i < n; i++) {
    l.push(2 * (xs[i] - xs[i - 1]) - h[i - 1] * mu[i - 1])
    mu.push(h[i - 1] / l[i])
    z.push((alpha[i] - h[i - 1] * z[i - 1]) / l[i])
  }

  const c2: number[] = new Array(n).fill(0)
  c2[n - 1] = 0
  for (let j = n - 2; j >= 0; j--) {
    c2[j] = z[j] - mu[j] * c2[j + 1]
    b[j] = (a[j + 1] - a[j]) / h[j] - (h[j] * (c2[j + 1] + 2 * c2[j])) / 3
    d[j] = (c2[j + 1] - c2[j]) / (3 * h[j])
  }

  return (x: number): number => {
    let i = xs.length - 2
    for (let k = 0; k < xs.length - 1; k++) {
      if (x >= xs[k] && x <= xs[k + 1]) { i = k; break }
    }
    const dx = x - xs[i]
    return a[i] + b[i] * dx + c2[i] * dx * dx + d[i] * dx * dx * dx
  }
}
