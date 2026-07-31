export type ChannelKind = { kind: 'joint'; index: number } | { kind: 'basePos'; axis: number } | { kind: 'baseQuat'; axis: number }

function quatToEuler(w: number, x: number, y: number, z: number): [number, number, number] {
  const sinr_cosp = 2 * (w * x + y * z)
  const cosr_cosp = 1 - 2 * (x * x + y * y)
  const roll = Math.atan2(sinr_cosp, cosr_cosp)

  const sinp = 2 * (w * y - z * x)
  let pitch: number
  if (Math.abs(sinp) >= 1) {
    pitch = Math.sign(sinp) * Math.PI / 2
  } else {
    pitch = Math.asin(sinp)
  }

  const siny_cosp = 2 * (w * z + x * y)
  const cosy_cosp = 1 - 2 * (y * y + z * z)
  const yaw = Math.atan2(siny_cosp, cosy_cosp)

  return [roll, pitch, yaw]
}

function eulerToQuat(roll: number, pitch: number, yaw: number): [number, number, number, number] {
  const cr = Math.cos(roll * 0.5)
  const sr = Math.sin(roll * 0.5)
  const cp = Math.cos(pitch * 0.5)
  const sp = Math.sin(pitch * 0.5)
  const cy = Math.cos(yaw * 0.5)
  const sy = Math.sin(yaw * 0.5)

  const w = cr * cp * cy + sr * sp * sy
  const x = sr * cp * cy - cr * sp * sy
  const y = cr * sp * cy + sr * cp * sy
  const z = cr * cp * sy - sr * sp * cy
  return [w, x, y, z]
}

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
    // Derive frame count from base data when there are no joints (e.g. base-only trajectories)
    this.frameCount = this.jointCount > 0
      ? Math.floor(jointPos.length / this.jointCount)
      : Math.floor(basePoseW.length / 3)
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
    const w = this.baseQuatW[frame * 4]
    const x = this.baseQuatW[frame * 4 + 1]
    const y = this.baseQuatW[frame * 4 + 2]
    const z = this.baseQuatW[frame * 4 + 3]
    const [roll, pitch, yaw] = quatToEuler(w, x, y, z)
    return channel.axis === 0 ? roll : channel.axis === 1 ? pitch : yaw
  }

  getQuatEuler(frame: number): [number, number, number] {
    const w = this.baseQuatW[frame * 4]
    const x = this.baseQuatW[frame * 4 + 1]
    const y = this.baseQuatW[frame * 4 + 2]
    const z = this.baseQuatW[frame * 4 + 3]
    return quatToEuler(w, x, y, z)
  }

  setQuatEuler(frame: number, roll: number, pitch: number, yaw: number): void {
    const [w, x, y, z] = eulerToQuat(roll, pitch, yaw)
    this.baseQuatW[frame * 4] = w
    this.baseQuatW[frame * 4 + 1] = x
    this.baseQuatW[frame * 4 + 2] = y
    this.baseQuatW[frame * 4 + 3] = z
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
    for (let f = 0; f < count; f++) {
      const [roll, pitch, yaw] = quatToEuler(
        this.baseQuatW[f * 4], this.baseQuatW[f * 4 + 1], this.baseQuatW[f * 4 + 2], this.baseQuatW[f * 4 + 3])
      out[f] = channel.axis === 0 ? roll : channel.axis === 1 ? pitch : yaw
    }
    return out
  }

  fillRange(start: number, end: number, state: FrameState): void {
    for (let f = start; f <= end; f++) {
      for (let j = 0; j < this.jointCount; j++) this.jointPos[f * this.jointCount + j] = state.jointPos[j]
      for (let i = 0; i < 3; i++) this.basePoseW[f * 3 + i] = state.basePoseW[i]
      for (let i = 0; i < 4; i++) this.baseQuatW[f * 4 + i] = state.baseQuatW[i]
    }
  }

  setChannelValue(frame: number, channel: ChannelKind, value: number): void {
    if (channel.kind === 'joint') {
      this.jointPos[frame * this.jointCount + channel.index] = value
    } else if (channel.kind === 'basePos') {
      this.basePoseW[frame * 3 + channel.axis] = value
    } else {
      const [roll, pitch, yaw] = this.getQuatEuler(frame)
      const r = channel.axis === 0 ? value : roll
      const p = channel.axis === 1 ? value : pitch
      const y = channel.axis === 2 ? value : yaw
      this.setQuatEuler(frame, r, p, y)
    }
  }

  /**
   * Knots for cubic-spline smoothing: segment endpoints plus any keyframe
   * anchors inside [start, end]. Endpoints are always preserved.
   */
  private smoothKnots(start: number, end: number): number[] {
    const knots = new Set<number>([start, end])
    for (const f of this.keyframes) {
      if (f > start && f < end) knots.add(f)
    }
    return [...knots].sort((a, b) => a - b)
  }

  smoothRange(start: number, end: number, channel: ChannelKind): void {
    if (end <= start) return
    const knots = this.smoothKnots(start, end)
    if (channel.kind === 'joint') {
      const spline = cubicSpline(knots, knots.map(f => this.jointPos[f * this.jointCount + channel.index]))
      for (let f = start; f <= end; f++) {
        if (f === start || f === end || this.keyframes.has(f)) continue
        this.jointPos[f * this.jointCount + channel.index] = spline(f)
      }
    } else if (channel.kind === 'basePos') {
      const spline = cubicSpline(knots, knots.map(f => this.basePoseW[f * 3 + channel.axis]))
      for (let f = start; f <= end; f++) {
        if (f === start || f === end || this.keyframes.has(f)) continue
        this.basePoseW[f * 3 + channel.axis] = spline(f)
      }
    } else {
      this.smoothQuatChannel(start, end, knots)
    }
  }

  /**
   * Smooths the whole base-orientation sequence (any baseQuat channel smooths
   * the coupled quaternion, not a single euler axis, to avoid angle-wrap
   * artifacts): hemisphere-fix knot quats, spline per component, renormalize.
   */
  private smoothQuatChannel(start: number, end: number, knots: number[]): void {
    const knotQuats: number[][] = []
    let prev: number[] | null = null
    for (const f of knots) {
      const q = [
        this.baseQuatW[f * 4], this.baseQuatW[f * 4 + 1],
        this.baseQuatW[f * 4 + 2], this.baseQuatW[f * 4 + 3],
      ]
      if (prev) {
        const dot = q[0] * prev[0] + q[1] * prev[1] + q[2] * prev[2] + q[3] * prev[3]
        if (dot < 0) { q[0] = -q[0]; q[1] = -q[1]; q[2] = -q[2]; q[3] = -q[3] }
      }
      knotQuats.push(q)
      prev = q
    }
    const splines = [0, 1, 2, 3].map(c => cubicSpline(knots, knotQuats.map(q => q[c])))
    for (let f = start; f <= end; f++) {
      if (f === start || f === end || this.keyframes.has(f)) continue
      const w = splines[0](f), x = splines[1](f), y = splines[2](f), z = splines[3](f)
      const n = Math.hypot(w, x, y, z) || 1
      this.baseQuatW[f * 4] = w / n
      this.baseQuatW[f * 4 + 1] = x / n
      this.baseQuatW[f * 4 + 2] = y / n
      this.baseQuatW[f * 4 + 3] = z / n
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
