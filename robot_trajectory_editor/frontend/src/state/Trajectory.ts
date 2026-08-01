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

  clone(): Trajectory {
    const copy = new Trajectory(
      this.framerate,
      [...this.jointNames],
      new Float32Array(this.jointPos),
      new Float32Array(this.basePoseW),
      new Float32Array(this.baseQuatW),
    )
    copy.keyframes = new Set(this.keyframes)
    return copy
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

  /** Fill one channel over [start, end] with a constant value, leaving all
   *  other channels (and, for baseQuat, the other euler axes) untouched. */
  fillChannelRange(start: number, end: number, channel: ChannelKind, value: number): void {
    for (let f = start; f <= end; f++) this.setChannelValue(f, channel, value)
  }

  /** Linearly interpolate one channel between the selected endpoints. */
  interpolateChannelRange(start: number, end: number, channel: ChannelKind): void {
    const first = Math.min(start, end)
    const last = Math.max(start, end)
    if (last <= first) return
    const firstValue = this.getChannelValue(first, channel)
    const lastValue = this.getChannelValue(last, channel)
    const span = last - first
    for (let frame = first + 1; frame < last; frame++) {
      const alpha = (frame - first) / span
      this.setChannelValue(frame, channel, firstValue + alpha * (lastValue - firstValue))
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
   * Low-pass smoothing of one channel over [start, end]: a Gaussian-weighted
   * moving average that removes high-frequency (frame-to-frame) jitter while
   * preserving the overall shape of the motion. Only frames strictly inside
   * the segment are modified: the endpoints and keyframe anchors are kept
   * exactly, and the smoothing strength tapers to zero at the segment edges
   * so the result stays continuous with the untouched frames outside.
   *
   * @param sigma Gaussian kernel width in frames; larger = stronger smoothing.
   */
  smoothRange(start: number, end: number, channel: ChannelKind, sigma: number = SMOOTH_SIGMA_FRAMES): void {
    if (end <= start || !(sigma > 0)) return
    if (channel.kind === 'baseQuat') {
      this.smoothQuatChannel(start, end, sigma)
      return
    }
    const n = end - start + 1
    const values = new Float64Array(n)
    for (let f = start; f <= end; f++) {
      values[f - start] = channel.kind === 'joint'
        ? this.jointPos[f * this.jointCount + channel.index]
        : this.basePoseW[f * 3 + channel.axis]
    }
    const kernel = gaussianKernel(sigma)
    const smoothed = gaussianConvolve(values, kernel)
    const radius = (kernel.length - 1) / 2
    for (let f = start + 1; f < end; f++) {
      if (this.keyframes.has(f)) continue
      const i = f - start
      const v = values[i] + smoothTaper(i, n, radius) * (smoothed[i] - values[i])
      if (channel.kind === 'joint') this.jointPos[f * this.jointCount + channel.index] = v
      else this.basePoseW[f * 3 + channel.axis] = v
    }
  }

  /**
   * Smooths the whole base-orientation sequence (any baseQuat channel smooths
   * the coupled quaternion, not a single euler axis, to avoid angle-wrap
   * artifacts): hemisphere-fix the segment (chained from the frame before the
   * segment when present, so the filter input is continuous with the
   * untouched prefix), Gaussian-filter per component, renormalize.
   */
  private smoothQuatChannel(start: number, end: number, sigma: number): void {
    const n = end - start + 1
    const readQuat = (f: number): number[] => [
      this.baseQuatW[f * 4], this.baseQuatW[f * 4 + 1],
      this.baseQuatW[f * 4 + 2], this.baseQuatW[f * 4 + 3],
    ]
    const quats: number[][] = []
    let prev = readQuat(start > 0 ? start - 1 : start)
    for (let f = start; f <= end; f++) {
      const q = readQuat(f)
      const dot = q[0] * prev[0] + q[1] * prev[1] + q[2] * prev[2] + q[3] * prev[3]
      if (dot < 0) { q[0] = -q[0]; q[1] = -q[1]; q[2] = -q[2]; q[3] = -q[3] }
      quats.push(q)
      prev = q
    }
    const kernel = gaussianKernel(sigma)
    const smoothed = [0, 1, 2, 3].map(c => gaussianConvolve(quats.map(q => q[c]), kernel))
    const radius = (kernel.length - 1) / 2
    for (let f = start + 1; f < end; f++) {
      if (this.keyframes.has(f)) continue
      const i = f - start
      const w = smoothTaper(i, n, radius)
      const o = quats[i]
      const qw = o[0] + w * (smoothed[0][i] - o[0])
      const qx = o[1] + w * (smoothed[1][i] - o[1])
      const qy = o[2] + w * (smoothed[2][i] - o[2])
      const qz = o[3] + w * (smoothed[3][i] - o[3])
      const norm = Math.hypot(qw, qx, qy, qz) || 1
      this.baseQuatW[f * 4] = qw / norm
      this.baseQuatW[f * 4 + 1] = qx / norm
      this.baseQuatW[f * 4 + 2] = qy / norm
      this.baseQuatW[f * 4 + 3] = qz / norm
    }
  }
}

/** Default Gaussian kernel width (frames) for smoothRange: kills frame-to-frame
 *  jitter while keeping motion slower than a few frames essentially intact. */
const SMOOTH_SIGMA_FRAMES = 2

/** Normalized Gaussian kernel truncated at ±3σ. */
function gaussianKernel(sigma: number): Float64Array {
  const radius = Math.max(1, Math.ceil(3 * sigma))
  const kernel = new Float64Array(2 * radius + 1)
  let sum = 0
  for (let i = -radius; i <= radius; i++) {
    const v = Math.exp(-(i * i) / (2 * sigma * sigma))
    kernel[i + radius] = v
    sum += v
  }
  for (let i = 0; i < kernel.length; i++) kernel[i] /= sum
  return kernel
}

/**
 * Gaussian low-pass filter. Samples outside the array are dropped and the
 * remaining weights renormalized, so edge frames stay near the edge values
 * instead of being pulled toward zero.
 */
function gaussianConvolve(values: ArrayLike<number>, kernel: Float64Array): Float64Array {
  const n = values.length
  const radius = (kernel.length - 1) / 2
  const out = new Float64Array(n)
  for (let i = 0; i < n; i++) {
    let acc = 0
    let wsum = 0
    const lo = Math.max(0, i - radius)
    const hi = Math.min(n - 1, i + radius)
    for (let j = lo; j <= hi; j++) {
      const w = kernel[j - i + radius]
      acc += w * values[j]
      wsum += w
    }
    out[i] = acc / wsum
  }
  return out
}

/** Smoothing strength for segment frame i (of n): 0 at the edges, ramping
 *  linearly to 1 one kernel-radius in, so the segment blends into the
 *  untouched frames outside instead of creating a step. */
function smoothTaper(i: number, n: number, radius: number): number {
  return Math.min(1, i / radius, (n - 1 - i) / radius)
}
