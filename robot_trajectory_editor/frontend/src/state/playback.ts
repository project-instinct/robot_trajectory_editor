import type { FrameState, Trajectory } from './Trajectory'

/** Fractional frame shown `elapsedSeconds` after playback started at frame
 *  `start`, at the trajectory's own frame rate. */
export function playbackPosition(start: number, elapsedSeconds: number, framerate: number): number {
  return start + Math.max(elapsedSeconds, 0) * framerate
}

/** Pose between the two frames around a fractional position, for display only:
 *  joints and base position are blended linearly, the base orientation along
 *  the shorter arc. The trajectory itself is never changed. */
export function interpolatedFrame(trajectory: Trajectory, position: number): FrameState {
  const last = trajectory.frameCount - 1
  const clamped = Math.min(Math.max(position, 0), last)
  const before = Math.floor(clamped)
  const after = Math.min(before + 1, last)
  const t = clamped - before
  const a = trajectory.getFrame(before)
  if (t === 0 || after === before) return a
  const b = trajectory.getFrame(after)
  const lerp = (x: Float32Array, y: Float32Array) => x.map((v, i) => v + (y[i] - v) * t)
  return { jointPos: lerp(a.jointPos, b.jointPos), basePoseW: lerp(a.basePoseW, b.basePoseW),
    baseQuatW: slerp(a.baseQuatW, b.baseQuatW, t) }
}

/** Spherical interpolation of wxyz quaternions. */
export function slerp(q0: Float32Array, q1: Float32Array, t: number): Float32Array {
  let dot = q0[0] * q1[0] + q0[1] * q1[1] + q0[2] * q1[2] + q0[3] * q1[3]
  const sign = dot < 0 ? -1 : 1
  dot *= sign
  let w0 = 1 - t
  let w1 = t * sign
  if (dot < 0.9995) {
    const angle = Math.acos(Math.min(dot, 1))
    const s = Math.sin(angle)
    w0 = Math.sin((1 - t) * angle) / s
    w1 = sign * Math.sin(t * angle) / s
  }
  const q = q0.map((v, i) => w0 * v + w1 * q1[i])
  const norm = Math.hypot(q[0], q[1], q[2], q[3])
  return q.map(v => v / norm)
}
