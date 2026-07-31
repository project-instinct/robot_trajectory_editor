import { useStore } from '../state/store'
import { useRef, useEffect, useCallback } from 'react'

type DragState =
  | { type: 'scrub' }
  | { type: 'segment' }
  | { type: 'keyframe'; startClientY: number; startValue: number; curFrame: number }
  | null

const KF_HIT_PX = 6

/**
 * Timeline interactions:
 * - click / drag: scrub playhead
 * - Shift+drag: select segment (used by Fill/Smooth)
 * - double-click: add keyframe anchor at frame
 * - drag keyframe horizontally: retime anchor; vertically: edit selected channel value
 * - right-click keyframe: remove it
 */
export function Timeline() {
  const {
    trajectory, trajectoryVersion, currentFrame, setCurrentFrame,
    selectedChannel, segmentStart, segmentEnd, setSegment, touchTrajectory,
  } = useStore()
  const canvasRef = useRef<HTMLCanvasElement>(null)
  const dragRef = useRef<DragState>(null)
  // Value range of the currently drawn curve, for vertical keyframe dragging.
  const scaleRef = useRef<{ vMin: number; vMax: number }>({ vMin: 0, vMax: 1 })

  const frameFromX = useCallback((clientX: number): number => {
    const rect = canvasRef.current?.getBoundingClientRect()
    if (!rect || trajectory.frameCount < 1) return 0
    return Math.max(0, Math.min(
      Math.round(((clientX - rect.left) / rect.width) * (trajectory.frameCount - 1)),
      trajectory.frameCount - 1,
    ))
  }, [trajectory.frameCount])

  const keyframeAtX = useCallback((clientX: number): number | null => {
    const rect = canvasRef.current?.getBoundingClientRect()
    if (!rect || trajectory.frameCount < 2) return null
    let best: number | null = null
    let bestDist = KF_HIT_PX + 1
    for (const f of trajectory.keyframes) {
      const x = (f / (trajectory.frameCount - 1)) * rect.width
      const d = Math.abs(clientX - rect.left - x)
      if (d < bestDist) { bestDist = d; best = f }
    }
    return best
  }, [trajectory])

  const handleMouseDown = useCallback((e: React.MouseEvent<HTMLCanvasElement>) => {
    if (trajectory.frameCount < 1) return
    const kf = keyframeAtX(e.clientX)
    if (kf !== null && !e.shiftKey) {
      const startValue = selectedChannel ? trajectory.getChannelValue(kf, selectedChannel) : 0
      dragRef.current = { type: 'keyframe', startClientY: e.clientY, startValue, curFrame: kf }
      return
    }
    if (e.shiftKey) {
      dragRef.current = { type: 'segment' }
      setSegment(frameFromX(e.clientX), null)
      return
    }
    dragRef.current = { type: 'scrub' }
    setCurrentFrame(frameFromX(e.clientX))
  }, [trajectory, selectedChannel, keyframeAtX, frameFromX, setCurrentFrame, setSegment])

  const handleDoubleClick = useCallback((e: React.MouseEvent<HTMLCanvasElement>) => {
    if (trajectory.frameCount < 1) return
    trajectory.insertKeyframe(frameFromX(e.clientX))
    touchTrajectory()
  }, [trajectory, frameFromX, touchTrajectory])

  const handleContextMenu = useCallback((e: React.MouseEvent<HTMLCanvasElement>) => {
    e.preventDefault()
    const kf = keyframeAtX(e.clientX)
    if (kf !== null) {
      trajectory.removeKeyframe(kf)
      touchTrajectory()
    }
  }, [keyframeAtX, trajectory, touchTrajectory])

  useEffect(() => {
    const handleWindowMouseMove = (e: MouseEvent) => {
      const drag = dragRef.current
      if (!drag) return
      const store = useStore.getState()

      if (drag.type === 'scrub') {
        store.setCurrentFrame(frameFromX(e.clientX))
        return
      }
      if (drag.type === 'segment') {
        if (store.segmentStart !== null) store.setSegment(store.segmentStart, frameFromX(e.clientX))
        return
      }
      // keyframe drag: horizontal = retime anchor, vertical = edit channel value
      const newFrame = frameFromX(e.clientX)
      if (newFrame !== drag.curFrame) {
        trajectory.removeKeyframe(drag.curFrame)
        trajectory.insertKeyframe(newFrame)
        drag.curFrame = newFrame
        store.touchTrajectory()
      }
      const channel = store.selectedChannel
      if (channel && trajectory.hasKeyframe(drag.curFrame)) {
        const rect = canvasRef.current?.getBoundingClientRect()
        if (rect && rect.height > 0) {
          const { vMin, vMax } = scaleRef.current
          const dv = ((drag.startClientY - e.clientY) / rect.height) * (vMax - vMin)
          trajectory.setChannelValue(drag.curFrame, channel, drag.startValue + dv)
          store.touchTrajectory()
        }
      }
    }

    const handleWindowMouseUp = () => {
      const drag = dragRef.current
      dragRef.current = null
      if (drag?.type === 'segment') {
        const store = useStore.getState()
        if (store.segmentStart !== null && store.segmentEnd === null) store.setSegment(null, null)
      }
    }

    window.addEventListener('mousemove', handleWindowMouseMove)
    window.addEventListener('mouseup', handleWindowMouseUp)
    return () => {
      window.removeEventListener('mousemove', handleWindowMouseMove)
      window.removeEventListener('mouseup', handleWindowMouseUp)
    }
  }, [trajectory, frameFromX])

  const handleWheel = useCallback((e: React.WheelEvent<HTMLCanvasElement>) => {
    e.preventDefault()
    const delta = e.deltaY > 0 ? 1 : -1
    setCurrentFrame(Math.max(0, Math.min(currentFrame + delta, trajectory.frameCount - 1)))
  }, [currentFrame, trajectory.frameCount, setCurrentFrame])

  useEffect(() => {
    const canvas = canvasRef.current
    if (!canvas) return

    const draw = () => {
      const ctx = canvas.getContext('2d')
      if (!ctx) return
      canvas.width = canvas.clientWidth * devicePixelRatio
      canvas.height = canvas.clientHeight * devicePixelRatio
      ctx.scale(devicePixelRatio, devicePixelRatio)
      ctx.clearRect(0, 0, canvas.clientWidth, canvas.clientHeight)

      const cw = canvas.clientWidth
      const ch = canvas.clientHeight

      ctx.strokeStyle = '#444'
      ctx.lineWidth = 1
      const tickCount = Math.min(Math.max(trajectory.frameCount, 1), 50)
      for (let i = 0; i <= tickCount; i++) {
        const x = (i / tickCount) * cw
        ctx.beginPath()
        ctx.moveTo(x, 0)
        ctx.lineTo(x, ch * 0.3)
        ctx.stroke()
      }

      if (selectedChannel && trajectory.frameCount > 1) {
        let vMin = Infinity
        let vMax = -Infinity
        for (let f = 0; f < trajectory.frameCount; f++) {
          const v = trajectory.getChannelValue(f, selectedChannel)
          if (v < vMin) vMin = v
          if (v > vMax) vMax = v
        }
        if (vMin === vMax) { vMin -= 0.1; vMax += 0.1 }
        const margin = (vMax - vMin) * 0.1
        vMin -= margin
        vMax += margin
        scaleRef.current = { vMin, vMax }
        const vRange = vMax - vMin

        ctx.strokeStyle = '#4af'
        ctx.lineWidth = 2
        ctx.beginPath()
        for (let f = 0; f < trajectory.frameCount; f++) {
          const x = (f / (trajectory.frameCount - 1)) * cw
          const val = trajectory.getChannelValue(f, selectedChannel)
          const t = (val - vMin) / vRange
          const y = ch * 0.85 - t * ch * 0.7
          if (f === 0) ctx.moveTo(x, y)
          else ctx.lineTo(x, y)
        }
        ctx.stroke()
      }

      trajectory.keyframes.forEach(f => {
        if (trajectory.frameCount < 2) return
        const x = (f / (trajectory.frameCount - 1)) * cw
        ctx.fillStyle = '#fa4'
        ctx.beginPath()
        ctx.moveTo(x, ch * 0.5 - 6)
        ctx.lineTo(x - 5, ch * 0.5)
        ctx.lineTo(x, ch * 0.5 + 6)
        ctx.lineTo(x + 5, ch * 0.5)
        ctx.closePath()
        ctx.fill()
      })

      const px = (currentFrame / Math.max(trajectory.frameCount - 1, 1)) * cw
      ctx.strokeStyle = '#f44'
      ctx.lineWidth = 2
      ctx.beginPath()
      ctx.moveTo(px, 0)
      ctx.lineTo(px, ch)
      ctx.stroke()

      if (segmentStart !== null && segmentEnd !== null && trajectory.frameCount > 1) {
        const sx = (Math.min(segmentStart, segmentEnd) / (trajectory.frameCount - 1)) * cw
        const ex = (Math.max(segmentStart, segmentEnd) / (trajectory.frameCount - 1)) * cw
        ctx.fillStyle = 'rgba(100, 100, 255, 0.2)'
        ctx.fillRect(sx, 0, ex - sx, ch)
      }
    }

    draw()
    const interval = setInterval(draw, 100)
    return () => clearInterval(interval)
  }, [trajectory, trajectoryVersion, currentFrame, selectedChannel, segmentStart, segmentEnd])

  return (
    <canvas
      ref={canvasRef}
      style={canvasStyle}
      onMouseDown={handleMouseDown}
      onDoubleClick={handleDoubleClick}
      onContextMenu={handleContextMenu}
      onWheel={handleWheel}
    />
  )
}

const canvasStyle: React.CSSProperties = {
  width: '100%',
  height: '100%',
  background: '#1a1a2e',
  cursor: 'pointer',
}
