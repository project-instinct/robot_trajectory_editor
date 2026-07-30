import { useStore } from '../state/store'
import { useRef, useEffect, useCallback } from 'react'

export function Timeline() {
  const { trajectory, currentFrame, setCurrentFrame, selectedChannel, segmentStart, segmentEnd, setSegment } = useStore()
  const canvasRef = useRef<HTMLCanvasElement>(null)
  const draggingRef = useRef(false)

  const frameFromEvent = useCallback((e: MouseEvent | React.MouseEvent): number => {
    const rect = canvasRef.current?.getBoundingClientRect()
    if (!rect) return 0
    return Math.max(0, Math.min(
      Math.round(((e.clientX - rect.left) / rect.width) * (trajectory.frameCount - 1)),
      trajectory.frameCount - 1
    ))
  }, [trajectory.frameCount])

  const handleCanvasClick = useCallback((e: React.MouseEvent<HTMLCanvasElement>) => {
    if (trajectory.frameCount < 1) return
    setCurrentFrame(frameFromEvent(e))
  }, [trajectory.frameCount, setCurrentFrame, frameFromEvent])

  const handleMouseDown = useCallback((e: React.MouseEvent<HTMLCanvasElement>) => {
    if (!e.shiftKey || trajectory.frameCount < 1) return
    draggingRef.current = true
    setSegment(frameFromEvent(e), null)
  }, [trajectory.frameCount, setSegment, frameFromEvent])

  useEffect(() => {
    const handleWindowMouseMove = (e: MouseEvent) => {
      if (!draggingRef.current) return
      const rect = canvasRef.current?.getBoundingClientRect()
      if (!rect) return
      const frame = Math.max(0, Math.min(
        Math.round(((e.clientX - rect.left) / rect.width) * (trajectory.frameCount - 1)),
        trajectory.frameCount - 1
      ))
      const store = useStore.getState()
      if (store.segmentStart !== null) {
        store.setSegment(store.segmentStart, frame)
      }
    }

    const handleWindowMouseUp = () => {
      if (!draggingRef.current) return
      draggingRef.current = false
      const store = useStore.getState()
      if (store.segmentStart !== null && store.segmentEnd === null) {
        store.setSegment(null, null)
      }
    }

    window.addEventListener('mousemove', handleWindowMouseMove)
    window.addEventListener('mouseup', handleWindowMouseUp)
    return () => {
      window.removeEventListener('mousemove', handleWindowMouseMove)
      window.removeEventListener('mouseup', handleWindowMouseUp)
    }
  }, [trajectory.frameCount])

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
      const tickCount = Math.min(trajectory.frameCount, 50)
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
        const x = (f / (trajectory.frameCount - 1)) * cw
        ctx.fillStyle = '#fa4'
        ctx.beginPath()
        ctx.moveTo(x, ch * 0.5)
        ctx.lineTo(x - 4, ch * 0.5 - 8)
        ctx.lineTo(x + 4, ch * 0.5 - 8)
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

      if (segmentStart !== null && segmentEnd !== null) {
        const sx = (Math.min(segmentStart, segmentEnd) / (trajectory.frameCount - 1)) * cw
        const ex = (Math.max(segmentStart, segmentEnd) / (trajectory.frameCount - 1)) * cw
        ctx.fillStyle = 'rgba(100, 100, 255, 0.2)'
        ctx.fillRect(sx, 0, ex - sx, ch)
      }
    }

    draw()
    const interval = setInterval(draw, 100)
    return () => clearInterval(interval)
  }, [trajectory, currentFrame, selectedChannel, segmentStart, segmentEnd])

  return (
    <canvas
      ref={canvasRef}
      style={canvasStyle}
      onClick={handleCanvasClick}
      onWheel={handleWheel}
      onMouseDown={handleMouseDown}
    />
  )
}

const canvasStyle: React.CSSProperties = {
  width: '100%',
  height: '100%',
  background: '#1a1a2e',
  cursor: 'pointer',
}
