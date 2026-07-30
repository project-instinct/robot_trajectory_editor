import { useStore } from '../state/store'
import { useRef, useEffect, useCallback } from 'react'

export function Timeline() {
  const { trajectory, currentFrame, setCurrentFrame, selectedChannel, segmentStart, segmentEnd, setSegment } = useStore()
  const canvasRef = useRef<HTMLCanvasElement>(null)

  const handleCanvasClick = useCallback((e: React.MouseEvent<HTMLCanvasElement>) => {
    const rect = canvasRef.current?.getBoundingClientRect()
    if (!rect || trajectory.frameCount < 1) return
    const x = e.clientX - rect.left
    const frame = Math.round((x / rect.width) * (trajectory.frameCount - 1))
    setCurrentFrame(Math.max(0, Math.min(frame, trajectory.frameCount - 1)))
  }, [trajectory.frameCount, setCurrentFrame])

  const handleMouseDown = useCallback((e: React.MouseEvent<HTMLCanvasElement>) => {
    if (!e.shiftKey || trajectory.frameCount < 1) return
    const rect = canvasRef.current?.getBoundingClientRect()
    if (!rect) return
    const frame = Math.round(((e.clientX - rect.left) / rect.width) * (trajectory.frameCount - 1))
    setSegment(frame, null)
  }, [trajectory.frameCount, setSegment])

  const handleMouseMove = useCallback((e: React.MouseEvent<HTMLCanvasElement>) => {
    if (segmentStart === null) return
    const rect = canvasRef.current?.getBoundingClientRect()
    if (!rect) return
    const frame = Math.round(((e.clientX - rect.left) / rect.width) * (trajectory.frameCount - 1))
    setSegment(segmentStart, frame)
  }, [segmentStart, setSegment, trajectory.frameCount])

  const handleMouseUp = useCallback(() => {
    if (segmentStart !== null && segmentEnd === null) setSegment(null, null)
  }, [segmentStart, segmentEnd, setSegment])

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
        ctx.strokeStyle = '#4af'
        ctx.lineWidth = 2
        ctx.beginPath()
        for (let f = 0; f < trajectory.frameCount; f++) {
          const x = (f / (trajectory.frameCount - 1)) * cw
          let val = trajectory.getChannelValue(f, selectedChannel)
          const y = ch * 0.5 - (val / (Math.PI * 2)) * ch * 0.3 + ch * 0.15
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
        const sx = (segmentStart / (trajectory.frameCount - 1)) * cw
        const ex = (segmentEnd / (trajectory.frameCount - 1)) * cw
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
      onMouseMove={handleMouseMove}
      onMouseUp={handleMouseUp}
    />
  )
}

const canvasStyle: React.CSSProperties = {
  width: '100%',
  height: '100%',
  background: '#1a1a2e',
  cursor: 'pointer',
}
