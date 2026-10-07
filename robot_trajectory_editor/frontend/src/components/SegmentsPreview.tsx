import { useEffect, useRef } from 'react'
import * as THREE from 'three'
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js'
import { useStore } from '../state/store'
import { getViewport } from '../three/viewportContext'
import { fullTimelineView, frameAtClientX, frameToCanvasX } from './timelineView'

export interface PreviewSegment {
  start: number | null
  end: number | null
}

interface SegmentsPreviewProps {
  segments: PreviewSegment[]
}

/**
 * Read-only preview used by the Segments modal: a 3D view that re-renders the
 * main viewport's scene through its own camera (so it can be orbited without
 * editing the robot) plus a scrub-only timeline for selecting segment frames.
 */
export function SegmentsPreview({ segments }: SegmentsPreviewProps) {
  const viewportRef = useRef<HTMLDivElement>(null)
  const timelineRef = useRef<HTMLCanvasElement>(null)
  const { trajectory, currentFrame, setCurrentFrame } = useStore()

  // Render the existing scene with a separate camera/renderer. Sharing the
  // scene keeps the live robot, terrain, and lights without duplicating the
  // URDF or moving objects out of the main viewport.
  useEffect(() => {
    const container = viewportRef.current
    if (!container) return
    const source = getViewport()
    if (!source) return

    const scene = source.scene
    const camera = new THREE.PerspectiveCamera(50, 2, 0.01, 100)
    camera.up.set(0, 0, 1)

    const robot = source.robotModel
    const target = robot
      ? robot.rootGroup.position.clone()
      : new THREE.Vector3(0, 0, 0)
    camera.position.set(target.x + 3, target.y - 3, target.z + 3)

    const renderer = new THREE.WebGLRenderer({ antialias: true })
    renderer.setPixelRatio(window.devicePixelRatio)
    renderer.domElement.style.display = 'block'
    renderer.domElement.style.width = '100%'
    renderer.domElement.style.height = '100%'
    container.appendChild(renderer.domElement)

    const controls = new OrbitControls(camera, renderer.domElement)
    controls.target.copy(target)
    controls.enableDamping = true
    controls.dampingFactor = 0.1
    controls.update()

    const resize = () => {
      const w = container.clientWidth
      const h = container.clientHeight
      if (!w || !h) return
      camera.aspect = w / h
      camera.updateProjectionMatrix()
      renderer.setSize(w, h)
    }
    resize()
    const observer = new ResizeObserver(resize)
    observer.observe(container)

    let running = true
    const loop = () => {
      if (!running) return
      requestAnimationFrame(loop)
      controls.update()
      renderer.render(scene, camera)
    }
    loop()

    return () => {
      running = false
      observer.disconnect()
      controls.dispose()
      renderer.dispose()
      renderer.domElement.remove()
    }
  }, [])

  const scrubTo = (clientX: number) => {
    const canvas = timelineRef.current
    if (!canvas || trajectory.frameCount < 1) return
    const rect = canvas.getBoundingClientRect()
    const frame = frameAtClientX(
      clientX,
      rect.left,
      rect.width,
      fullTimelineView(trajectory.frameCount),
      trajectory.frameCount,
    )
    setCurrentFrame(frame)
  }

  const handleTimelinePointerDown = (e: React.PointerEvent<HTMLCanvasElement>) => {
    e.currentTarget.setPointerCapture(e.pointerId)
    scrubTo(e.clientX)
  }

  const handleTimelinePointerMove = (e: React.PointerEvent<HTMLCanvasElement>) => {
    if (e.buttons !== 0) scrubTo(e.clientX)
  }

  useEffect(() => {
    const canvas = timelineRef.current
    if (!canvas) return

    const draw = () => {
      const ctx = canvas.getContext('2d')
      if (!ctx) return
      const dpr = window.devicePixelRatio
      canvas.width = canvas.clientWidth * dpr
      canvas.height = canvas.clientHeight * dpr
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0)

      const w = canvas.clientWidth
      const h = canvas.clientHeight
      ctx.clearRect(0, 0, w, h)
      ctx.fillStyle = '#111827'
      ctx.fillRect(0, 0, w, h)

      const view = fullTimelineView(trajectory.frameCount)
      for (const segment of segments) {
        if (segment.start === null || segment.end === null) continue
        const x1 = frameToCanvasX(Math.min(segment.start, segment.end), w, view)
        const x2 = frameToCanvasX(Math.max(segment.start, segment.end), w, view)
        ctx.fillStyle = 'rgba(100, 100, 255, 0.35)'
        ctx.fillRect(x1, 2, x2 - x1, h - 4)
      }

      ctx.strokeStyle = '#555'
      ctx.lineWidth = 1
      const tickCount = Math.min(Math.max(trajectory.frameCount - 1, 1), 50)
      for (let i = 0; i <= tickCount; i++) {
        const x = (i / tickCount) * w
        ctx.beginPath()
        ctx.moveTo(x, 0)
        ctx.lineTo(x, h)
        ctx.stroke()
      }

      if (trajectory.frameCount > 1) {
        const px = frameToCanvasX(currentFrame, w, view)
        ctx.strokeStyle = '#f44'
        ctx.lineWidth = 2
        ctx.beginPath()
        ctx.moveTo(px, 0)
        ctx.lineTo(px, h)
        ctx.stroke()
      }

      ctx.fillStyle = '#ccc'
      ctx.font = '11px monospace'
      ctx.textAlign = 'left'
      ctx.fillText(`frame ${currentFrame}`, 4, 14)
      ctx.textAlign = 'right'
      ctx.fillText(`0 - ${trajectory.frameCount - 1}`, w - 4, 14)
    }

    draw()
    const interval = setInterval(draw, 100)
    return () => clearInterval(interval)
  }, [trajectory, currentFrame, segments])

  return (
    <div style={previewStyle}>
      <div ref={viewportRef} style={viewportStyle} />
      <canvas
        ref={timelineRef}
        style={timelineStyle}
        onPointerDown={handleTimelinePointerDown}
        onPointerMove={handleTimelinePointerMove}
      />
    </div>
  )
}

const previewStyle: React.CSSProperties = {
  display: 'flex',
  flexDirection: 'column',
  gap: '6px',
  marginBottom: '8px',
}

const viewportStyle: React.CSSProperties = {
  position: 'relative',
  height: '240px',
  background: '#2a2a4e',
  borderRadius: '4px',
  overflow: 'hidden',
  touchAction: 'none',
}

const timelineStyle: React.CSSProperties = {
  width: '100%',
  height: '48px',
  background: '#111827',
  borderRadius: '4px',
  cursor: 'crosshair',
  touchAction: 'none',
  userSelect: 'none',
}
