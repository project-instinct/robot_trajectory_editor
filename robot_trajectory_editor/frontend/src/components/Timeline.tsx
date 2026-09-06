import { useStore } from '../state/store'
import { useRef, useEffect, useCallback, useMemo, useState } from 'react'
import {
  frameAtClientX,
  frameToCanvasX,
  fullTimelineView,
  resizeTimelineView,
  resizeValueRange,
  zoomTimelineView,
  type TimelineView,
  type ValueRange,
} from './timelineView'

type NavigatorEdge = 'frame-start' | 'frame-end' | 'value-min' | 'value-max'

type DragState =
  | { type: 'scrub'; pointerId: number }
  | { type: 'segment'; pointerId: number }
  | { type: 'navigator'; pointerId: number; edge: NavigatorEdge }
  | {
      type: 'keyframe'
      pointerId: number
      startClientY: number
      startValue: number
      lastValue: number
      valuePerPixel: number
      curFrame: number
    }
  | null

interface PinchState {
  pointerIds: [number, number]
  startDistance: number
  startMidpointX: number
  startView: TimelineView
}

const KF_HIT_PX = 6
const RANGE_LABEL_HEIGHT = 16
const NAVIGATOR_SIDE_MARGIN = 8
const NAVIGATOR_HEIGHT = 22
const NAVIGATOR_BOTTOM_MARGIN = 4
const NAVIGATOR_GAP = 6
const NAVIGATOR_EDGE_HIT_PX = 8

interface CanvasArea {
  left: number
  top: number
  width: number
  height: number
}

function navigatorArea(canvasWidth: number, canvasHeight: number): CanvasArea {
  return {
    left: NAVIGATOR_SIDE_MARGIN,
    top: Math.max(RANGE_LABEL_HEIGHT, canvasHeight - NAVIGATOR_HEIGHT - NAVIGATOR_BOTTOM_MARGIN),
    width: Math.max(canvasWidth - NAVIGATOR_SIDE_MARGIN * 2, 1),
    height: NAVIGATOR_HEIGHT,
  }
}

function plotArea(canvasWidth: number, canvasHeight: number): CanvasArea {
  const navigator = navigatorArea(canvasWidth, canvasHeight)
  return {
    left: 0,
    top: RANGE_LABEL_HEIGHT,
    width: canvasWidth,
    height: Math.max(navigator.top - RANGE_LABEL_HEIGHT - NAVIGATOR_GAP, 1),
  }
}

function valueToY(value: number, area: CanvasArea, range: ValueRange): number {
  const span = range.maxValue - range.minValue
  if (span <= 0) return area.top + area.height / 2
  return area.top + ((range.maxValue - value) / span) * area.height
}

function formatValue(value: number): string {
  const magnitude = Math.abs(value)
  return magnitude >= 1000 || (magnitude > 0 && magnitude < 0.001)
    ? value.toExponential(2)
    : value.toFixed(3)
}

/**
 * Timeline interactions:
 * - click / drag: scrub playhead
 * - Shift+drag: select segment (used by Fill/Smooth)
 * - double-click: add keyframe anchor at frame
 * - drag keyframe horizontally: retime anchor; vertically: edit selected channel value
 * - right-click keyframe: remove it
 * - Alt+wheel or a two-finger pinch: zoom around the gesture position
 * - drag the global navigator edges: adjust visible frame and value ranges
 */
export function Timeline() {
  const {
    trajectory, trajectoryVersion, currentFrame, setCurrentFrame,
    selectedChannel, segmentStart, segmentEnd, setSegment, beginTrajectoryEdit, touchTrajectory,
  } = useStore()
  const canvasRef = useRef<HTMLCanvasElement>(null)
  const dragRef = useRef<DragState>(null)
  const touchPointsRef = useRef(new Map<number, { x: number; y: number }>())
  const pinchRef = useRef<PinchState | null>(null)
  const [view, setView] = useState<TimelineView>(() => fullTimelineView(trajectory.frameCount))
  const viewRef = useRef(view)
  const [valueView, setValueView] = useState<ValueRange | null>(null)

  const globalValueRange = useMemo<ValueRange>(() => {
    if (!selectedChannel || trajectory.frameCount < 1) return { minValue: 0, maxValue: 1 }
    let minValue = Infinity
    let maxValue = -Infinity
    for (let frame = 0; frame < trajectory.frameCount; frame++) {
      const value = trajectory.getChannelValue(frame, selectedChannel)
      if (value < minValue) minValue = value
      if (value > maxValue) maxValue = value
    }
    if (minValue === maxValue) {
      const padding = Math.max(Math.abs(minValue) * 0.1, 0.1)
      return { minValue: minValue - padding, maxValue: maxValue + padding }
    }
    const padding = (maxValue - minValue) * 0.1
    return { minValue: minValue - padding, maxValue: maxValue + padding }
    // Trajectory samples mutate in place; the version is their cache invalidator.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [trajectory, trajectoryVersion, selectedChannel])

  const currentValueView = valueView ?? globalValueRange
  const valueViewRef = useRef(currentValueView)
  valueViewRef.current = currentValueView

  const updateView = useCallback((nextView: TimelineView) => {
    viewRef.current = nextView
    setView(nextView)
  }, [])

  useEffect(() => {
    updateView(fullTimelineView(trajectory.frameCount))
  }, [trajectory, trajectory.frameCount, updateView])

  useEffect(() => {
    setValueView(null)
  }, [trajectory, selectedChannel])

  const frameFromX = useCallback((clientX: number): number => {
    const rect = canvasRef.current?.getBoundingClientRect()
    if (!rect || trajectory.frameCount < 1) return 0
    return frameAtClientX(clientX, rect.left, rect.width, viewRef.current, trajectory.frameCount)
  }, [trajectory.frameCount])

  const keyframeAtX = useCallback((clientX: number): number | null => {
    const rect = canvasRef.current?.getBoundingClientRect()
    if (!rect || trajectory.frameCount < 2) return null
    let best: number | null = null
    let bestDist = KF_HIT_PX + 1
    for (const f of trajectory.keyframes) {
      if (f < viewRef.current.startFrame || f > viewRef.current.endFrame) continue
      const x = frameToCanvasX(f, rect.width, viewRef.current)
      const d = Math.abs(clientX - rect.left - x)
      if (d < bestDist) { bestDist = d; best = f }
    }
    return best
  }, [trajectory])

  const navigatorEdgeAt = useCallback((
    clientX: number,
    clientY: number,
    tolerance = NAVIGATOR_EDGE_HIT_PX,
  ): NavigatorEdge | null => {
    const rect = canvasRef.current?.getBoundingClientRect()
    if (!rect || trajectory.frameCount < 1) return null
    const area = navigatorArea(rect.width, rect.height)
    const x = clientX - rect.left
    const y = clientY - rect.top
    if (
      x < area.left - tolerance || x > area.left + area.width + tolerance
      || y < area.top - tolerance || y > area.top + area.height + tolerance
    ) return null

    const fullView = fullTimelineView(trajectory.frameCount)
    const startX = area.left + frameToCanvasX(viewRef.current.startFrame, area.width, fullView)
    const endX = area.left + frameToCanvasX(viewRef.current.endFrame, area.width, fullView)
    const maxY = valueToY(valueViewRef.current.maxValue, area, globalValueRange)
    const minY = valueToY(valueViewRef.current.minValue, area, globalValueRange)
    const candidates: Array<{ edge: NavigatorEdge; distance: number }> = []

    if (y >= maxY - tolerance && y <= minY + tolerance) {
      if (Math.abs(x - startX) <= tolerance) {
        candidates.push({ edge: 'frame-start', distance: Math.abs(x - startX) })
      }
      if (Math.abs(x - endX) <= tolerance) {
        candidates.push({ edge: 'frame-end', distance: Math.abs(x - endX) })
      }
    }
    if (selectedChannel && x >= startX - tolerance && x <= endX + tolerance) {
      if (Math.abs(y - maxY) <= tolerance) {
        candidates.push({ edge: 'value-max', distance: Math.abs(y - maxY) })
      }
      if (Math.abs(y - minY) <= tolerance) {
        candidates.push({ edge: 'value-min', distance: Math.abs(y - minY) })
      }
    }

    candidates.sort((a, b) => a.distance - b.distance)
    return candidates[0]?.edge ?? null
  }, [trajectory.frameCount, globalValueRange, selectedChannel])

  const pointIsInNavigator = useCallback((clientX: number, clientY: number): boolean => {
    const rect = canvasRef.current?.getBoundingClientRect()
    if (!rect) return false
    const area = navigatorArea(rect.width, rect.height)
    const x = clientX - rect.left
    const y = clientY - rect.top
    return x >= area.left && x <= area.left + area.width
      && y >= area.top && y <= area.top + area.height
  }, [])

  const handlePointerDown = useCallback((e: React.PointerEvent<HTMLCanvasElement>) => {
    if (e.button !== 0 || trajectory.frameCount < 1) return
    e.preventDefault()
    e.currentTarget.setPointerCapture(e.pointerId)

    if (e.pointerType === 'touch') {
      touchPointsRef.current.set(e.pointerId, { x: e.clientX, y: e.clientY })
      if (touchPointsRef.current.size >= 2) {
        const [[firstId, first], [secondId, second]] = Array.from(touchPointsRef.current.entries())
        pinchRef.current = {
          pointerIds: [firstId, secondId],
          startDistance: Math.hypot(second.x - first.x, second.y - first.y),
          startMidpointX: (first.x + second.x) / 2,
          startView: viewRef.current,
        }
        dragRef.current = null
        return
      }
    }

    const navigatorEdge = navigatorEdgeAt(
      e.clientX,
      e.clientY,
      e.pointerType === 'mouse' ? NAVIGATOR_EDGE_HIT_PX : NAVIGATOR_EDGE_HIT_PX + 4,
    )
    if (navigatorEdge) {
      dragRef.current = { type: 'navigator', pointerId: e.pointerId, edge: navigatorEdge }
      return
    }
    if (pointIsInNavigator(e.clientX, e.clientY)) return

    const kf = keyframeAtX(e.clientX)
    if (kf !== null && !e.shiftKey) {
      beginTrajectoryEdit()
      const startValue = selectedChannel ? trajectory.getChannelValue(kf, selectedChannel) : 0
      const rect = canvasRef.current?.getBoundingClientRect()
      const plotHeight = rect ? plotArea(rect.width, rect.height).height : 0
      const valuePerPixel = plotHeight > 0
        ? (valueViewRef.current.maxValue - valueViewRef.current.minValue) / plotHeight
        : 0
      dragRef.current = {
        type: 'keyframe',
        pointerId: e.pointerId,
        startClientY: e.clientY,
        startValue,
        lastValue: startValue,
        valuePerPixel,
        curFrame: kf,
      }
      setCurrentFrame(kf)
      return
    }
    if (e.shiftKey) {
      dragRef.current = { type: 'segment', pointerId: e.pointerId }
      setSegment(frameFromX(e.clientX), null)
      return
    }
    dragRef.current = { type: 'scrub', pointerId: e.pointerId }
    setCurrentFrame(frameFromX(e.clientX))
  }, [
    trajectory, selectedChannel, keyframeAtX, frameFromX, setCurrentFrame,
    setSegment, beginTrajectoryEdit, navigatorEdgeAt, pointIsInNavigator,
  ])

  const handleDoubleClick = useCallback((e: React.MouseEvent<HTMLCanvasElement>) => {
    if (trajectory.frameCount < 1 || pointIsInNavigator(e.clientX, e.clientY)) return
    beginTrajectoryEdit()
    trajectory.insertKeyframe(frameFromX(e.clientX))
    touchTrajectory()
  }, [trajectory, frameFromX, beginTrajectoryEdit, touchTrajectory, pointIsInNavigator])

  const handleContextMenu = useCallback((e: React.MouseEvent<HTMLCanvasElement>) => {
    e.preventDefault()
    if (pointIsInNavigator(e.clientX, e.clientY)) return
    const kf = keyframeAtX(e.clientX)
    if (kf !== null) {
      beginTrajectoryEdit()
      trajectory.removeKeyframe(kf)
      touchTrajectory()
    }
  }, [keyframeAtX, trajectory, beginTrajectoryEdit, touchTrajectory, pointIsInNavigator])

  const handlePointerMove = useCallback((e: React.PointerEvent<HTMLCanvasElement>) => {
    if (e.pointerType === 'touch' && touchPointsRef.current.has(e.pointerId)) {
      touchPointsRef.current.set(e.pointerId, { x: e.clientX, y: e.clientY })
    }

    const pinch = pinchRef.current
    if (pinch) {
      const first = touchPointsRef.current.get(pinch.pointerIds[0])
      const second = touchPointsRef.current.get(pinch.pointerIds[1])
      const rect = canvasRef.current?.getBoundingClientRect()
      if (first && second && rect && rect.width > 0) {
        const distance = Math.hypot(second.x - first.x, second.y - first.y)
        if (distance > 0 && pinch.startDistance > 0) {
          const midpointX = (first.x + second.x) / 2
          updateView(zoomTimelineView(
            pinch.startView,
            trajectory.frameCount,
            pinch.startDistance / distance,
            (pinch.startMidpointX - rect.left) / rect.width,
            (midpointX - rect.left) / rect.width,
          ))
        }
      }
      return
    }

    const drag = dragRef.current
    if (!drag) {
      const edge = navigatorEdgeAt(e.clientX, e.clientY)
      e.currentTarget.style.cursor = edge === 'frame-start' || edge === 'frame-end'
        ? 'ew-resize'
        : edge === 'value-min' || edge === 'value-max' ? 'ns-resize' : 'pointer'
      return
    }
    if (drag.pointerId !== e.pointerId) return
    const store = useStore.getState()

    if (drag.type === 'navigator') {
      const rect = canvasRef.current?.getBoundingClientRect()
      if (!rect) return
      const area = navigatorArea(rect.width, rect.height)
      if (drag.edge === 'frame-start' || drag.edge === 'frame-end') {
        const frame = frameAtClientX(
          e.clientX,
          rect.left + area.left,
          area.width,
          fullTimelineView(trajectory.frameCount),
          trajectory.frameCount,
        )
        updateView(resizeTimelineView(
          viewRef.current,
          trajectory.frameCount,
          drag.edge === 'frame-start' ? 'start' : 'end',
          frame,
        ))
      } else {
        const ratio = Math.max(0, Math.min((e.clientY - rect.top - area.top) / area.height, 1))
        const value = globalValueRange.maxValue
          - ratio * (globalValueRange.maxValue - globalValueRange.minValue)
        const nextRange = resizeValueRange(
          valueViewRef.current,
          globalValueRange,
          drag.edge === 'value-min' ? 'min' : 'max',
          value,
        )
        valueViewRef.current = nextRange
        setValueView(nextRange)
      }
      return
    }

    if (drag.type === 'scrub') {
      store.setCurrentFrame(frameFromX(e.clientX))
      return
    }
    if (drag.type === 'segment') {
      if (store.segmentStart !== null) store.setSegment(store.segmentStart, frameFromX(e.clientX))
      return
    }

    // Treat the keyframe as a drawing pen: X controls time, Y controls the
    // selected channel value, and interpolate across skipped pointer frames.
    const previousFrame = drag.curFrame
    const newFrame = frameFromX(e.clientX)
    const channel = store.selectedChannel
    let changed = false
    if (channel) {
      const newValue = drag.startValue
        + (drag.startClientY - e.clientY) * drag.valuePerPixel
      if (newFrame === previousFrame) {
        trajectory.setChannelValue(newFrame, channel, newValue)
      } else {
        trajectory.setChannelValue(previousFrame, channel, drag.lastValue)
        trajectory.setChannelValue(newFrame, channel, newValue)
        trajectory.interpolateChannelRange(previousFrame, newFrame, channel)
      }
      drag.lastValue = newValue
      changed = true
    }

    if (newFrame !== previousFrame) {
      trajectory.removeKeyframe(previousFrame)
      trajectory.insertKeyframe(newFrame)
      drag.curFrame = newFrame
      store.setCurrentFrame(newFrame)
      changed = true
    }
    if (changed) store.touchTrajectory()
  }, [trajectory, frameFromX, updateView, globalValueRange, navigatorEdgeAt])

  const finishPointer = useCallback((e: React.PointerEvent<HTMLCanvasElement>) => {
    e.currentTarget.style.cursor = 'pointer'
    if (e.pointerType === 'touch') touchPointsRef.current.delete(e.pointerId)
    if (pinchRef.current) {
      if (touchPointsRef.current.size < 2) pinchRef.current = null
      dragRef.current = null
      return
    }

    const drag = dragRef.current
    if (!drag || drag.pointerId !== e.pointerId) return
    dragRef.current = null
    if (drag.type === 'segment') {
      const store = useStore.getState()
      if (store.segmentStart !== null && store.segmentEnd === null) store.setSegment(null, null)
    }
  }, [])

  const handleWheel = useCallback((e: React.WheelEvent<HTMLCanvasElement>) => {
    e.preventDefault()
    if (e.altKey) {
      const rect = canvasRef.current?.getBoundingClientRect()
      if (!rect || rect.width <= 0) return
      const deltaScale = e.deltaMode === WheelEvent.DOM_DELTA_LINE
        ? 16
        : e.deltaMode === WheelEvent.DOM_DELTA_PAGE ? 100 : 1
      const spanFactor = Math.exp(Math.max(-500, Math.min(e.deltaY * deltaScale, 500)) * 0.002)
      updateView(zoomTimelineView(
        viewRef.current,
        trajectory.frameCount,
        spanFactor,
        (e.clientX - rect.left) / rect.width,
      ))
      return
    }
    if (trajectory.frameCount < 1) return
    const delta = e.deltaY > 0 ? 1 : -1
    setCurrentFrame(Math.max(0, Math.min(currentFrame + delta, trajectory.frameCount - 1)))
  }, [currentFrame, trajectory.frameCount, setCurrentFrame, updateView])

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
      const plot = plotArea(cw, ch)
      const navigator = navigatorArea(cw, ch)
      const visibleStart = view.startFrame
      const visibleEnd = view.endFrame
      const visibleValueRange = currentValueView

      ctx.strokeStyle = '#444'
      ctx.lineWidth = 1
      const tickCount = Math.min(Math.max(Math.ceil(visibleEnd - visibleStart), 1), 50)
      for (let i = 0; i <= tickCount; i++) {
        const x = (i / tickCount) * cw
        ctx.beginPath()
        ctx.moveTo(x, plot.top)
        ctx.lineTo(x, plot.top + plot.height)
        ctx.stroke()
      }

      if (selectedChannel && trajectory.frameCount > 1) {
        ctx.save()
        ctx.beginPath()
        ctx.rect(plot.left, plot.top, plot.width, plot.height)
        ctx.clip()
        ctx.strokeStyle = '#4af'
        ctx.lineWidth = 2
        ctx.beginPath()
        const firstVisibleFrame = Math.max(0, Math.floor(visibleStart))
        const lastVisibleFrame = Math.min(trajectory.frameCount - 1, Math.ceil(visibleEnd))
        for (let f = firstVisibleFrame; f <= lastVisibleFrame; f++) {
          const x = frameToCanvasX(f, cw, view)
          const y = valueToY(trajectory.getChannelValue(f, selectedChannel), plot, visibleValueRange)
          if (f === firstVisibleFrame) ctx.moveTo(x, y)
          else ctx.lineTo(x, y)
        }
        ctx.stroke()
        ctx.restore()
      }

      trajectory.keyframes.forEach(f => {
        if (trajectory.frameCount < 2) return
        if (f < visibleStart || f > visibleEnd) return
        const x = frameToCanvasX(f, cw, view)
        const y = plot.top + plot.height / 2
        ctx.fillStyle = '#fa4'
        ctx.beginPath()
        ctx.moveTo(x, y - 6)
        ctx.lineTo(x - 5, y)
        ctx.lineTo(x, y + 6)
        ctx.lineTo(x + 5, y)
        ctx.closePath()
        ctx.fill()
      })

      if (currentFrame >= visibleStart && currentFrame <= visibleEnd) {
        const px = frameToCanvasX(currentFrame, cw, view)
        ctx.strokeStyle = '#f44'
        ctx.lineWidth = 2
        ctx.beginPath()
        ctx.moveTo(px, plot.top)
        ctx.lineTo(px, plot.top + plot.height)
        ctx.stroke()
      }

      if (segmentStart !== null && segmentEnd !== null && trajectory.frameCount > 1) {
        const sx = frameToCanvasX(Math.min(segmentStart, segmentEnd), cw, view)
        const ex = frameToCanvasX(Math.max(segmentStart, segmentEnd), cw, view)
        ctx.fillStyle = 'rgba(100, 100, 255, 0.2)'
        ctx.fillRect(sx, plot.top, ex - sx, plot.height)
      }

      // Always show the exact visible ranges above the detailed plot.
      ctx.fillStyle = '#bbb'
      ctx.font = '11px monospace'
      ctx.textAlign = 'left'
      ctx.fillText(
        `frames ${Math.ceil(visibleStart)}-${Math.floor(visibleEnd)}`,
        6,
        12,
      )
      ctx.textAlign = 'right'
      ctx.fillText(
        selectedChannel
          ? `values ${formatValue(visibleValueRange.minValue)}-${formatValue(visibleValueRange.maxValue)}`
          : 'values n/a',
        cw - 6,
        12,
      )

      // Global navigator: its selection rectangle is the detailed plot's
      // frame/value window, and each of its four edges is draggable.
      ctx.fillStyle = '#111827'
      ctx.fillRect(navigator.left, navigator.top, navigator.width, navigator.height)
      ctx.strokeStyle = '#596075'
      ctx.lineWidth = 1
      ctx.strokeRect(navigator.left, navigator.top, navigator.width, navigator.height)

      const fullView = fullTimelineView(trajectory.frameCount)
      if (selectedChannel && trajectory.frameCount > 1) {
        ctx.save()
        ctx.beginPath()
        ctx.rect(navigator.left, navigator.top, navigator.width, navigator.height)
        ctx.clip()
        ctx.strokeStyle = '#567fa5'
        ctx.lineWidth = 1
        ctx.beginPath()
        for (let frame = 0; frame < trajectory.frameCount; frame++) {
          const x = navigator.left + frameToCanvasX(frame, navigator.width, fullView)
          const y = valueToY(trajectory.getChannelValue(frame, selectedChannel), navigator, globalValueRange)
          if (frame === 0) ctx.moveTo(x, y)
          else ctx.lineTo(x, y)
        }
        ctx.stroke()
        ctx.restore()
      }

      const selectionStartX = trajectory.frameCount > 1
        ? navigator.left + frameToCanvasX(view.startFrame, navigator.width, fullView)
        : navigator.left
      const selectionEndX = trajectory.frameCount > 1
        ? navigator.left + frameToCanvasX(view.endFrame, navigator.width, fullView)
        : navigator.left + navigator.width
      const selectionTop = selectedChannel
        ? valueToY(visibleValueRange.maxValue, navigator, globalValueRange)
        : navigator.top
      const selectionBottom = selectedChannel
        ? valueToY(visibleValueRange.minValue, navigator, globalValueRange)
        : navigator.top + navigator.height

      ctx.fillStyle = 'rgba(255, 170, 68, 0.12)'
      ctx.fillRect(
        selectionStartX,
        selectionTop,
        selectionEndX - selectionStartX,
        selectionBottom - selectionTop,
      )
      ctx.strokeStyle = '#fa4'
      ctx.lineWidth = 2
      ctx.strokeRect(
        selectionStartX,
        selectionTop,
        selectionEndX - selectionStartX,
        selectionBottom - selectionTop,
      )

      const selectionMiddleX = (selectionStartX + selectionEndX) / 2
      const selectionMiddleY = (selectionTop + selectionBottom) / 2
      ctx.fillStyle = '#ffc06a'
      ctx.fillRect(selectionStartX - 2, selectionMiddleY - 4, 4, 8)
      ctx.fillRect(selectionEndX - 2, selectionMiddleY - 4, 4, 8)
      if (selectedChannel) {
        ctx.fillRect(selectionMiddleX - 4, selectionTop - 2, 8, 4)
        ctx.fillRect(selectionMiddleX - 4, selectionBottom - 2, 8, 4)
      }

      if (trajectory.frameCount > 1) {
        const globalPlayheadX = navigator.left
          + (currentFrame / (trajectory.frameCount - 1)) * navigator.width
        ctx.strokeStyle = '#f44'
        ctx.lineWidth = 1
        ctx.beginPath()
        ctx.moveTo(globalPlayheadX, navigator.top)
        ctx.lineTo(globalPlayheadX, navigator.top + navigator.height)
        ctx.stroke()
      }
    }

    draw()
    const interval = setInterval(draw, 100)
    return () => clearInterval(interval)
  }, [
    trajectory, trajectoryVersion, currentFrame, selectedChannel, segmentStart,
    segmentEnd, view, currentValueView, globalValueRange,
  ])

  return (
    <canvas
      ref={canvasRef}
      style={canvasStyle}
      onPointerDown={handlePointerDown}
      onPointerMove={handlePointerMove}
      onPointerUp={finishPointer}
      onPointerCancel={finishPointer}
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
  touchAction: 'none',
  userSelect: 'none',
}
