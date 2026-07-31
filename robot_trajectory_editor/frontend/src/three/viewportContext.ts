import type { Viewport } from './Viewport'

/**
 * Module-level handle to the live Viewport so non-three components
 * (e.g. StatePanel for joint limits) can read robot/viewport state
 * without prop drilling through App.
 */
let viewport: Viewport | null = null

export function setViewport(vp: Viewport | null): void {
  viewport = vp
}

export function getViewport(): Viewport | null {
  return viewport
}
