import type { Vec2 } from '../scene/boat'
import { SCALE_PX_PER_M } from '../units'

// View transform on top of the base world→canvas mapping. `zoom` multiplies the
// nominal pixels-per-metre; `panX/panY` shift the view in screen pixels.
export interface Camera {
  panX: number
  panY: number
  zoom: number
}

export function makeCamera(): Camera {
  return { panX: 0, panY: 0, zoom: 1 }
}

// World metres → screen pixels. The scene origin sits at (originX, originY) and
// the world y-axis points up, so screen y is flipped.
export function worldToScreen(
  cam: Camera, originX: number, originY: number, wx: number, wy: number,
): [number, number] {
  const s = SCALE_PX_PER_M * cam.zoom
  return [originX + cam.panX + wx * s, originY + cam.panY - wy * s]
}

// Inverse of worldToScreen.
export function screenToWorld(
  cam: Camera, originX: number, originY: number, sx: number, sy: number,
): Vec2 {
  const s = SCALE_PX_PER_M * cam.zoom
  return {
    x: (sx - originX - cam.panX) / s,
    y: (originY + cam.panY - sy) / s,
  }
}

export interface Bounds {
  minX: number; minY: number; maxX: number; maxY: number
}

// Fit the given world bounds into a screen viewport with a margin, returning the
// camera (zoom + pan) that centres them.
export function fitCamera(
  bounds: Bounds, screenW: number, screenH: number, originX: number, originY: number,
  margin = 0.1,
): Camera {
  const worldW = Math.max(1e-6, bounds.maxX - bounds.minX)
  const worldH = Math.max(1e-6, bounds.maxY - bounds.minY)
  const usableW = screenW * (1 - 2 * margin)
  const usableH = screenH * (1 - 2 * margin)
  const zoom = Math.min(usableW / (worldW * SCALE_PX_PER_M), usableH / (worldH * SCALE_PX_PER_M))

  const cam: Camera = { panX: 0, panY: 0, zoom }
  // Centre the bounds: align the world centre with the screen centre.
  const cx = (bounds.minX + bounds.maxX) / 2
  const cy = (bounds.minY + bounds.maxY) / 2
  const [scx, scy] = worldToScreen(cam, originX, originY, cx, cy)
  cam.panX = screenW / 2 - scx
  cam.panY = screenH / 2 - scy
  return cam
}
