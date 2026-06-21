import { describe, it, expect } from 'vitest'
import {
  makeCamera, worldToScreen, screenToWorld, fitCamera,
} from '../../src/render/camera'

const ORIGIN_X = 50
const ORIGIN_Y = 420

describe('camera transform', () => {
  it('default camera places the world origin at the screen origin', () => {
    const cam = makeCamera()
    const [sx, sy] = worldToScreen(cam, ORIGIN_X, ORIGIN_Y, 0, 0)
    expect(sx).toBeCloseTo(ORIGIN_X, 6)
    expect(sy).toBeCloseTo(ORIGIN_Y, 6)
  })

  it('screenToWorld inverts worldToScreen', () => {
    const cam = { panX: 30, panY: -20, zoom: 1.7 }
    const [sx, sy] = worldToScreen(cam, ORIGIN_X, ORIGIN_Y, 12, 7)
    const w = screenToWorld(cam, ORIGIN_X, ORIGIN_Y, sx, sy)
    expect(w.x).toBeCloseTo(12, 6)
    expect(w.y).toBeCloseTo(7, 6)
  })

  it('zoom scales distances from the origin', () => {
    const a = worldToScreen(makeCamera(), ORIGIN_X, ORIGIN_Y, 10, 0)
    const b = worldToScreen({ panX: 0, panY: 0, zoom: 2 }, ORIGIN_X, ORIGIN_Y, 10, 0)
    const da = a[0] - ORIGIN_X
    const db = b[0] - ORIGIN_X
    expect(db).toBeCloseTo(2 * da, 6)
  })

  it('y increases upward in world but downward on screen', () => {
    const [, sy] = worldToScreen(makeCamera(), ORIGIN_X, ORIGIN_Y, 0, 10)
    expect(sy).toBeLessThan(ORIGIN_Y)
  })
})

describe('fitCamera', () => {
  it('frames the bounds inside the viewport', () => {
    const bounds = { minX: 0, minY: 0, maxX: 100, maxY: 40 }
    const cam = fitCamera(bounds, 900, 500, ORIGIN_X, ORIGIN_Y)
    // every corner must land within the screen
    for (const [wx, wy] of [[0, 0], [100, 0], [0, 40], [100, 40]] as const) {
      const [sx, sy] = worldToScreen(cam, ORIGIN_X, ORIGIN_Y, wx, wy)
      expect(sx).toBeGreaterThanOrEqual(0)
      expect(sx).toBeLessThanOrEqual(900)
      expect(sy).toBeGreaterThanOrEqual(0)
      expect(sy).toBeLessThanOrEqual(500)
    }
    expect(cam.zoom).toBeGreaterThan(0)
  })
})
