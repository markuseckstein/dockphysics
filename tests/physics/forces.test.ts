import { describe, it, expect } from 'vitest'
import { makeBody } from '../../src/physics/integrator'
import { worldForcesToBody } from '../../src/physics/forces'

describe('worldForcesToBody', () => {
  it('passes a world +x force through unchanged at heading 0', () => {
    const b = makeBody(0, 0, 0)
    const f = worldForcesToBody(b, [{ label: 'x', point: { x: 0, y: 0 }, fx: 100, fy: 0 }])
    expect(f.fx).toBeCloseTo(100, 6)
    expect(f.fy).toBeCloseTo(0, 6)
    expect(f.mz).toBeCloseTo(0, 6)
  })

  it('rotates a world +x force into body sway at heading +pi/2', () => {
    const b = makeBody(0, 0, Math.PI / 2)
    const f = worldForcesToBody(b, [{ label: 'x', point: { x: 0, y: 0 }, fx: 100, fy: 0 }])
    // boat points +y; a world +x force is felt as -sway (starboard)
    expect(f.fx).toBeCloseTo(0, 6)
    expect(f.fy).toBeCloseTo(-100, 6)
  })

  it('produces a yaw moment for an offset force (frame independent)', () => {
    const b = makeBody(0, 0, 0)
    // +y force applied 2 m ahead (world +x) of the CoM → +CCW moment
    const f = worldForcesToBody(b, [{ label: 'y', point: { x: 2, y: 0 }, fx: 0, fy: 50 }])
    expect(f.mz).toBeCloseTo(2 * 50, 6)
  })

  it('sums multiple world forces', () => {
    const b = makeBody(0, 0, 0)
    const f = worldForcesToBody(b, [
      { label: 'a', point: { x: 0, y: 0 }, fx: 100, fy: 0 },
      { label: 'b', point: { x: 0, y: 0 }, fx: -30, fy: 20 },
    ])
    expect(f.fx).toBeCloseTo(70, 6)
    expect(f.fy).toBeCloseTo(20, 6)
  })
})
