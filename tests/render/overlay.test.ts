import { describe, it, expect } from 'vitest'
import { pivotPoint } from '../../src/render/overlay'
import { makeBody } from '../../src/physics/integrator'

describe('pivotPoint (instantaneous centre of rotation)', () => {
  it('is null for pure translation (no yaw)', () => {
    const b = { ...makeBody(0, 0, 0), vx: 1, vy: 0, yawRate: 0 }
    expect(pivotPoint(b)).toBeNull()
  })

  it('is the zero-velocity point of the rigid body', () => {
    // vx=1, ω=1 about origin → pivot at (0, 1): velocity there cancels.
    const b = { ...makeBody(0, 0, 0), vx: 1, vy: 0, yawRate: 1 }
    const p = pivotPoint(b)!
    expect(p.x).toBeCloseTo(0, 8)
    expect(p.y).toBeCloseTo(1, 8)
    // Verify: v_com + ω × (p - com) = 0
    const rx = p.x - b.x, ry = p.y - b.y
    expect(b.vx - b.yawRate * ry).toBeCloseTo(0, 8)
    expect(b.vy + b.yawRate * rx).toBeCloseTo(0, 8)
  })

  it('translates with the body origin', () => {
    const b = { ...makeBody(5, -3, 0), vx: 0, vy: 2, yawRate: 1 }
    const p = pivotPoint(b)!
    expect(p.x).toBeCloseTo(5 - 2, 8)
    expect(p.y).toBeCloseTo(-3 + 0, 8)
  })
})
