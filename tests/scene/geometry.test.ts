import { describe, it, expect } from 'vitest'
import { closestPointOnSegment } from '../../src/scene/geometry'

describe('closestPointOnSegment', () => {
  it('projects onto the interior of the segment', () => {
    const r = closestPointOnSegment({ x: 2, y: 3 }, { x: 0, y: 0 }, { x: 10, y: 0 })
    expect(r.point.x).toBeCloseTo(2, 6)
    expect(r.point.y).toBeCloseTo(0, 6)
    expect(r.t).toBeCloseTo(0.2, 6)
    expect(r.dist).toBeCloseTo(3, 6)
  })

  it('clamps to the start endpoint when projection is before it', () => {
    const r = closestPointOnSegment({ x: -5, y: 4 }, { x: 0, y: 0 }, { x: 10, y: 0 })
    expect(r.point.x).toBeCloseTo(0, 6)
    expect(r.point.y).toBeCloseTo(0, 6)
    expect(r.t).toBe(0)
  })

  it('clamps to the end endpoint when projection is after it', () => {
    const r = closestPointOnSegment({ x: 99, y: 1 }, { x: 0, y: 0 }, { x: 10, y: 0 })
    expect(r.point.x).toBeCloseTo(10, 6)
    expect(r.t).toBe(1)
  })

  it('handles a degenerate zero-length segment', () => {
    const r = closestPointOnSegment({ x: 3, y: 4 }, { x: 0, y: 0 }, { x: 0, y: 0 })
    expect(r.point.x).toBe(0)
    expect(r.point.y).toBe(0)
    expect(r.dist).toBeCloseTo(5, 6)
  })
})
