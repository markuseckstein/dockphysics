import type { Vec2 } from './boat'

export interface ClosestPoint {
  point: Vec2  // closest point on the segment
  t: number    // parameter along the segment, 0..1
  dist: number // distance from the query point to that closest point
}

// Closest point on segment [a,b] to point p, with the clamped parameter t and
// the resulting distance. Handles a degenerate (zero-length) segment.
export function closestPointOnSegment(p: Vec2, a: Vec2, b: Vec2): ClosestPoint {
  const abx = b.x - a.x
  const aby = b.y - a.y
  const len2 = abx * abx + aby * aby
  let t = 0
  if (len2 > 1e-12) {
    t = ((p.x - a.x) * abx + (p.y - a.y) * aby) / len2
    t = Math.max(0, Math.min(1, t))
  }
  const point = { x: a.x + t * abx, y: a.y + t * aby }
  const dist = Math.hypot(p.x - point.x, p.y - point.y)
  return { point, t, dist }
}
