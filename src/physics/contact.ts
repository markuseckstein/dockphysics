import type { RigidBody } from './integrator'
import type { BoatGeometry, Vec2 } from '../scene/boat'
import type { Segment } from '../scene/dock'
import type { WorldForce } from './forces'

export interface ContactTuning {
  stiffness: number  // N/m of penetration
  damping: number    // N·s/m, normal-direction
  friction: number   // Coulomb coefficient
}

// Contact is stiffer than the docklines (the quay is hard) but still a penalty
// spring kept stable at the capped step. Scaled by mass so it holds bigger boats.
export function defaultContactTuning(geo: BoatGeometry): ContactTuning {
  return {
    stiffness: geo.massKg * 30,   // ~2.4e5 N/m for a 40-footer
    damping: geo.massKg * 4,
    friction: 0.6,
  }
}

export interface ContactResult extends WorldForce {
  label: 'contact'
  segmentIndex: number
  penetration: number
  normal: Vec2  // outward unit normal of the contacted wall
}

// One fender point (world pos p, world velocity v) against one wall segment.
// The berth side is +normal; penetration occurs when the fender crosses to the
// −normal (solid) side, within the segment's extent. Returns the world force
// (penalty normal + Coulomb friction) or null when there is no contact.
export function segmentContact(
  p: Vec2, v: Vec2, seg: Segment, t: ContactTuning,
): { fx: number; fy: number; penetration: number; normal: Vec2 } | null {
  const n = { x: seg.nx, y: seg.ny }

  // Signed distance from the edge along the outward normal; ≥ 0 means clear.
  const d = (p.x - seg.x1) * n.x + (p.y - seg.y1) * n.y
  if (d >= 0) return null
  const penetration = -d

  // Must project within the segment span (ends are handled by the neighbour wall).
  const ex = seg.x2 - seg.x1
  const ey = seg.y2 - seg.y1
  const len = Math.hypot(ex, ey)
  const along = ((p.x - seg.x1) * ex + (p.y - seg.y1) * ey) / len
  if (along < 0 || along > len) return null

  // Penalty normal force, resisting further inward motion (one-way: ≥ 0).
  const vn = v.x * n.x + v.y * n.y
  let normalMag = t.stiffness * penetration - t.damping * vn
  if (normalMag <= 0) return null

  let fx = normalMag * n.x
  let fy = normalMag * n.y

  // Coulomb friction opposing the tangential component of velocity.
  const vtx = v.x - vn * n.x
  const vty = v.y - vn * n.y
  const vtMag = Math.hypot(vtx, vty)
  if (vtMag > 1e-6) {
    const fricMag = t.friction * normalMag
    fx += -fricMag * (vtx / vtMag)
    fy += -fricMag * (vty / vtMag)
  }

  return { fx, fy, penetration, normal: n }
}

// All hull–quay contacts for the current pose: every fender point tested against
// every wall segment. A fender near the inside (L) corner may register against
// two segments at once — both are returned so the corner pushes out cleanly.
export function contactForces(
  body: RigidBody, geo: BoatGeometry, segments: Segment[], t: ContactTuning,
): ContactResult[] {
  const cos = Math.cos(body.heading)
  const sin = Math.sin(body.heading)
  const out: ContactResult[] = []

  for (const f of geo.fenders) {
    const p: Vec2 = { x: body.x + f.x * cos - f.y * sin, y: body.y + f.x * sin + f.y * cos }
    // World velocity of the fender point: v_CoM + ω × r.
    const rx = p.x - body.x
    const ry = p.y - body.y
    const v: Vec2 = { x: body.vx - body.yawRate * ry, y: body.vy + body.yawRate * rx }

    for (let s = 0; s < segments.length; s++) {
      const c = segmentContact(p, v, segments[s], t)
      if (c) {
        out.push({
          label: 'contact', point: p, fx: c.fx, fy: c.fy,
          segmentIndex: s, penetration: c.penetration, normal: c.normal,
        })
      }
    }
  }
  return out
}
