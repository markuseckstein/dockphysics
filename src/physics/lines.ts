import type { RigidBody } from './integrator'
import type { BoatGeometry, Vec2 } from '../scene/boat'
import type { WorldForce } from './forces'

// A dockline: one end fixed to a boat cleat (body frame), the other free along a
// dock edge (world frame). It is a stiff one-way penalty spring — it can only
// ever pull (tension), never push.
export interface Line {
  cleatIndex: number  // index into BoatGeometry.cleats
  dockPoint: Vec2     // world-frame attach point on a dock segment edge
  restLength: number  // m; line is slack while distance ≤ restLength
  stiffness: number   // N/m
  damping: number     // N·s/m, light, for numerical calm
}

// Penalty-spring constants, kept sane for stability at the capped 1/60 s step.
export const DEFAULT_LINE_STIFFNESS = 40000  // N/m → 0.1 m stretch ≈ 4 kN
export const DEFAULT_LINE_DAMPING = 3000     // N·s/m
export const EASE_STEP = 0.25                // m of rope per ease / take-up press

// World position of a boat cleat for the current pose.
export function cleatWorld(body: RigidBody, geo: BoatGeometry, cleatIndex: number): Vec2 {
  const c = geo.cleats[cleatIndex]
  const cos = Math.cos(body.heading)
  const sin = Math.sin(body.heading)
  return { x: body.x + c.x * cos - c.y * sin, y: body.y + c.x * sin + c.y * cos }
}

// World velocity of a boat cleat: v_CoM + ω × r.
function cleatVel(body: RigidBody, world: Vec2): Vec2 {
  const rx = world.x - body.x
  const ry = world.y - body.y
  return { x: body.vx - body.yawRate * ry, y: body.vy + body.yawRate * rx }
}

// Make a line fast "just taut": rest length = current cleat→dock distance, so it
// starts with ~zero tension and tightens the moment the boat moves away.
export function makeLine(
  body: RigidBody, geo: BoatGeometry, cleatIndex: number, dockPoint: Vec2,
  opts: { stiffness?: number; damping?: number } = {},
): Line {
  const cw = cleatWorld(body, geo, cleatIndex)
  const restLength = Math.hypot(dockPoint.x - cw.x, dockPoint.y - cw.y)
  return {
    cleatIndex,
    dockPoint: { ...dockPoint },
    restLength,
    stiffness: opts.stiffness ?? DEFAULT_LINE_STIFFNESS,
    damping: opts.damping ?? DEFAULT_LINE_DAMPING,
  }
}

export function easeLine(line: Line, step = EASE_STEP): void {
  line.restLength += step
}

export function takeUpLine(line: Line, step = EASE_STEP): void {
  line.restLength = Math.max(0, line.restLength - step)
}

// Current tension (N), clamped ≥ 0 (one-way). Zero when the line is slack.
export function lineTension(body: RigidBody, geo: BoatGeometry, line: Line): number {
  const cw = cleatWorld(body, geo, line.cleatIndex)
  const dx = line.dockPoint.x - cw.x
  const dy = line.dockPoint.y - cw.y
  const dist = Math.hypot(dx, dy)
  const stretch = dist - line.restLength
  if (stretch <= 0 || dist < 1e-9) return 0  // slack

  // Stretch rate along the line: positive when the cleat moves away from the dock.
  const ux = dx / dist
  const uy = dy / dist
  const v = cleatVel(body, cw)
  const stretchRate = -(v.x * ux + v.y * uy)

  const tension = line.stiffness * stretch + line.damping * stretchRate
  return Math.max(0, tension)
}

// World force the line exerts on the boat at its cleat (pulling toward the dock),
// or null when the line is slack.
export function lineForce(body: RigidBody, geo: BoatGeometry, line: Line): WorldForce | null {
  const tension = lineTension(body, geo, line)
  if (tension <= 0) return null
  const cw = cleatWorld(body, geo, line.cleatIndex)
  const dx = line.dockPoint.x - cw.x
  const dy = line.dockPoint.y - cw.y
  const dist = Math.hypot(dx, dy)
  if (dist < 1e-9) return null
  return { label: 'line', point: cw, fx: (dx / dist) * tension, fy: (dy / dist) * tension }
}
