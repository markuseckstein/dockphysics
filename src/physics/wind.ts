import type { RigidBody } from './integrator'
import type { BoatGeometry, Vec2 } from '../scene/boat'
import type { WorldForce } from './forces'

// Steady wind as a world-frame air-velocity vector (the direction the air moves
// toward). No gusts (PRD).
export interface Wind {
  vx: number
  vy: number
}

// Build a wind vector from a speed (m/s) and the world-frame direction the wind
// blows toward (radians, 0 = +x, CCW positive).
export function windFromSpeedDir(speedMs: number, dirTowardRad: number): Wind {
  return { vx: speedMs * Math.cos(dirTowardRad), vy: speedMs * Math.sin(dirTowardRad) }
}

export interface WindTuning {
  rho: number  // air density, kg/m³
  cd: number   // drag coefficient
}

export function defaultWindTuning(): WindTuning {
  return { rho: 1.225, cd: 1.0 }
}

// Apparent wind = true wind − boat velocity (both world frame).
export function apparentWind(body: RigidBody, wind: Wind): Vec2 {
  return { x: wind.vx - body.vx, y: wind.vy - body.vy }
}

// Angle-dependent projected area: the side (lateral) area when beam-on, the
// frontal area when bow-on, blended by the apparent-wind angle to the hull.
export function projectedArea(body: RigidBody, apparent: Vec2, geo: BoatGeometry): number {
  const speed = Math.hypot(apparent.x, apparent.y)
  if (speed < 1e-9) return 0
  // Apparent-wind direction relative to the boat's centreline.
  const cos = Math.cos(body.heading)
  const sin = Math.sin(body.heading)
  const along =  (apparent.x * cos + apparent.y * sin) / speed  // cos(angle to bow)
  const across = (-apparent.x * sin + apparent.y * cos) / speed // sin(angle to beam)
  return geo.windageFrontalArea * Math.abs(along) + geo.windageLateralArea * Math.abs(across)
}

// World wind force, applied at the windage centre (forward of midships). Force is
// 0.5·ρ·Cd·A(angle)·V²  in the apparent-wind direction. Null when there is no wind.
export function windForce(
  body: RigidBody, geo: BoatGeometry, wind: Wind, t: WindTuning,
): WorldForce | null {
  const aw = apparentWind(body, wind)
  const speed = Math.hypot(aw.x, aw.y)
  if (speed < 1e-9) return null

  const area = projectedArea(body, aw, geo)
  const mag = 0.5 * t.rho * t.cd * area * speed * speed
  if (mag < 1e-9) return null

  // Application point: windage centre rotated into the world frame.
  const cos = Math.cos(body.heading)
  const sin = Math.sin(body.heading)
  const wc = geo.windageCentre
  const point: Vec2 = {
    x: body.x + wc.x * cos - wc.y * sin,
    y: body.y + wc.x * sin + wc.y * cos,
  }
  return { label: 'wind', point, fx: mag * (aw.x / speed), fy: mag * (aw.y / speed) }
}
