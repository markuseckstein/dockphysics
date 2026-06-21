import { describe, it, expect } from 'vitest'
import { makeBody, step } from '../../src/physics/integrator'
import type { BoatParams } from '../../src/physics/integrator'
import { boatFromLength } from '../../src/scene/boat'
import { worldForcesToBody } from '../../src/physics/forces'
import {
  windFromSpeedDir, apparentWind, projectedArea, windForce, defaultWindTuning,
} from '../../src/physics/wind'

const geo = boatFromLength(40)
const tuning = defaultWindTuning()
const params: BoatParams = {
  mass: geo.massKg,
  addedMassSway: geo.addedMassSway,
  inertia: geo.inertiaYaw,
  addedInertiaYaw: geo.addedInertiaYaw,
  dampSurge: geo.dampSurge,
  dampSway: geo.dampSway,
  dampYaw: geo.dampYaw,
}

describe('apparent wind', () => {
  it('equals true wind when the boat is stationary', () => {
    const wind = windFromSpeedDir(10, 0)  // blowing toward +x at 10 m/s
    const aw = apparentWind(makeBody(0, 0, 0), wind)
    expect(aw.x).toBeCloseTo(10, 6)
    expect(aw.y).toBeCloseTo(0, 6)
  })

  it('is true wind minus boat velocity', () => {
    const wind = windFromSpeedDir(10, 0)
    const b = { ...makeBody(0, 0, 0), vx: 4, vy: 0 }
    const aw = apparentWind(b, wind)
    expect(aw.x).toBeCloseTo(6, 6)
  })
})

describe('angle-dependent projected area', () => {
  it('beam-on catches much more than bow-on', () => {
    const heading0 = makeBody(0, 0, 0)  // boat points +x
    const bowOn  = projectedArea(heading0, { x: -1, y: 0 }, geo)  // wind along centreline
    const beamOn = projectedArea(heading0, { x: 0, y: -1 }, geo)  // wind across the beam
    expect(beamOn).toBeGreaterThan(bowOn * 1.5)
  })
})

describe('wind force', () => {
  it('grows with the square of apparent wind speed', () => {
    const b = makeBody(0, 0, 0)
    const slow = windForce(b, geo, windFromSpeedDir(5, Math.PI / 2), tuning)!
    const fast = windForce(b, geo, windFromSpeedDir(10, Math.PI / 2), tuning)!
    const slowMag = Math.hypot(slow.fx, slow.fy)
    const fastMag = Math.hypot(fast.fx, fast.fy)
    expect(fastMag).toBeCloseTo(4 * slowMag, 1)
  })

  it('is applied forward of midships (windage centre)', () => {
    const b = makeBody(0, 0, 0)
    const f = windForce(b, geo, windFromSpeedDir(10, Math.PI / 2), tuning)!
    // application point ahead of CoM in body +x
    expect(f.point.x).toBeGreaterThan(b.x)
  })
})

describe('preset #1 — weathercock', () => {
  it('beam wind with no lines/engine blows the bow off downwind', () => {
    // Boat heading +x; wind blows toward -y (beam wind from port).
    let b = makeBody(0, 0, 0)
    const wind = windFromSpeedDir(12, -Math.PI / 2)
    for (let i = 0; i < 600; i++) {
      const f = windForce(b, geo, wind, tuning)
      const bodyF = worldForcesToBody(b, f ? [f] : [])
      b = step(b, params, bodyF, 1 / 60)
    }
    // The boat drifts downwind (-y) ...
    expect(b.y).toBeLessThan(0)
    // ... and the bow swings off the wind: heading rotates toward the downwind side.
    expect(b.heading).toBeLessThan(-0.05)
  })
})
