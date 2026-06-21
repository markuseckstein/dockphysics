import { describe, it, expect } from 'vitest'
import { makeBody, step } from '../../src/physics/integrator'
import type { BoatParams, RigidBody } from '../../src/physics/integrator'
import { boatFromLength } from '../../src/scene/boat'
import { worldForcesToBody } from '../../src/physics/forces'
import {
  makeLine, lineTension, lineForce, cleatWorld, easeLine, takeUpLine, EASE_STEP,
} from '../../src/physics/lines'

const geo = boatFromLength(40)
const params: BoatParams = {
  mass: geo.massKg,
  addedMassSway: geo.addedMassSway,
  inertia: geo.inertiaYaw,
  addedInertiaYaw: geo.addedInertiaYaw,
  dampSurge: geo.dampSurge,
  dampSway: geo.dampSway,
  dampYaw: geo.dampYaw,
}

// Place the boat at the origin, heading 0, so cleats sit at known body offsets.
function restBody(): RigidBody {
  return makeBody(0, 0, 0)
}

describe('makeLine — just taut', () => {
  it('creates a line whose rest length equals the current cleat→dock distance', () => {
    const b = restBody()
    const cw = cleatWorld(b, geo, 0)
    const dock = { x: cw.x + 3, y: cw.y + 4 }  // 5 m away
    const line = makeLine(b, geo, 0, dock)
    expect(line.restLength).toBeCloseTo(5, 6)
    // just-taut → no tension yet
    expect(lineTension(b, geo, line)).toBeCloseTo(0, 6)
  })
})

describe('one-way penalty spring', () => {
  it('is slack (zero tension, no force) when distance <= rest length', () => {
    const b = restBody()
    const cw = cleatWorld(b, geo, 0)
    const line = makeLine(b, geo, 0, { x: cw.x + 5, y: cw.y })
    // take up a metre so the rope is longer than the gap → slack
    line.restLength = 7
    expect(lineTension(b, geo, line)).toBe(0)
    expect(lineForce(b, geo, line)).toBeNull()
  })

  it('tension = stiffness × stretch when stretched', () => {
    const b = restBody()
    const cw = cleatWorld(b, geo, 0)
    const line = makeLine(b, geo, 0, { x: cw.x + 5, y: cw.y })
    line.restLength = 4   // 1 m of stretch
    line.damping = 0
    expect(lineTension(b, geo, line)).toBeCloseTo(line.stiffness * 1, 4)
  })

  it('pulls the cleat toward the dock point', () => {
    const b = restBody()
    const cw = cleatWorld(b, geo, 0)
    const line = makeLine(b, geo, 0, { x: cw.x + 5, y: cw.y })
    line.restLength = 4
    line.damping = 0
    const f = lineForce(b, geo, line)!
    // dock is in +x from the cleat → force on boat is +x
    expect(f.fx).toBeGreaterThan(0)
    expect(Math.abs(f.fy)).toBeLessThan(1e-6)
    expect(f.point.x).toBeCloseTo(cw.x, 6)
  })
})

describe('ease / take-up', () => {
  it('ease increases rest length, take-up decreases it', () => {
    const b = restBody()
    const cw = cleatWorld(b, geo, 0)
    const line = makeLine(b, geo, 0, { x: cw.x + 5, y: cw.y })
    const r0 = line.restLength
    easeLine(line)
    expect(line.restLength).toBeCloseTo(r0 + EASE_STEP, 6)
    takeUpLine(line)
    takeUpLine(line)
    expect(line.restLength).toBeCloseTo(r0 - EASE_STEP, 6)
  })

  it('take-up never drives rest length negative', () => {
    const line = makeLine(restBody(), geo, 0, { x: 100, y: 0 })
    line.restLength = 0.1
    takeUpLine(line)
    expect(line.restLength).toBeGreaterThanOrEqual(0)
  })
})

describe('stability', () => {
  it('a stretched line settles the boat without blowing up', () => {
    let b = makeBody(0, 0, 0)
    const cw = cleatWorld(b, geo, 0)
    const line = makeLine(b, geo, 0, { x: cw.x + 2, y: cw.y })
    line.restLength = 0  // maximally stretched to stress the integrator
    for (let i = 0; i < 3000; i++) {
      const lf = lineForce(b, geo, line)
      const bodyF = worldForcesToBody(b, lf ? [lf] : [])
      b = step(b, params, bodyF, 1 / 60)
    }
    expect(isFinite(b.x)).toBe(true)
    expect(isFinite(b.y)).toBe(true)
    expect(Math.abs(b.x)).toBeLessThan(1000)
  })
})
