import { describe, it, expect } from 'vitest'
import { makeBody, step } from '../../src/physics/integrator'
import type { BoatParams } from '../../src/physics/integrator'
import { boatFromLength } from '../../src/scene/boat'
import { lBerth } from '../../src/scene/dock'
import { worldForcesToBody } from '../../src/physics/forces'
import {
  segmentContact, contactForces, defaultContactTuning,
} from '../../src/physics/contact'

const geo = boatFromLength(40)
const tuning = defaultContactTuning(geo)
const params: BoatParams = {
  mass: geo.massKg,
  addedMassSway: geo.addedMassSway,
  inertia: geo.inertiaYaw,
  addedInertiaYaw: geo.addedInertiaYaw,
  dampSurge: geo.dampSurge,
  dampSway: geo.dampSway,
  dampYaw: geo.dampYaw,
}

// Main quay along +x, outward normal +y; boat berths on the +y side.
const seg = lBerth(100, 20)[0]

describe('segmentContact — one fender vs one wall', () => {
  it('returns null when the fender is on the water side of the edge', () => {
    const r = segmentContact({ x: 10, y: 2 }, { x: 0, y: 0 }, seg, tuning)
    expect(r).toBeNull()
  })

  it('returns null when the fender is beyond the segment ends', () => {
    const r = segmentContact({ x: -5, y: -1 }, { x: 0, y: 0 }, seg, tuning)
    expect(r).toBeNull()
  })

  it('pushes back along the outward normal when penetrating', () => {
    const r = segmentContact({ x: 10, y: -0.5 }, { x: 0, y: 0 }, seg, tuning)!
    expect(r.penetration).toBeCloseTo(0.5, 6)
    expect(r.fy).toBeGreaterThan(0)          // pushed in +y (out of the wall)
    expect(Math.abs(r.fx)).toBeLessThan(1e-6) // no normal-direction x component when at rest
  })

  it('one-way: never sucks the fender back into the wall', () => {
    // A fender exactly on the edge with no penetration produces no force.
    const r = segmentContact({ x: 10, y: 0 }, { x: 0, y: 0 }, seg, tuning)
    expect(r === null || r.fy === 0).toBe(true)
  })

  it('Coulomb friction opposes tangential sliding', () => {
    // Penetrating and sliding in +x → friction acts in -x.
    const r = segmentContact({ x: 10, y: -0.3 }, { x: 1, y: 0 }, seg, tuning)!
    expect(r.fx).toBeLessThan(0)
    // friction is bounded by mu * normal force
    const normalMag = Math.abs(r.fy)
    expect(Math.abs(r.fx)).toBeLessThanOrEqual(tuning.friction * normalMag + 1e-6)
  })
})

describe('contactForces — full hull against the dock', () => {
  it('produces no contact when floating clear of the quay', () => {
    const b = makeBody(20, geo.beamM / 2 + 1, 0)
    const segments = lBerth(100, 20)
    expect(contactForces(b, geo, segments, tuning).length).toBe(0)
  })

  it('pushes the boat off the quay when its fenders penetrate', () => {
    // Drop the boat so its starboard fenders dig into the main quay (y just below beam/2).
    const b = makeBody(20, geo.beamM / 2 - 0.4, 0)
    const segments = lBerth(100, 20)
    const contacts = contactForces(b, geo, segments, tuning)
    expect(contacts.length).toBeGreaterThan(0)
    const net = worldForcesToBody(b, contacts)
    expect(net.fy).toBeGreaterThan(0)  // net push in +y, off the quay
  })

  it('inside corner: a fender near the L can touch two segments', () => {
    // Push the boat into the inside corner so a stern fender straddles both walls.
    const b = makeBody(0.2, geo.beamM / 2 - 0.4, 0)
    const segments = lBerth(100, 20)
    const contacts = contactForces(b, geo, segments, tuning)
    const touchedSegments = new Set(contacts.map(c => c.segmentIndex))
    expect(touchedSegments.size).toBe(2)
  })

  it('settles against the quay without penetrating through it', () => {
    // Boat pressed gently toward the quay; after settling it should rest at/above the edge.
    let b = makeBody(20, geo.beamM / 2 - 0.2, 0)
    const segments = lBerth(100, 20)
    for (let i = 0; i < 2000; i++) {
      const contacts = contactForces(b, geo, segments, tuning)
      // constant push toward the quay (-y) to emulate wind/engine holding it on
      const push = worldForcesToBody(b, [{ label: 'push', point: { x: b.x, y: b.y }, fx: 0, fy: -2000 }])
      const cf = worldForcesToBody(b, contacts)
      b = step(b, params, { fx: push.fx + cf.fx, fy: push.fy + cf.fy, mz: push.mz + cf.mz }, 1 / 60)
    }
    expect(isFinite(b.y)).toBe(true)
    // lowest fender must not be driven far through the wall (no blow-through)
    expect(b.y).toBeGreaterThan(geo.beamM / 2 - 0.6)
  })
})
