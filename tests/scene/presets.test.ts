import { describe, it, expect } from 'vitest'
import { makeBody, step } from '../../src/physics/integrator'
import type { BoatParams } from '../../src/physics/integrator'
import { boatFromLength } from '../../src/scene/boat'
import { lBerth } from '../../src/scene/dock'
import { PRESETS, buildPreset } from '../../src/scene/presets'
import { worldForcesToBody } from '../../src/physics/forces'
import { lineForce } from '../../src/physics/lines'
import { windForce, defaultWindTuning } from '../../src/physics/wind'
import { controlForces, netForces, propulsionTuning } from '../../src/physics/propulsion'
import { contactForces, defaultContactTuning } from '../../src/physics/contact'

function paramsFor(geo: ReturnType<typeof boatFromLength>): BoatParams {
  return {
    mass: geo.massKg,
    addedMassSway: geo.addedMassSway,
    inertia: geo.inertiaYaw,
    addedInertiaYaw: geo.addedInertiaYaw,
    dampSurge: geo.dampSurge,
    dampSway: geo.dampSway,
    dampYaw: geo.dampYaw,
  }
}

describe('presets catalogue', () => {
  it('exposes the four acceptance presets', () => {
    expect(PRESETS.length).toBe(4)
    const ids = PRESETS.map(p => p.id)
    expect(ids).toContain('weathercock')
    expect(ids).toContain('spring-stern-in')
    expect(ids).toContain('spring-off-bow')
    expect(ids).toContain('standstill-steering')
  })
})

describe('buildPreset', () => {
  it('weathercock has wind and no lines or engine', () => {
    const s = buildPreset('weathercock')
    expect(s.wind.speed).toBeGreaterThan(0)
    expect(s.lines.length).toBe(0)
    expect(s.controls.throttle ?? 0).toBe(0)
  })

  it('spring-stern-in makes fast a bow spring and drives ahead', () => {
    const s = buildPreset('spring-stern-in')
    expect(s.lines.length).toBeGreaterThanOrEqual(1)
    expect((s.controls.throttle ?? 0)).toBeGreaterThan(0)
  })

  it('spring-off-bow makes fast a line and drives astern', () => {
    const s = buildPreset('spring-off-bow')
    expect(s.lines.length).toBeGreaterThanOrEqual(1)
    expect((s.controls.throttle ?? 0)).toBeLessThan(0)
  })
})

// End-to-end behavioural checks: a preset, simulated forward, reproduces its
// described outcome. These are the slice's definition of done.

interface HullPointHistory { startY: number; endY: number }

// Run a preset forward and report where the bow and stern ended up (world y),
// the quay being at y = 0 on the −y side of the boat.
function simulate(id: Parameters<typeof buildPreset>[0], steps = 1500) {
  const s = buildPreset(id)
  const geo = boatFromLength(s.lengthFt)
  const params = paramsFor(geo)
  const segments = lBerth(geo.lengthM * 2.5, geo.beamM * 4)
  const tuning = propulsionTuning(geo)
  const windTuning = defaultWindTuning()
  const contactTuning = defaultContactTuning(geo)
  const controls = { throttle: 0, helm: 0, rudderConfig: 'single' as const, standstillAuthority: 0.3, ...s.controls }
  const lines = s.lines.map(l => ({ ...l, dockPoint: { ...l.dockPoint } }))

  const halfL = geo.lengthM / 2
  const yAt = (b: typeof s.body, bx: number) => b.y + bx * Math.sin(b.heading)
  const bow: HullPointHistory = { startY: yAt(s.body, halfL), endY: 0 }
  const stern: HullPointHistory = { startY: yAt(s.body, -halfL), endY: 0 }

  let b = { ...s.body }
  for (let i = 0; i < steps; i++) {
    const forces = [
      ...lines.map(l => lineForce(b, geo, l)).filter((f): f is NonNullable<typeof f> => f !== null),
      ...contactForces(b, geo, segments, contactTuning),
    ]
    const wf = windForce(b, geo, { vx: 0, vy: 0 }, windTuning)
    if (wf) forces.push(wf)
    const world = worldForcesToBody(b, forces)
    const ctrl = netForces(controlForces(b, geo, controls, tuning))
    b = step(b, params, { fx: ctrl.fx + world.fx, fy: ctrl.fy + world.fy, mz: ctrl.mz + world.mz }, 1 / 60)
  }
  bow.endY = yAt(b, halfL)
  stern.endY = yAt(b, -halfL)
  return { s, geo, body: b, bow, stern }
}

describe('preset #2 — spring the stern in (headline demo)', () => {
  it('pivots about the bow spring; the stern walks in and settles on the fenders', () => {
    const r = simulate('spring-stern-in')
    // Stern moves toward the quay (smaller y) — it "walks in".
    expect(r.stern.endY).toBeLessThan(r.stern.startY - 0.2)
    // The boat is held by the spring, not driving off down the quay.
    expect(r.body.x).toBeLessThan(r.s.body.x + r.geo.lengthM)
    // It comes to rest against the fenders (≈ half-beam off the quay), not through it.
    expect(r.stern.endY).toBeGreaterThan(r.geo.beamM / 2 - 0.3)
  })
})

describe('preset #3 — spring off the bow', () => {
  it('the bow swings out away from the quay', () => {
    const r = simulate('spring-off-bow')
    expect(r.bow.endY).toBeGreaterThan(r.bow.startY + 0.2)
    // Held by the spring rather than running off astern.
    expect(r.body.x).toBeGreaterThan(r.s.body.x - r.geo.lengthM)
  })
})
