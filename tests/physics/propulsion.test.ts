import { describe, it, expect } from 'vitest'
import { makeBody, step } from '../../src/physics/integrator'
import type { BoatParams, RigidBody } from '../../src/physics/integrator'
import { boatFromLength } from '../../src/scene/boat'
import {
  controlForces, netForces, propulsionTuning,
  defaultStandstillAuthority, effectiveStandstillAuthority, rudderAngle,
} from '../../src/physics/propulsion'
import type { Controls } from '../../src/physics/propulsion'

const geo = boatFromLength(40)
const tuning = propulsionTuning(geo)
const params: BoatParams = {
  mass: geo.massKg,
  addedMassSway: geo.addedMassSway,
  inertia: geo.inertiaYaw,
  addedInertiaYaw: geo.addedInertiaYaw,
  dampSurge: geo.dampSurge,
  dampSway: geo.dampSway,
  dampYaw: geo.dampYaw,
}

function controls(over: Partial<Controls> = {}): Controls {
  return { throttle: 0, helm: 0, rudderConfig: 'single', standstillAuthority: 0.3, ...over }
}

// Surge velocity through water for a body heading along +x.
function forward(speed: number): RigidBody {
  return { ...makeBody(0, 0, 0), vx: speed }
}

describe('standstill authority configuration', () => {
  it('defaults modest for single, zero for twin', () => {
    expect(defaultStandstillAuthority('single')).toBeGreaterThan(0)
    expect(defaultStandstillAuthority('twin')).toBe(0)
  })

  it('twin is always locked to zero regardless of slider', () => {
    expect(effectiveStandstillAuthority(controls({ rudderConfig: 'twin', standstillAuthority: 1 }))).toBe(0)
  })

  it('single honours and clamps the slider', () => {
    expect(effectiveStandstillAuthority(controls({ standstillAuthority: 0.7 }))).toBeCloseTo(0.7)
    expect(effectiveStandstillAuthority(controls({ standstillAuthority: 5 }))).toBe(1)
  })
})

describe('thrust', () => {
  it('is applied on the centreline at the saildrive and scales with throttle', () => {
    const half = netForces(controlForces(forward(0), geo, controls({ throttle: 0.5 }), tuning))
    const full = netForces(controlForces(forward(0), geo, controls({ throttle: 1.0 }), tuning))
    expect(half.fx).toBeGreaterThan(0)
    expect(full.fx).toBeCloseTo(2 * half.fx, 6)
    expect(half.fy).toBe(0)
  })

  it('reverses with negative throttle', () => {
    const net = netForces(controlForces(forward(0), geo, controls({ throttle: -1 }), tuning))
    expect(net.fx).toBeLessThan(0)
  })

  it('produces no yaw moment by itself (pure centreline force)', () => {
    const net = netForces(controlForces(forward(0), geo, controls({ throttle: 1 }), tuning))
    expect(net.mz).toBeCloseTo(0, 6)
  })
})

describe('rudder side-force ∝ angle × speed²', () => {
  it('is limp at rest (no throttle) and strong when moving', () => {
    const atRest = netForces(controlForces(forward(0), geo, controls({ helm: 1 }), tuning))
    const moving = netForces(controlForces(forward(2), geo, controls({ helm: 1 }), tuning))
    expect(Math.abs(atRest.mz)).toBeCloseTo(0, 6)
    expect(Math.abs(moving.mz)).toBeGreaterThan(1000)
  })

  it('scales with the square of boat speed', () => {
    const slow = netForces(controlForces(forward(1), geo, controls({ helm: 1 }), tuning))
    const fast = netForces(controlForces(forward(2), geo, controls({ helm: 1 }), tuning))
    expect(Math.abs(fast.mz)).toBeCloseTo(4 * Math.abs(slow.mz), 4)
  })

  it('zero helm produces no side force', () => {
    const net = netForces(controlForces(forward(2), geo, controls({ helm: 0 }), tuning))
    expect(net.fy).toBeCloseTo(0, 6)
    expect(net.mz).toBeCloseTo(0, 6)
  })

  it('reverses steering sense when going astern', () => {
    const ahead  = netForces(controlForces(forward(2),  geo, controls({ helm: 1 }), tuning))
    const astern = netForces(controlForces(forward(-2), geo, controls({ helm: 1 }), tuning))
    expect(Math.sign(ahead.mz)).toBe(-Math.sign(astern.mz))
  })
})

describe('preset #4 — standstill steering check', () => {
  // At zero speed, throttle ahead + full helm: single rudder gives a weak but
  // visible kick (prop wash over centre rudder); twin barely responds.
  const inputsSingle = controls({ throttle: 1, helm: 1, rudderConfig: 'single' })
  const inputsTwin   = controls({ throttle: 1, helm: 1, rudderConfig: 'twin', standstillAuthority: 1 })

  it('single rudder produces a non-zero standstill yaw moment', () => {
    const net = netForces(controlForces(forward(0), geo, inputsSingle, tuning))
    expect(Math.abs(net.mz)).toBeGreaterThan(0)
  })

  it('twin produces essentially no standstill yaw moment', () => {
    const net = netForces(controlForces(forward(0), geo, inputsTwin, tuning))
    expect(net.mz).toBeCloseTo(0, 6)
  })

  it('single kicks the boat into a turn from rest, twin does not', () => {
    let single = forward(0)
    let twin = forward(0)
    for (let i = 0; i < 120; i++) {
      single = step(single, params, netForces(controlForces(single, geo, inputsSingle, tuning)), 1 / 60)
      twin   = step(twin,   params, netForces(controlForces(twin,   geo, inputsTwin,   tuning)), 1 / 60)
    }
    expect(Math.abs(single.yawRate)).toBeGreaterThan(Math.abs(twin.yawRate) + 1e-3)
    expect(Math.abs(single.heading)).toBeGreaterThan(0.01) // visibly turned
  })

  it('the standstill kick is weak relative to steering with way on', () => {
    const standstill = netForces(controlForces(forward(0), geo, inputsSingle, tuning))
    const withWay    = netForces(controlForces(forward(2), geo, controls({ throttle: 1, helm: 1 }), tuning))
    expect(Math.abs(standstill.mz)).toBeLessThan(Math.abs(withWay.mz) * 0.5)
  })
})

describe('twin vs single when moving', () => {
  it('both steer comparably with way on (wash term irrelevant)', () => {
    const single = netForces(controlForces(forward(2), geo, controls({ throttle: 0, helm: 1, rudderConfig: 'single' }), tuning))
    const twin   = netForces(controlForces(forward(2), geo, controls({ throttle: 0, helm: 1, rudderConfig: 'twin' }), tuning))
    expect(twin.mz).toBeCloseTo(single.mz, 6)
  })
})

describe('prop walk', () => {
  it('adds a small lateral kick in astern only', () => {
    const ahead  = controlForces(forward(0), geo, controls({ throttle: 1 }), tuning)
    const astern = controlForces(forward(0), geo, controls({ throttle: -1 }), tuning)
    expect(ahead.some(c => c.label === 'prop-walk')).toBe(false)
    expect(astern.some(c => c.label === 'prop-walk')).toBe(true)
  })
})

describe('rudderAngle mapping', () => {
  it('maps full helm to the max rudder angle', () => {
    expect(rudderAngle(controls({ helm: 1 }), tuning)).toBeCloseTo(tuning.maxRudderAngle)
    expect(rudderAngle(controls({ helm: -1 }), tuning)).toBeCloseTo(-tuning.maxRudderAngle)
  })
})

// Integrate the boat alone (engine/rudder + hull drag via step), no lines/wind.
function run(start: RigidBody, inputs: Controls, seconds: number): RigidBody {
  let b = start
  const steps = Math.round(seconds * 60)
  for (let i = 0; i < steps; i++) {
    b = step(b, params, netForces(controlForces(b, geo, inputs, tuning)), 1 / 60)
  }
  return b
}

describe('engine responsiveness', () => {
  it('reaches most of top speed within ~5 s (not the old ~17 s crawl)', () => {
    // τ = 5 s ⇒ v(5 s) ≈ 0.63·Vmax ≈ 1.58 m/s. The old 0.06 damping gave a
    // ~17 s constant, i.e. only ~0.65 m/s after 5 s — well below this floor.
    const b = run(forward(0), controls({ throttle: 1, helm: 0 }), 5)
    expect(b.vx).toBeGreaterThan(1.45)
    expect(b.vx).toBeLessThan(2.5)
  })

  it('keeps a ~5 kn top speed at full throttle (unchanged)', () => {
    const b = run(forward(0), controls({ throttle: 1, helm: 0 }), 40)
    expect(b.vx).toBeGreaterThan(2.4)
    expect(b.vx).toBeLessThan(2.6)
  })

  it('thrust is unchanged in direction by the faster response (pure centreline)', () => {
    const b = run(forward(0), controls({ throttle: 1, helm: 0 }), 5)
    expect(Math.abs(b.vy)).toBeLessThan(1e-9)
    expect(Math.abs(b.yawRate)).toBeLessThan(1e-9)
  })
})

describe('steering authority — tighter turns underway', () => {
  it('full helm at cruise targets a brisk ~0.35 rad/s steady-state yaw rate', () => {
    // ω_ss = |M_rudder| / dampYaw. Rudder is sized (decoupled from the engine
    // tuning) to hit ~0.35 rad/s ≈ 20 °/s at cruise speed with full helm.
    const m = netForces(controlForces(forward(2.5), geo, controls({ throttle: 0, helm: 1 }), tuning))
    const omegaSS = Math.abs(m.mz) / params.dampYaw
    expect(omegaSS).toBeCloseTo(0.35, 1)  // within ±0.05 rad/s
  })

  it('actually swings the bow round quickly when driven', () => {
    // From cruise, full throttle + full helm: heading should sweep past 45°
    // within a few seconds (steady-state ~0.35 rad/s, ~1.7 s build-up).
    const b = run(forward(2.5), controls({ throttle: 1, helm: 1 }), 4)
    expect(Math.abs(b.heading)).toBeGreaterThan(Math.PI / 4)
  })
})

describe('low-speed authority — stronger single-rudder kick at standstill', () => {
  const def = defaultStandstillAuthority('single')

  it('the default single-rudder authority gives a clearly visible standstill turn', () => {
    const b = run(forward(0), controls({ throttle: 1, helm: 1, standstillAuthority: def }), 3)
    expect(Math.abs(b.heading)).toBeGreaterThan(0.05)  // visibly swinging
  })

  it('the single-rudder prop-wash kick beats twin from rest (preset #4 contract)', () => {
    // Both start dead in the water and throttle ahead. The single centre rudder
    // gets the immediate prop-wash kick; twin only bites once it has way on, so
    // single is clearly ahead on heading after a few seconds.
    const single = run(forward(0), controls({ throttle: 1, helm: 1, standstillAuthority: def }), 3)
    const twin   = run(forward(0), controls({ throttle: 1, helm: 1, rudderConfig: 'twin', standstillAuthority: 1 }), 3)
    expect(Math.abs(single.heading)).toBeGreaterThan(Math.abs(twin.heading) * 1.5)
  })

  it('twin has zero authority at the instant of standstill', () => {
    const m = netForces(controlForces(forward(0), geo, controls({ throttle: 1, helm: 1, rudderConfig: 'twin', standstillAuthority: 1 }), tuning))
    expect(m.mz).toBeCloseTo(0, 6)
  })

  it('the standstill kick is still weak relative to steering with way on', () => {
    const standstill = netForces(controlForces(forward(0), geo, controls({ throttle: 1, helm: 1, standstillAuthority: def }), tuning))
    const withWay    = netForces(controlForces(forward(2.5), geo, controls({ throttle: 1, helm: 1, standstillAuthority: def }), tuning))
    expect(Math.abs(standstill.mz)).toBeLessThan(Math.abs(withWay.mz) * 0.5)
  })
})
