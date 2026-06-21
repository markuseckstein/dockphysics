import { describe, it, expect } from 'vitest'
import { makeBody, step } from '../../src/physics/integrator'
import type { BoatParams } from '../../src/physics/integrator'

const PARAMS: BoatParams = {
  mass: 8000,           // kg
  addedMassSway: 4000,  // kg
  inertia: 60000,       // kg·m²
  addedInertiaYaw: 30000,
  dampSurge: 500,       // N·s/m
  dampSway: 8000,
  dampYaw: 40000,
}

describe('makeBody', () => {
  it('creates a body at origin with zero velocity', () => {
    const b = makeBody(0, 0, 0)
    expect(b.x).toBe(0)
    expect(b.y).toBe(0)
    expect(b.heading).toBe(0)
    expect(b.vx).toBe(0)
    expect(b.vy).toBe(0)
    expect(b.yawRate).toBe(0)
  })

  it('creates a body at specified position and heading', () => {
    const b = makeBody(10, 20, Math.PI / 4)
    expect(b.x).toBe(10)
    expect(b.y).toBe(20)
    expect(b.heading).toBeCloseTo(Math.PI / 4, 8)
  })
})

describe('step — zero forces, zero velocity', () => {
  it('body at rest stays at rest', () => {
    const b = makeBody(5, 3, 0.2)
    const b2 = step(b, PARAMS, { fx: 0, fy: 0, mz: 0 }, 0.016)
    expect(b2.x).toBe(5)
    expect(b2.y).toBe(3)
    expect(b2.heading).toBeCloseTo(0.2, 8)
    expect(b2.vx).toBe(0)
    expect(b2.vy).toBe(0)
    expect(b2.yawRate).toBe(0)
  })
})

describe('step — free motion (no damping params)', () => {
  const nofriction: BoatParams = { ...PARAMS, dampSurge: 0, dampSway: 0, dampYaw: 0 }

  it('constant velocity advances position correctly', () => {
    const b = { x: 0, y: 0, heading: 0, vx: 2, vy: 0, yawRate: 0 }
    const dt = 0.1
    const b2 = step(b, nofriction, { fx: 0, fy: 0, mz: 0 }, dt)
    expect(b2.x).toBeCloseTo(0.2, 8)
    expect(b2.y).toBeCloseTo(0, 8)
    expect(b2.vx).toBeCloseTo(2, 8)
  })

  it('force accelerates body', () => {
    const b = makeBody(0, 0, 0)
    // F = ma → a = F/m_eff.  In body frame heading=0 so world x = surge
    const m_eff = PARAMS.mass  // surge, no added mass
    const fx = 1000  // N
    const dt = 1.0
    const b2 = step(b, nofriction, { fx, fy: 0, mz: 0 }, dt)
    // symplectic: v updated first, then pos
    const expectedVx = fx / m_eff * dt
    const expectedX  = expectedVx * dt
    expect(b2.vx).toBeCloseTo(expectedVx, 6)
    expect(b2.x).toBeCloseTo(expectedX, 6)
  })
})

describe('step — anisotropic damping', () => {
  it('surge decays slower than sway from equal initial speed', () => {
    const dt = 0.1
    const steps = 20

    let bSurge = { x: 0, y: 0, heading: 0, vx: 1, vy: 0, yawRate: 0 }
    let bSway  = { x: 0, y: 0, heading: 0, vx: 0, vy: 1, yawRate: 0 }

    for (let i = 0; i < steps; i++) {
      bSurge = step(bSurge, PARAMS, { fx: 0, fy: 0, mz: 0 }, dt)
      bSway  = step(bSway,  PARAMS, { fx: 0, fy: 0, mz: 0 }, dt)
    }

    // surge should retain more speed than sway
    expect(Math.abs(bSurge.vx)).toBeGreaterThan(Math.abs(bSway.vy))
  })

  it('velocities remain finite and damp to near zero after many steps', () => {
    let b = { x: 0, y: 0, heading: 0, vx: 2, vy: 1, yawRate: 0.5 }
    // Surge time constant τ = mass/dampSurge = 8000/500 = 16s; v0=2 m/s.
    // Need t > ln(v0/0.01)*τ = ln(200)*16 ≈ 85s → 6000 steps × 0.016 = 96s.
    for (let i = 0; i < 6000; i++) {
      b = step(b, PARAMS, { fx: 0, fy: 0, mz: 0 }, 0.016)
    }
    expect(isFinite(b.vx)).toBe(true)
    expect(isFinite(b.vy)).toBe(true)
    expect(isFinite(b.yawRate)).toBe(true)
    expect(Math.abs(b.vx)).toBeLessThan(0.01)
    expect(Math.abs(b.vy)).toBeLessThan(0.01)
    expect(Math.abs(b.yawRate)).toBeLessThan(0.01)
  })
})

describe('step — forces in body frame', () => {
  it('surge force at heading=π/2 moves boat in world-y direction', () => {
    // heading = π/2 means boat points up (+y in world)
    const b = makeBody(0, 0, Math.PI / 2)
    const nofriction: BoatParams = { ...PARAMS, dampSurge: 0, dampSway: 0, dampYaw: 0 }
    const fx = 1000  // surge (forward in body frame)
    const b2 = step(b, nofriction, { fx, fy: 0, mz: 0 }, 1.0)
    // world force = rotate (fx,0) by heading
    // vx_world = fx/m * cos(π/2) ≈ 0
    // vy_world = fx/m * sin(π/2) ≈ fx/m
    expect(Math.abs(b2.vx)).toBeLessThan(1e-9)
    expect(b2.vy).toBeCloseTo(fx / PARAMS.mass, 4)
  })
})
