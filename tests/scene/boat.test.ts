import { describe, it, expect } from 'vitest'
import { boatFromLength } from '../../src/scene/boat'
import { ftToM } from '../../src/units'

describe('boatFromLength', () => {
  it('lengthM equals input length in metres', () => {
    const b = boatFromLength(40)
    expect(b.lengthM).toBeCloseTo(ftToM(40), 4)
  })

  it('beam is roughly 1/3 of length (Bavaria Cruiser ratio)', () => {
    const b = boatFromLength(40)
    const ratio = b.beamM / b.lengthM
    expect(ratio).toBeGreaterThan(0.25)
    expect(ratio).toBeLessThan(0.40)
  })

  it('mass grows with length (8 t at 37 ft → ~12 t at 49 ft)', () => {
    const small = boatFromLength(37)
    const large = boatFromLength(49)
    expect(small.massKg).toBeGreaterThan(6000)
    expect(large.massKg).toBeGreaterThan(small.massKg)
    expect(large.massKg).toBeLessThan(14000)
  })

  it('yaw inertia scales with mass and length squared', () => {
    const b37 = boatFromLength(37)
    const b49 = boatFromLength(49)
    expect(b49.inertiaYaw).toBeGreaterThan(b37.inertiaYaw)
    // slender-body approx: I ≈ m * L^2 / 12
    const expected = b37.massKg * b37.lengthM ** 2 / 12
    expect(b37.inertiaYaw).toBeCloseTo(expected, -2)  // within ~100 kg·m²
  })

  it('has six cleats (bow/midship/stern × port/stbd)', () => {
    const b = boatFromLength(40)
    expect(b.cleats.length).toBe(6)
  })

  it('cleats are within hull bounds', () => {
    const b = boatFromLength(43)
    const halfL = b.lengthM / 2
    const halfB = b.beamM / 2
    for (const c of b.cleats) {
      expect(Math.abs(c.x)).toBeLessThanOrEqual(halfL + 0.01)
      expect(Math.abs(c.y)).toBeLessThanOrEqual(halfB + 0.01)
    }
  })

  it('added mass sway is a positive fraction of mass', () => {
    const b = boatFromLength(40)
    expect(b.addedMassSway).toBeGreaterThan(0)
    expect(b.addedMassSway).toBeLessThan(b.massKg * 2)
  })
})
