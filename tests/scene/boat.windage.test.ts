import { describe, it, expect } from 'vitest'
import { boatFromLength } from '../../src/scene/boat'

describe('boat windage parameters', () => {
  it('lateral (beam-on) windage area exceeds frontal (bow-on) area', () => {
    const b = boatFromLength(40)
    expect(b.windageLateralArea).toBeGreaterThan(b.windageFrontalArea)
  })

  it('windage centre is forward of midships (body +x)', () => {
    const b = boatFromLength(40)
    expect(b.windageCentre.x).toBeGreaterThan(0)
  })

  it('areas scale up with boat length', () => {
    const small = boatFromLength(37)
    const large = boatFromLength(49)
    expect(large.windageLateralArea).toBeGreaterThan(small.windageLateralArea)
    expect(large.windageFrontalArea).toBeGreaterThan(small.windageFrontalArea)
  })
})

describe('boat fender points', () => {
  it('provides fender points around the hull', () => {
    const b = boatFromLength(40)
    expect(b.fenders.length).toBeGreaterThanOrEqual(6)
  })

  it('fender points lie within the hull bounding box', () => {
    const b = boatFromLength(43)
    const halfL = b.lengthM / 2
    const halfB = b.beamM / 2
    for (const f of b.fenders) {
      expect(Math.abs(f.x)).toBeLessThanOrEqual(halfL + 0.01)
      expect(Math.abs(f.y)).toBeLessThanOrEqual(halfB + 0.01)
    }
  })
})
