import { describe, it, expect } from 'vitest'
import { ftToM, mToFt, knotsToMs, msToKnots, SCALE_PX_PER_M } from '../src/units'

describe('unit conversions', () => {
  it('ftToM: 1 foot = 0.3048 m', () => {
    expect(ftToM(1)).toBeCloseTo(0.3048, 4)
  })

  it('ftToM: 37 ft ≈ 11.278 m', () => {
    expect(ftToM(37)).toBeCloseTo(37 * 0.3048, 3)
  })

  it('mToFt is inverse of ftToM', () => {
    expect(mToFt(ftToM(42))).toBeCloseTo(42, 6)
  })

  it('knotsToMs: 1 knot ≈ 0.5144 m/s', () => {
    expect(knotsToMs(1)).toBeCloseTo(0.5144, 3)
  })

  it('msToKnots is inverse of knotsToMs', () => {
    expect(msToKnots(knotsToMs(3))).toBeCloseTo(3, 6)
  })

  it('SCALE_PX_PER_M is a positive number', () => {
    expect(SCALE_PX_PER_M).toBeGreaterThan(0)
  })
})
