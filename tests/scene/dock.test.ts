import { describe, it, expect } from 'vitest'
import { lBerth } from '../../src/scene/dock'

describe('lBerth', () => {
  it('returns exactly 2 segments', () => {
    const segs = lBerth(40, 10)
    expect(segs.length).toBe(2)
  })

  it('main quay has the specified length', () => {
    const quayLen = 40
    const segs = lBerth(quayLen, 10)
    const main = segs[0]
    const dx = main.x2 - main.x1
    const dy = main.y2 - main.y1
    const len = Math.hypot(dx, dy)
    expect(len).toBeCloseTo(quayLen, 6)
  })

  it('finger has the specified length', () => {
    const fingerLen = 12
    const segs = lBerth(40, fingerLen)
    const finger = segs[1]
    const dx = finger.x2 - finger.x1
    const dy = finger.y2 - finger.y1
    const len = Math.hypot(dx, dy)
    expect(len).toBeCloseTo(fingerLen, 6)
  })

  it('finger is perpendicular to main quay', () => {
    const segs = lBerth(40, 10)
    const main   = segs[0]
    const finger = segs[1]
    const dxM = main.x2 - main.x1
    const dyM = main.y2 - main.y1
    const dxF = finger.x2 - finger.x1
    const dyF = finger.y2 - finger.y1
    const dot = dxM * dxF + dyM * dyF
    // perpendicular → dot product = 0
    expect(dot).toBeCloseTo(0, 6)
  })

  it('finger shares an endpoint with the main quay', () => {
    const segs = lBerth(40, 10)
    const main   = segs[0]
    const finger = segs[1]
    const sharedX = finger.x1
    const sharedY = finger.y1
    const onMain =
      (Math.abs(sharedX - main.x1) < 1e-9 && Math.abs(sharedY - main.y1) < 1e-9) ||
      (Math.abs(sharedX - main.x2) < 1e-9 && Math.abs(sharedY - main.y2) < 1e-9)
    expect(onMain).toBe(true)
  })

  it('each segment has an outward normal', () => {
    const segs = lBerth(40, 10)
    for (const seg of segs) {
      expect(typeof seg.nx).toBe('number')
      expect(typeof seg.ny).toBe('number')
      const mag = Math.hypot(seg.nx, seg.ny)
      expect(mag).toBeCloseTo(1, 5)
    }
  })
})
