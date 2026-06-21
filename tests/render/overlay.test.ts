import { describe, it, expect } from 'vitest'
import { pivotPoint, drawLegend } from '../../src/render/overlay'
import { makeBody } from '../../src/physics/integrator'

// Minimal 2D-context stub that records the calls drawLegend makes.
function stubCtx() {
  const arcs: number[][] = []
  const texts: string[] = []
  const rects: number[][] = []
  const ctx = {
    canvas: { width: 880, height: 520 },
    font: '', textBaseline: '', fillStyle: '', strokeStyle: '', lineWidth: 0,
    fillText: (t: string) => { texts.push(t) },
    fillRect: (...a: number[]) => { rects.push(a) },
    arc: (...a: number[]) => { arcs.push(a) },
    beginPath() {}, moveTo() {}, lineTo() {}, stroke() {}, fill() {}, closePath() {},
  }
  return { ctx: ctx as unknown as CanvasRenderingContext2D, arcs, texts, rects }
}

describe('pivotPoint (instantaneous centre of rotation)', () => {
  it('is null for pure translation (no yaw)', () => {
    const b = { ...makeBody(0, 0, 0), vx: 1, vy: 0, yawRate: 0 }
    expect(pivotPoint(b)).toBeNull()
  })

  it('is the zero-velocity point of the rigid body', () => {
    // vx=1, ω=1 about origin → pivot at (0, 1): velocity there cancels.
    const b = { ...makeBody(0, 0, 0), vx: 1, vy: 0, yawRate: 1 }
    const p = pivotPoint(b)!
    expect(p.x).toBeCloseTo(0, 8)
    expect(p.y).toBeCloseTo(1, 8)
    // Verify: v_com + ω × (p - com) = 0
    const rx = p.x - b.x, ry = p.y - b.y
    expect(b.vx - b.yawRate * ry).toBeCloseTo(0, 8)
    expect(b.vy + b.yawRate * rx).toBeCloseTo(0, 8)
  })

  it('translates with the body origin', () => {
    const b = { ...makeBody(5, -3, 0), vx: 0, vy: 2, yawRate: 1 }
    const p = pivotPoint(b)!
    expect(p.x).toBeCloseTo(5 - 2, 8)
    expect(p.y).toBeCloseTo(-3 + 0, 8)
  })
})

describe('legend', () => {
  it('lists the pivot point and draws it as a crosshair glyph (arc), not a swatch', () => {
    const { ctx, arcs, texts } = stubCtx()
    drawLegend(ctx, { showHullDrag: false, showContact: false })
    expect(texts).toContain('pivot point')
    // The crosshair circle is the only arc drawn by the legend.
    expect(arcs.length).toBeGreaterThanOrEqual(1)
  })

  it('shows hull-drag and contact rows only when their overlays are on', () => {
    const off = stubCtx()
    drawLegend(off.ctx, { showHullDrag: false, showContact: false })
    expect(off.texts).not.toContain('hull drag')
    expect(off.texts).not.toContain('contact')

    const on = stubCtx()
    drawLegend(on.ctx, { showHullDrag: true, showContact: true })
    expect(on.texts).toContain('hull drag')
    expect(on.texts).toContain('contact')
  })
})
