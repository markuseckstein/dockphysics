import type { RigidBody, BoatParams } from '../physics/integrator'
import { computeHullDrag } from '../physics/integrator'
import type { Vec2 } from '../scene/boat'
import type { ForceComponent } from '../physics/propulsion'
import { netForces } from '../physics/propulsion'
import type { WorldForce } from '../physics/forces'
import type { Camera } from './camera'
import { worldToScreen } from './camera'

// On-screen force scale: pixels of arrow per newton (zoom-independent so arrows
// stay readable). Tuned so ~1 kN draws ~30 px.
export const FORCE_PX_PER_N = 0.03
const FORCE_REF_N = 1000  // legend reference = 1 kN

const COLORS = {
  thrust:   '#4ade80',  // green
  rudder:   '#60a5fa',  // blue
  netForce: '#f59e0b',  // amber
  moment:   '#f472b6',  // pink
  pivot:    '#ffffff',  // white
  drag:     '#94a3b8',  // slate
  walk:     '#a78bfa',  // violet
  contact:  '#cbd5e1',  // light slate
  wind:     '#22d3ee',  // cyan
}

export interface OverlayOptions {
  showHullDrag: boolean
  showContact: boolean
}

// Colour-code line tension: green (light) → amber → red (heavy).
export function tensionColor(tensionN: number): string {
  if (tensionN < 2000) return '#4ade80'
  if (tensionN < 8000) return '#f59e0b'
  return '#ef4444'
}

function bodyToWorld(b: RigidBody, p: Vec2): Vec2 {
  const cos = Math.cos(b.heading)
  const sin = Math.sin(b.heading)
  return { x: b.x + p.x * cos - p.y * sin, y: b.y + p.x * sin + p.y * cos }
}

function bodyVecToWorld(b: RigidBody, fx: number, fy: number): Vec2 {
  const cos = Math.cos(b.heading)
  const sin = Math.sin(b.heading)
  return { x: fx * cos - fy * sin, y: fx * sin + fy * cos }
}

function drawArrow(
  ctx: CanvasRenderingContext2D,
  x0: number, y0: number, x1: number, y1: number,
  color: string, width = 2,
) {
  const dx = x1 - x0, dy = y1 - y0
  const len = Math.hypot(dx, dy)
  if (len < 0.5) return
  ctx.strokeStyle = color
  ctx.fillStyle = color
  ctx.lineWidth = width
  ctx.beginPath()
  ctx.moveTo(x0, y0)
  ctx.lineTo(x1, y1)
  ctx.stroke()
  const a = Math.atan2(dy, dx)
  const head = Math.min(9, len * 0.4)
  ctx.beginPath()
  ctx.moveTo(x1, y1)
  ctx.lineTo(x1 - head * Math.cos(a - 0.4), y1 - head * Math.sin(a - 0.4))
  ctx.lineTo(x1 - head * Math.cos(a + 0.4), y1 - head * Math.sin(a + 0.4))
  ctx.closePath()
  ctx.fill()
}

// Draw a world-frame force (N) at a world point as a screen-space arrow.
function drawWorldForce(
  ctx: CanvasRenderingContext2D, cam: Camera,
  point: Vec2, fx: number, fy: number,
  color: string, originX: number, originY: number, width = 2,
) {
  const [px, py] = worldToScreen(cam, originX, originY, point.x, point.y)
  // Force px length is zoom-independent; world y is up so screen y is flipped.
  drawArrow(ctx, px, py, px + fx * FORCE_PX_PER_N, py - fy * FORCE_PX_PER_N, color, width)
}

// Draw a body-frame force at a body point.
function drawBodyForce(
  ctx: CanvasRenderingContext2D, cam: Camera, body: RigidBody,
  point: Vec2, fx: number, fy: number,
  color: string, originX: number, originY: number, width = 2,
) {
  const wPoint = bodyToWorld(body, point)
  const wVec = bodyVecToWorld(body, fx, fy)
  drawWorldForce(ctx, cam, wPoint, wVec.x, wVec.y, color, originX, originY, width)
}

// Instantaneous pivot (zero-velocity point) of the rigid body, world frame.
export function pivotPoint(b: RigidBody): Vec2 | null {
  const w = b.yawRate
  if (Math.abs(w) < 1e-3) return null
  return { x: b.x - b.vy / w, y: b.y + b.vx / w }
}

export function drawOverlay(
  ctx: CanvasRenderingContext2D,
  cam: Camera,
  body: RigidBody,
  params: BoatParams,
  components: ForceComponent[],   // body-frame propulsion/steering
  worldForces: WorldForce[],      // lines + contact + wind, world frame
  opts: OverlayOptions,
  originX: number,
  originY: number,
) {
  // ── Propulsion arrows: thrust, rudder, prop-walk ──────────────────────────
  for (const c of components) {
    const color = c.label === 'thrust' ? COLORS.thrust
      : c.label === 'rudder' ? COLORS.rudder
      : COLORS.walk
    drawBodyForce(ctx, cam, body, c.point, c.fx, c.fy, color, originX, originY, 2)
  }

  // ── Lines / wind / contact arrows (world frame) ───────────────────────────
  for (const f of worldForces) {
    if (f.label === 'contact' && !opts.showContact) continue
    const mag = Math.hypot(f.fx, f.fy)
    const color = f.label === 'wind' ? COLORS.wind
      : f.label === 'contact' ? COLORS.contact
      : tensionColor(mag)  // lines
    drawWorldForce(ctx, cam, f.point, f.fx, f.fy, color, originX, originY, f.label === 'line' ? 2.5 : 2)
  }

  // ── Hull drag (toggle) ────────────────────────────────────────────────────
  if (opts.showHullDrag) {
    const drag = computeHullDrag(body, params)
    drawBodyForce(ctx, cam, body, { x: 0, y: 0 }, drag.fx, drag.fy, COLORS.drag, originX, originY, 2)
  }

  // ── Net force at the CoM (everything that's currently shown) ───────────────
  const propNet = netForces(components)
  const propW = bodyVecToWorld(body, propNet.fx, propNet.fy)
  let netX = propW.x
  let netY = propW.y
  if (opts.showHullDrag) {
    const drag = computeHullDrag(body, params)
    const dW = bodyVecToWorld(body, drag.fx, drag.fy)
    netX += dW.x; netY += dW.y
  }
  let netMz = propNet.mz + (opts.showHullDrag ? computeHullDrag(body, params).mz : 0)
  for (const f of worldForces) {
    if (f.label === 'contact' && !opts.showContact) continue
    netX += f.fx; netY += f.fy
    netMz += (f.point.x - body.x) * f.fy - (f.point.y - body.y) * f.fx
  }
  drawWorldForce(ctx, cam, { x: body.x, y: body.y }, netX, netY, COLORS.netForce, originX, originY, 3)

  // ── Net yaw-moment curved arrow ───────────────────────────────────────────
  drawMomentArc(ctx, cam, body, netMz, originX, originY)

  // ── Instantaneous pivot-point marker ──────────────────────────────────────
  const pivot = pivotPoint(body)
  if (pivot) {
    const [cx, cy] = worldToScreen(cam, originX, originY, pivot.x, pivot.y)
    ctx.strokeStyle = COLORS.pivot
    ctx.lineWidth = 1.5
    ctx.beginPath(); ctx.arc(cx, cy, 6, 0, Math.PI * 2); ctx.stroke()
    ctx.beginPath()
    ctx.moveTo(cx - 9, cy); ctx.lineTo(cx + 9, cy)
    ctx.moveTo(cx, cy - 9); ctx.lineTo(cx, cy + 9)
    ctx.stroke()
  }

  drawLegend(ctx, opts)
}

function drawMomentArc(
  ctx: CanvasRenderingContext2D, cam: Camera,
  body: RigidBody, mz: number, originX: number, originY: number,
) {
  if (Math.abs(mz) < 1) return
  const [cx, cy] = worldToScreen(cam, originX, originY, body.x, body.y)
  const r = 22
  const sweep = Math.min(Math.PI * 1.5, (Math.abs(mz) / 8000) * Math.PI)
  const ccw = mz > 0
  const start = -Math.PI / 2
  const end = start + (ccw ? -sweep : sweep)
  ctx.strokeStyle = COLORS.moment
  ctx.lineWidth = 2.5
  ctx.beginPath()
  ctx.arc(cx, cy, r, start, end, ccw)
  ctx.stroke()
  const tangent = end + (ccw ? -Math.PI / 2 : Math.PI / 2)
  const ax = cx + r * Math.cos(end)
  const ay = cy + r * Math.sin(end)
  const h = 7
  ctx.fillStyle = COLORS.moment
  ctx.beginPath()
  ctx.moveTo(ax, ay)
  ctx.lineTo(ax - h * Math.cos(tangent - 0.4), ay - h * Math.sin(tangent - 0.4))
  ctx.lineTo(ax - h * Math.cos(tangent + 0.4), ay - h * Math.sin(tangent + 0.4))
  ctx.closePath()
  ctx.fill()
}

// The pivot-point label gets the crosshair glyph (circle + cross) instead of a
// colour swatch, so the legend visibly matches the moving marker on the scene.
const PIVOT_LABEL = 'pivot point'

// Draw the pivot crosshair glyph centred at (cx, cy) — same shape as the
// on-scene marker, scaled down for the legend swatch.
function drawPivotGlyph(ctx: CanvasRenderingContext2D, cx: number, cy: number) {
  ctx.strokeStyle = COLORS.pivot
  ctx.lineWidth = 1.5
  ctx.beginPath(); ctx.arc(cx, cy, 4, 0, Math.PI * 2); ctx.stroke()
  ctx.beginPath()
  ctx.moveTo(cx - 6, cy); ctx.lineTo(cx + 6, cy)
  ctx.moveTo(cx, cy - 6); ctx.lineTo(cx, cy + 6)
  ctx.stroke()
}

export function drawLegend(ctx: CanvasRenderingContext2D, opts: OverlayOptions) {
  const x = ctx.canvas.width - 170
  let y = 18
  ctx.font = '13px monospace'
  ctx.textBaseline = 'middle'

  ctx.fillStyle = '#c8d8e8'
  ctx.fillText('force scale:', x, y)
  drawArrow(ctx, x + 84, y, x + 84 + FORCE_REF_N * FORCE_PX_PER_N, y, '#c8d8e8', 2)
  ctx.fillText('1 kN', x + 90 + FORCE_REF_N * FORCE_PX_PER_N, y)

  const items: [string, string][] = [
    ['thrust', COLORS.thrust],
    ['rudder', COLORS.rudder],
    ['line tension', '#f59e0b'],
    ['wind', COLORS.wind],
    ['net force', COLORS.netForce],
    ['yaw moment', COLORS.moment],
    [PIVOT_LABEL, COLORS.pivot],
  ]
  if (opts.showHullDrag) items.push(['hull drag', COLORS.drag])
  if (opts.showContact) items.push(['contact', COLORS.contact])
  for (const [label, color] of items) {
    y += 18
    if (label === PIVOT_LABEL) {
      drawPivotGlyph(ctx, x + 5, y)
    } else {
      ctx.fillStyle = color
      ctx.fillRect(x, y - 4, 10, 8)
    }
    ctx.fillStyle = '#c8d8e8'
    ctx.fillText(label, x + 16, y)
  }
}
