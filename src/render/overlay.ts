import type { RigidBody, Forces, BoatParams } from '../physics/integrator'
import { computeHullDrag } from '../physics/integrator'
import type { Vec2 } from '../scene/boat'
import type { ForceComponent } from '../physics/propulsion'
import { netForces } from '../physics/propulsion'
import { worldToCanvas } from './canvas'
import { SCALE_PX_PER_M } from '../units'

// Consistent on-screen force scale: pixels of arrow per newton of force.
// Tuned so a ~1 kN force draws ~30 px.
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
}

export interface OverlayOptions {
  showHullDrag: boolean
}

// Rotate a body-frame point to world coordinates.
function bodyToWorld(b: RigidBody, p: Vec2): Vec2 {
  const cos = Math.cos(b.heading)
  const sin = Math.sin(b.heading)
  return { x: b.x + p.x * cos - p.y * sin, y: b.y + p.x * sin + p.y * cos }
}

// Rotate a body-frame vector (no translation) to world.
function bodyVecToWorld(b: RigidBody, fx: number, fy: number): Vec2 {
  const cos = Math.cos(b.heading)
  const sin = Math.sin(b.heading)
  return { x: fx * cos - fy * sin, y: fx * sin + fy * cos }
}

// Draw an arrow in canvas space from (x0,y0) to (x1,y1).
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

// Draw a single body-frame force as a world-space arrow from its application point.
function drawForceVector(
  ctx: CanvasRenderingContext2D,
  body: RigidBody, point: Vec2, fx: number, fy: number,
  color: string, originX: number, originY: number, width = 2,
) {
  const wPoint = bodyToWorld(body, point)
  const wVec = bodyVecToWorld(body, fx, fy)
  const [px, py] = worldToCanvas(wPoint.x, wPoint.y, originX, originY)
  const [ex, ey] = worldToCanvas(
    wPoint.x + wVec.x * FORCE_PX_PER_N / SCALE_PX_PER_M,
    wPoint.y + wVec.y * FORCE_PX_PER_N / SCALE_PX_PER_M,
    originX, originY,
  )
  drawArrow(ctx, px, py, ex, ey, color, width)
}

// Instantaneous pivot (zero-velocity point) of the rigid body, world frame.
// For velocity v at CoM and yaw rate ω: offset r = (-vy/ω, vx/ω).
export function pivotPoint(b: RigidBody): Vec2 | null {
  const w = b.yawRate
  if (Math.abs(w) < 1e-3) return null  // ~pure translation, pivot at infinity
  return { x: b.x - b.vy / w, y: b.y + b.vx / w }
}

// Draw the full teaching overlay on top of the scene.
export function drawOverlay(
  ctx: CanvasRenderingContext2D,
  body: RigidBody,
  params: BoatParams,
  components: ForceComponent[],
  opts: OverlayOptions,
  originX: number,
  originY: number,
) {
  // ── Per-source arrows: thrust, rudder, prop-walk ──────────────────────────
  for (const c of components) {
    const color = c.label === 'thrust' ? COLORS.thrust
      : c.label === 'rudder' ? COLORS.rudder
      : COLORS.walk
    drawForceVector(ctx, body, c.point, c.fx, c.fy, color, originX, originY, 2)
  }

  // ── Hull drag arrows (toggle, off by default) ─────────────────────────────
  if (opts.showHullDrag) {
    const drag = computeHullDrag(body, params)
    drawForceVector(ctx, body, { x: 0, y: 0 }, drag.fx, drag.fy, COLORS.drag, originX, originY, 2)
  }

  // ── Net force at the CoM ──────────────────────────────────────────────────
  const propNet = netForces(components)
  const dragNet = opts.showHullDrag ? computeHullDrag(body, params) : { fx: 0, fy: 0, mz: 0 }
  const net: Forces = {
    fx: propNet.fx + dragNet.fx,
    fy: propNet.fy + dragNet.fy,
    mz: propNet.mz + dragNet.mz,
  }
  drawForceVector(ctx, body, { x: 0, y: 0 }, net.fx, net.fy, COLORS.netForce, originX, originY, 3)

  // ── Net yaw-moment curved arrow around the CoM ────────────────────────────
  drawMomentArc(ctx, body, net.mz, originX, originY)

  // ── Instantaneous pivot-point marker ──────────────────────────────────────
  const pivot = pivotPoint(body)
  if (pivot) {
    const [cx, cy] = worldToCanvas(pivot.x, pivot.y, originX, originY)
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

// Curved arrow whose sweep length grows with |moment|; arrowhead shows CCW/CW.
function drawMomentArc(
  ctx: CanvasRenderingContext2D,
  body: RigidBody, mz: number, originX: number, originY: number,
) {
  if (Math.abs(mz) < 1) return
  const [cx, cy] = worldToCanvas(body.x, body.y, originX, originY)
  const r = 22
  // Sweep proportional to magnitude, clamped to most of a circle.
  const sweep = Math.min(Math.PI * 1.5, (Math.abs(mz) / 8000) * Math.PI)
  const ccw = mz > 0                 // +mz = CCW in world; canvas y is flipped
  const start = -Math.PI / 2
  // In flipped canvas space, CCW world rotation draws clockwise on screen.
  const end = start + (ccw ? -sweep : sweep)
  ctx.strokeStyle = COLORS.moment
  ctx.lineWidth = 2.5
  ctx.beginPath()
  ctx.arc(cx, cy, r, start, end, ccw)
  ctx.stroke()
  // Arrowhead at the arc end, tangent direction.
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

// Force-scale reference + legend in the top-right corner.
function drawLegend(ctx: CanvasRenderingContext2D, opts: OverlayOptions) {
  const x = ctx.canvas.width - 150
  let y = 16
  ctx.font = '11px monospace'
  ctx.textBaseline = 'middle'

  // Scale reference arrow = 1 kN
  ctx.fillStyle = '#c8d8e8'
  ctx.fillText('force scale:', x, y)
  drawArrow(ctx, x + 72, y, x + 72 + FORCE_REF_N * FORCE_PX_PER_N, y, '#c8d8e8', 2)
  ctx.fillText('1 kN', x + 78 + FORCE_REF_N * FORCE_PX_PER_N, y)

  const items: [string, string][] = [
    ['thrust', COLORS.thrust],
    ['rudder', COLORS.rudder],
    ['net force', COLORS.netForce],
    ['yaw moment', COLORS.moment],
    ['pivot point', COLORS.pivot],
  ]
  if (opts.showHullDrag) items.push(['hull drag', COLORS.drag])
  for (const [label, color] of items) {
    y += 15
    ctx.fillStyle = color
    ctx.fillRect(x, y - 4, 10, 8)
    ctx.fillStyle = '#c8d8e8'
    ctx.fillText(label, x + 16, y)
  }
}
