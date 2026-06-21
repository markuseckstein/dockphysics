import type { RigidBody } from '../physics/integrator'
import type { BoatGeometry } from '../scene/boat'
import type { Segment } from '../scene/dock'
import type { Line } from '../physics/lines'
import { cleatWorld, lineTension } from '../physics/lines'
import type { Camera } from './camera'
import { worldToScreen } from './camera'
import { SCALE_PX_PER_M } from '../units'

const WATER_COLOR  = '#0d2440'
const QUAY_COLOR   = '#4a5568'
const QUAY_EDGE    = '#6b7280'
const HULL_FILL    = '#c8aa6e'
const HULL_STROKE  = '#e8c87e'
const CLEAT_COLOR  = '#e8e8e8'
const CLEAT_SELECT = '#fde047'
const FENDER_COLOR = '#1f2937'
const LINE_TAUT    = '#e2e8f0'
const LINE_SLACK   = '#64748b'

export function setupCanvas(canvas: HTMLCanvasElement, width: number, height: number) {
  canvas.width  = width
  canvas.height = height
}

export function drawScene(
  ctx: CanvasRenderingContext2D,
  cam: Camera,
  segments: Segment[],
  boat: BoatGeometry,
  body: RigidBody,
  lines: Line[],
  selectedCleat: number | null,
  originX: number,
  originY: number,
) {
  const { width, height } = ctx.canvas
  const s = SCALE_PX_PER_M * cam.zoom
  const toC = (wx: number, wy: number): [number, number] =>
    worldToScreen(cam, originX, originY, wx, wy)

  // Water background
  ctx.fillStyle = WATER_COLOR
  ctx.fillRect(0, 0, width, height)

  // Dock quay walls: thick band on the −normal (solid) side of each edge.
  for (const seg of segments) {
    const [x1c, y1c] = toC(seg.x1, seg.y1)
    const [x2c, y2c] = toC(seg.x2, seg.y2)
    const wallThickness = 2.5  // m
    // Outward normal in world → canvas (y flipped). Solid side is −normal.
    const nxC =  seg.nx * wallThickness * s
    const nyC = -seg.ny * wallThickness * s

    ctx.beginPath()
    ctx.moveTo(x1c, y1c)
    ctx.lineTo(x2c, y2c)
    ctx.lineTo(x2c - nxC, y2c - nyC)
    ctx.lineTo(x1c - nxC, y1c - nyC)
    ctx.closePath()
    ctx.fillStyle = QUAY_COLOR
    ctx.fill()

    ctx.beginPath()
    ctx.moveTo(x1c, y1c)
    ctx.lineTo(x2c, y2c)
    ctx.strokeStyle = QUAY_EDGE
    ctx.lineWidth = 2
    ctx.stroke()
  }

  // Docklines (drawn under the hull so cleats sit on top).
  for (const line of lines) {
    const cw = cleatWorld(body, boat, line.cleatIndex)
    const taut = lineTension(body, boat, line) > 0
    const [c0x, c0y] = toC(cw.x, cw.y)
    const [d0x, d0y] = toC(line.dockPoint.x, line.dockPoint.y)
    ctx.strokeStyle = taut ? LINE_TAUT : LINE_SLACK
    ctx.lineWidth = taut ? 2 : 1.5
    ctx.beginPath()
    ctx.moveTo(c0x, c0y)
    if (taut) {
      ctx.lineTo(d0x, d0y)
    } else {
      // Slack: sag the rope so it reads as limp.
      const sag = 12
      ctx.quadraticCurveTo((c0x + d0x) / 2, (c0y + d0y) / 2 + sag, d0x, d0y)
    }
    ctx.stroke()
    // Dock attach point.
    ctx.fillStyle = QUAY_EDGE
    ctx.beginPath(); ctx.arc(d0x, d0y, 3, 0, Math.PI * 2); ctx.fill()
  }

  // Boat hull
  const cos = Math.cos(body.heading)
  const sin = Math.sin(body.heading)
  const halfL = boat.lengthM / 2
  const halfB = boat.beamM  / 2
  const hullPts: [number, number][] = [
    [ halfL,         halfB * 0.4 ],
    [ halfL * 0.85,  halfB ],
    [-halfL,         halfB ],
    [-halfL,        -halfB ],
    [ halfL * 0.85, -halfB ],
    [ halfL,        -halfB * 0.4 ],
  ]
  ctx.beginPath()
  for (let i = 0; i < hullPts.length; i++) {
    const [bx, by] = hullPts[i]
    const wx = body.x + bx * cos - by * sin
    const wy = body.y + bx * sin + by * cos
    const [cx, cy] = toC(wx, wy)
    if (i === 0) ctx.moveTo(cx, cy)
    else ctx.lineTo(cx, cy)
  }
  ctx.closePath()
  ctx.fillStyle   = HULL_FILL
  ctx.fill()
  ctx.strokeStyle = HULL_STROKE
  ctx.lineWidth   = 1.5
  ctx.stroke()

  // Fenders
  for (const f of boat.fenders) {
    const wx = body.x + f.x * cos - f.y * sin
    const wy = body.y + f.x * sin + f.y * cos
    const [cx, cy] = toC(wx, wy)
    ctx.beginPath()
    ctx.arc(cx, cy, 2.5, 0, Math.PI * 2)
    ctx.fillStyle = FENDER_COLOR
    ctx.fill()
  }

  // Cleats (selected one highlighted to guide the make-fast flow).
  for (let i = 0; i < boat.cleats.length; i++) {
    const cleat = boat.cleats[i]
    const wx = body.x + cleat.x * cos - cleat.y * sin
    const wy = body.y + cleat.x * sin + cleat.y * cos
    const [cx, cy] = toC(wx, wy)
    const selected = i === selectedCleat
    ctx.beginPath()
    ctx.arc(cx, cy, selected ? 5 : 3, 0, Math.PI * 2)
    ctx.fillStyle = selected ? CLEAT_SELECT : CLEAT_COLOR
    ctx.fill()
  }

  // Heading arrow from CoM
  const arrowLen = 4  // m
  const [bx, by] = toC(body.x, body.y)
  const [ax, ay] = toC(body.x + arrowLen * cos, body.y + arrowLen * sin)
  ctx.beginPath()
  ctx.moveTo(bx, by)
  ctx.lineTo(ax, ay)
  ctx.strokeStyle = '#ef4444'
  ctx.lineWidth = 2
  ctx.stroke()
  const headLen = 8
  const angle = Math.atan2(ay - by, ax - bx)
  ctx.beginPath()
  ctx.moveTo(ax, ay)
  ctx.lineTo(ax - headLen * Math.cos(angle - 0.4), ay - headLen * Math.sin(angle - 0.4))
  ctx.lineTo(ax - headLen * Math.cos(angle + 0.4), ay - headLen * Math.sin(angle + 0.4))
  ctx.closePath()
  ctx.fillStyle = '#ef4444'
  ctx.fill()

  // HUD text
  ctx.fillStyle = '#7aa7c8'
  ctx.font = '13px monospace'
  ctx.fillText(`x: ${body.x.toFixed(1)} m  y: ${body.y.toFixed(1)} m  hdg: ${(body.heading * 180 / Math.PI).toFixed(1)}°`, 12, 18)
}

// Cleat hit-test in screen space: returns the index of the nearest cleat within
// `radius` px of (sx, sy), or null.
export function pickCleat(
  cam: Camera, boat: BoatGeometry, body: RigidBody,
  sx: number, sy: number, originX: number, originY: number, radius = 10,
): number | null {
  const cos = Math.cos(body.heading)
  const sin = Math.sin(body.heading)
  let best: number | null = null
  let bestD = radius
  for (let i = 0; i < boat.cleats.length; i++) {
    const cleat = boat.cleats[i]
    const wx = body.x + cleat.x * cos - cleat.y * sin
    const wy = body.y + cleat.x * sin + cleat.y * cos
    const [cx, cy] = worldToScreen(cam, originX, originY, wx, wy)
    const d = Math.hypot(cx - sx, cy - sy)
    if (d <= bestD) { bestD = d; best = i }
  }
  return best
}
