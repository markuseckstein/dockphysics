import type { RigidBody } from '../physics/integrator'
import type { BoatGeometry } from '../scene/boat'
import type { Segment } from '../scene/dock'
import { SCALE_PX_PER_M } from '../units'

const WATER_COLOR  = '#0d2440'
const QUAY_COLOR   = '#4a5568'
const QUAY_EDGE    = '#6b7280'
const HULL_FILL    = '#c8aa6e'
const HULL_STROKE  = '#e8c87e'
const CLEAT_COLOR  = '#e8e8e8'

export function setupCanvas(canvas: HTMLCanvasElement, width: number, height: number) {
  canvas.width  = width
  canvas.height = height
}

// Convert world metres to canvas pixels.
// The scene origin is placed at the canvas bottom-left (Y axis flipped).
export function worldToCanvas(
  wx: number, wy: number,
  originX: number, originY: number,
): [number, number] {
  return [originX + wx * SCALE_PX_PER_M, originY - wy * SCALE_PX_PER_M]
}

export function drawScene(
  ctx: CanvasRenderingContext2D,
  segments: Segment[],
  boat: BoatGeometry,
  body: RigidBody,
  originX: number,
  originY: number,
) {
  const { width, height } = ctx.canvas

  // Water background
  ctx.fillStyle = WATER_COLOR
  ctx.fillRect(0, 0, width, height)

  const s = SCALE_PX_PER_M
  const toC = (wx: number, wy: number): [number, number] =>
    worldToCanvas(wx, wy, originX, originY)

  // Draw dock quay walls (thick lines with filled quay region)
  ctx.lineWidth = 3
  for (const seg of segments) {
    const [x1c, y1c] = toC(seg.x1, seg.y1)
    const [x2c, y2c] = toC(seg.x2, seg.y2)

    // Draw quay as a thick band on the outward-normal side
    const wallThickness = 2.5 * s  // 2.5 m thick quay
    const nx = seg.nx
    const ny = seg.ny

    // Four corners of the quay band (in canvas space, y flipped)
    const dx = (x2c - x1c)
    const dy = (y2c - y1c)
    const len = Math.hypot(dx, dy)
    // perpendicular in canvas space: rotate 90° CW because y is flipped
    const perpX = -dy / len * wallThickness * s / s  // already in pixels
    const perpY =  dx / len * wallThickness * s / s

    // normal in world → canvas: nx flips y
    const nxC =  nx * s * (wallThickness / s)
    const nyC = -ny * s * (wallThickness / s)

    ctx.beginPath()
    ctx.moveTo(x1c, y1c)
    ctx.lineTo(x2c, y2c)
    ctx.lineTo(x2c - nxC, y2c - nyC)
    ctx.lineTo(x1c - nxC, y1c - nyC)
    ctx.closePath()
    ctx.fillStyle = QUAY_COLOR
    ctx.fill()

    // Edge line facing the water
    ctx.beginPath()
    ctx.moveTo(x1c, y1c)
    ctx.lineTo(x2c, y2c)
    ctx.strokeStyle = QUAY_EDGE
    ctx.lineWidth = 2
    ctx.stroke()

    // Suppress unused variable warnings
    void perpX; void perpY
  }

  // Draw boat
  const cos = Math.cos(body.heading)
  const sin = Math.sin(body.heading)
  const halfL = boat.lengthM / 2
  const halfB = boat.beamM  / 2

  // Boat hull as a simple rectangle with pointed bow
  // Corners in body frame
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
    // Rotate body → world
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

  // Cleats
  for (const cleat of boat.cleats) {
    const wx = body.x + cleat.x * cos - cleat.y * sin
    const wy = body.y + cleat.x * sin + cleat.y * cos
    const [cx, cy] = toC(wx, wy)
    ctx.beginPath()
    ctx.arc(cx, cy, 3, 0, Math.PI * 2)
    ctx.fillStyle = CLEAT_COLOR
    ctx.fill()
  }

  // Heading arrow from CoM
  const arrowLen = 4 * s  // 4 m
  const [bx, by] = toC(body.x, body.y)
  const [ax, ay] = toC(body.x + arrowLen / s * cos, body.y + arrowLen / s * sin)
  ctx.beginPath()
  ctx.moveTo(bx, by)
  ctx.lineTo(ax, ay)
  ctx.strokeStyle = '#ef4444'
  ctx.lineWidth = 2
  ctx.stroke()
  // Arrowhead
  const headLen = 0.8 * s
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
  ctx.font = '11px monospace'
  ctx.fillText(`x: ${body.x.toFixed(1)} m  y: ${body.y.toFixed(1)} m  hdg: ${(body.heading * 180 / Math.PI).toFixed(1)}°`, 12, 18)
}
