import { makeBody, step } from './physics/integrator'
import type { RigidBody, BoatParams } from './physics/integrator'
import { boatFromLength } from './scene/boat'
import { lBerth } from './scene/dock'
import { setupCanvas, drawScene } from './render/canvas'
import { ftToM, msToKnots, SCALE_PX_PER_M } from './units'

// ── Scene setup ────────────────────────────────────────────────────────────

const BOAT_LENGTH_FT = 40
const geo = boatFromLength(BOAT_LENGTH_FT)
const params: BoatParams = {
  mass:            geo.massKg,
  addedMassSway:   geo.addedMassSway,
  inertia:         geo.inertiaYaw,
  addedInertiaYaw: geo.addedInertiaYaw,
  dampSurge:       geo.dampSurge,
  dampSway:        geo.dampSway,
  dampYaw:         geo.dampYaw,
}

// L-berth: quay along +x, finger at x=0 extending in +y
// Size the quay generously around the boat
const QUAY_LEN   = geo.lengthM * 2.5
const FINGER_LEN = geo.beamM   * 4
const segments = lBerth(QUAY_LEN, FINGER_LEN)

// Initial boat position: lying alongside the main quay (y = beamM/2 + small gap)
const BERTH_GAP = 0.5  // m gap from quay
function makeInitialBody(): RigidBody {
  return makeBody(
    geo.lengthM * 0.8,          // x: part-way along the quay
    geo.beamM / 2 + BERTH_GAP, // y: just off the quay
    0,                          // heading: parallel to quay
  )
}

let body = makeInitialBody()

// ── Canvas ─────────────────────────────────────────────────────────────────

const canvas = document.getElementById('canvas') as HTMLCanvasElement
const W = 900
const H = 500
setupCanvas(canvas, W, H)
const ctx = canvas.getContext('2d')!

// Scene origin in canvas coords: 30 px from left, 3/4 down
const ORIGIN_X = 50
const ORIGIN_Y = H - 80

// ── Fixed-timestep loop ────────────────────────────────────────────────────

const FIXED_DT = 1 / 60  // s
const MAX_ACCUMULATOR = 0.1  // cap at 6 frames to prevent spiral of death

let lastTime: number | null = null
let accumulator = 0

function tick(now: number) {
  const nowS = now / 1000
  if (lastTime === null) { lastTime = nowS }
  const rawDt = Math.min(nowS - lastTime, MAX_ACCUMULATOR)
  lastTime = nowS
  accumulator += rawDt

  while (accumulator >= FIXED_DT) {
    body = step(body, params, { fx: 0, fy: 0, mz: 0 }, FIXED_DT)
    accumulator -= FIXED_DT
  }

  drawScene(ctx, segments, geo, body, ORIGIN_X, ORIGIN_Y)
  updateHUD()
  requestAnimationFrame(tick)
}

function updateHUD() {
  const speed = Math.hypot(body.vx, body.vy)
  const info = document.getElementById('info')
  if (info) {
    info.textContent =
      `vel: ${msToKnots(speed).toFixed(2)} kn  yawRate: ${(body.yawRate * 180 / Math.PI).toFixed(1)} °/s`
  }
}

// ── Debug impulse buttons ──────────────────────────────────────────────────

// Impulse magnitudes: force × dt to give a velocity kick
const SURGE_IMPULSE = params.mass * 0.8           // N·s → ~0.8 m/s surge
const SWAY_IMPULSE  = (params.mass + params.addedMassSway) * 0.6
const YAW_IMPULSE   = (params.inertia + params.addedInertiaYaw) * 0.4  // N·m·s

function applyImpulse(bodySurge: number, bodySway: number, bodyYaw: number) {
  const cos = Math.cos(body.heading)
  const sin = Math.sin(body.heading)
  body = {
    ...body,
    // Convert body-frame impulse to velocity change, then to world velocities
    vx: body.vx + (bodySurge * cos - bodySway * sin) / params.mass,
    vy: body.vy + (bodySurge * sin + bodySway * cos) / params.mass,
    yawRate: body.yawRate + bodyYaw / (params.inertia + params.addedInertiaYaw),
  }
}

document.getElementById('btn-surge')!.addEventListener('click', () =>
  applyImpulse(SURGE_IMPULSE, 0, 0))

document.getElementById('btn-sway')!.addEventListener('click', () =>
  applyImpulse(0, SWAY_IMPULSE, 0))

document.getElementById('btn-yaw')!.addEventListener('click', () =>
  applyImpulse(0, 0, YAW_IMPULSE))

document.getElementById('btn-reset')!.addEventListener('click', () => {
  body = makeInitialBody()
})

// ── Canvas info label ──────────────────────────────────────────────────────

// Append a scale ruler to the footer
const info = document.getElementById('info')
if (info) {
  const scaleM = 10  // 10 m ruler
  const scalePx = scaleM * SCALE_PX_PER_M
  info.insertAdjacentHTML('afterend',
    `<span style="color:#3a6a9f;font-size:11px">scale: ${scalePx}px = ${scaleM} m &nbsp;| boat: ${BOAT_LENGTH_FT} ft (${ftToM(BOAT_LENGTH_FT).toFixed(1)} m)</span>`)
}

// ── Start ──────────────────────────────────────────────────────────────────

requestAnimationFrame(tick)
