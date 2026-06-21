import { makeBody, step } from './physics/integrator'
import type { RigidBody, BoatParams } from './physics/integrator'
import { boatFromLength } from './scene/boat'
import { lBerth } from './scene/dock'
import { setupCanvas, drawScene } from './render/canvas'
import { drawOverlay } from './render/overlay'
import {
  controlForces, netForces, propulsionTuning, defaultStandstillAuthority,
} from './physics/propulsion'
import type { Controls, RudderConfig } from './physics/propulsion'
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
const tuning = propulsionTuning(geo)

// L-berth: quay along +x, finger at x=0 extending in +y
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

// ── Control state ────────────────────────────────────────────────────────────

const controls: Controls = {
  throttle: 0,
  helm: 0,
  rudderConfig: 'single',
  standstillAuthority: defaultStandstillAuthority('single'),
}
let showHullDrag = false

// ── Canvas ─────────────────────────────────────────────────────────────────

const canvas = document.getElementById('canvas') as HTMLCanvasElement
const W = 900
const H = 500
setupCanvas(canvas, W, H)
const ctx = canvas.getContext('2d')!

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
    const components = controlForces(body, geo, controls, tuning)
    body = step(body, params, netForces(components), FIXED_DT)
    accumulator -= FIXED_DT
  }

  drawScene(ctx, segments, geo, body, ORIGIN_X, ORIGIN_Y)
  // Recompute components for the current frame so overlay matches what's drawn.
  const components = controlForces(body, geo, controls, tuning)
  drawOverlay(ctx, body, params, components, { showHullDrag }, ORIGIN_X, ORIGIN_Y)
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

// ── Control wiring ───────────────────────────────────────────────────────────

const throttleEl   = document.getElementById('throttle')      as HTMLInputElement
const throttleVal  = document.getElementById('throttle-val')  as HTMLSpanElement
const helmEl       = document.getElementById('helm')          as HTMLInputElement
const configEl     = document.getElementById('rudder-config') as HTMLSelectElement
const standstillEl = document.getElementById('standstill')    as HTMLInputElement
const standstillVal = document.getElementById('standstill-val') as HTMLSpanElement
const dragEl       = document.getElementById('toggle-drag')   as HTMLInputElement

function clamp(v: number, lo: number, hi: number) { return Math.max(lo, Math.min(hi, v)) }

function syncThrottle() {
  controls.throttle = parseFloat(throttleEl.value)
  throttleVal.textContent = `${Math.round(controls.throttle * 100)}%`
}
function syncHelm() {
  controls.helm = parseFloat(helmEl.value)
}
function syncStandstill() {
  controls.standstillAuthority = parseFloat(standstillEl.value)
  standstillVal.textContent = controls.standstillAuthority.toFixed(2)
}

// Rudder config: twin locks standstill authority to zero and disables the slider.
function applyConfig(config: RudderConfig) {
  controls.rudderConfig = config
  if (config === 'twin') {
    standstillEl.disabled = true
    standstillEl.value = '0'
  } else {
    standstillEl.disabled = false
    standstillEl.value = String(defaultStandstillAuthority('single'))
  }
  syncStandstill()
}

throttleEl.addEventListener('input', syncThrottle)
helmEl.addEventListener('input', syncHelm)
standstillEl.addEventListener('input', syncStandstill)
configEl.addEventListener('change', () => applyConfig(configEl.value as RudderConfig))
dragEl.addEventListener('change', () => { showHullDrag = dragEl.checked })

document.getElementById('btn-centre')!.addEventListener('click', () => {
  helmEl.value = '0'
  syncHelm()
})

document.getElementById('btn-reset')!.addEventListener('click', () => {
  body = makeInitialBody()
  throttleEl.value = '0'
  helmEl.value = '0'
  syncThrottle()
  syncHelm()
})

// Keyboard: ↑/↓ throttle (holds), ←/→ helm (holds), C centres helm.
window.addEventListener('keydown', (e) => {
  switch (e.key) {
    case 'ArrowUp':    throttleEl.value = String(clamp(controls.throttle + 0.1, -1, 1)); syncThrottle(); e.preventDefault(); break
    case 'ArrowDown':  throttleEl.value = String(clamp(controls.throttle - 0.1, -1, 1)); syncThrottle(); e.preventDefault(); break
    case 'ArrowLeft':  helmEl.value = String(clamp(controls.helm - 0.1, -1, 1)); syncHelm(); e.preventDefault(); break
    case 'ArrowRight': helmEl.value = String(clamp(controls.helm + 0.1, -1, 1)); syncHelm(); e.preventDefault(); break
    case 'c': case 'C': helmEl.value = '0'; syncHelm(); break
  }
})

// Initialise UI to defaults.
syncThrottle()
syncHelm()
applyConfig('single')

// ── Scale label ──────────────────────────────────────────────────────────────

const info = document.getElementById('info')
if (info) {
  const scaleM = 10  // 10 m ruler
  const scalePx = scaleM * SCALE_PX_PER_M
  info.insertAdjacentHTML('afterend',
    `<span style="color:#3a6a9f;font-size:11px">scale: ${scalePx}px = ${scaleM} m &nbsp;| boat: ${BOAT_LENGTH_FT} ft (${ftToM(BOAT_LENGTH_FT).toFixed(1)} m)</span>`)
}

// ── Start ──────────────────────────────────────────────────────────────────

requestAnimationFrame(tick)
