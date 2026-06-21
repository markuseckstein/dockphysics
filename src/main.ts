import { makeBody, step } from './physics/integrator'
import type { RigidBody, BoatParams } from './physics/integrator'
import { boatFromLength } from './scene/boat'
import type { BoatGeometry } from './scene/boat'
import { lBerth } from './scene/dock'
import type { Segment } from './scene/dock'
import { setupCanvas, drawScene, pickCleat } from './render/canvas'
import { drawOverlay } from './render/overlay'
import { makeCamera, fitCamera, screenToWorld, worldToScreen } from './render/camera'
import type { Camera } from './render/camera'
import {
  controlForces, netForces, propulsionTuning, defaultStandstillAuthority,
} from './physics/propulsion'
import type { Controls, RudderConfig } from './physics/propulsion'
import { worldForcesToBody } from './physics/forces'
import type { WorldForce } from './physics/forces'
import {
  makeLine, lineForce, lineTension, easeLine, takeUpLine,
} from './physics/lines'
import type { Line } from './physics/lines'
import { contactForces, defaultContactTuning } from './physics/contact'
import { windFromSpeedDir, windForce, defaultWindTuning } from './physics/wind'
import { closestPointOnSegment } from './scene/geometry'
import { PRESETS, buildPreset, quayLength, fingerLength } from './scene/presets'
import type { PresetId } from './scene/presets'
import { msToKnots, knotsToMs } from './units'

// ── Mutable simulation state ──────────────────────────────────────────────────

let lengthFt = 40
let geo: BoatGeometry = boatFromLength(lengthFt)
let params: BoatParams = toParams(geo)
let tuning = propulsionTuning(geo)
let segments: Segment[] = lBerth(quayLength(geo), fingerLength(geo))
let body: RigidBody = makeBody(0, 0, 0)
let lines: Line[] = []

let contactTuning = defaultContactTuning(geo)
const windTuning = defaultWindTuning()

const controls: Controls = {
  throttle: 0, helm: 0, rudderConfig: 'single',
  standstillAuthority: defaultStandstillAuthority('single'),
}

let wind = { speed: 0, dirToward: (270 * Math.PI) / 180 }  // m/s, radians toward

let activePreset: PresetId | 'free' = 'free'
let selectedCleat: number | null = null
let selectedLine: number | null = null
let showHullDrag = false
let showContact = false
let paused = false
let slowMo = false

const CLEAT_NAMES = ['bow stbd', 'bow port', 'mid stbd', 'mid port', 'stern stbd', 'stern port']

function toParams(g: BoatGeometry): BoatParams {
  return {
    mass: g.massKg, addedMassSway: g.addedMassSway,
    inertia: g.inertiaYaw, addedInertiaYaw: g.addedInertiaYaw,
    dampSurge: g.dampSurge, dampSway: g.dampSway, dampYaw: g.dampYaw,
  }
}

// ── Canvas / camera ───────────────────────────────────────────────────────────

const canvas = document.getElementById('canvas') as HTMLCanvasElement
const W = 880, H = 520
setupCanvas(canvas, W, H)
const ctx = canvas.getContext('2d')!
const ORIGIN_X = 60
const ORIGIN_Y = H - 90
let cam: Camera = makeCamera()

function sceneBounds() {
  let minX = body.x, minY = body.y, maxX = body.x, maxY = body.y
  for (const s of segments) {
    minX = Math.min(minX, s.x1, s.x2); maxX = Math.max(maxX, s.x1, s.x2)
    minY = Math.min(minY, s.y1, s.y2); maxY = Math.max(maxY, s.y1, s.y2)
  }
  const pad = geo.lengthM
  return { minX: minX - pad, minY: minY - pad, maxX: maxX + pad, maxY: maxY + pad }
}

function fitView() { cam = fitCamera(sceneBounds(), W, H, ORIGIN_X, ORIGIN_Y) }

// ── State assembly ────────────────────────────────────────────────────────────

function rebuildBoat(ft: number) {
  lengthFt = ft
  geo = boatFromLength(ft)
  params = toParams(geo)
  tuning = propulsionTuning(geo)
  contactTuning = defaultContactTuning(geo)
  segments = lBerth(quayLength(geo), fingerLength(geo))
}

function alongsideBody(): RigidBody {
  return makeBody(geo.lengthM * 0.8, geo.beamM / 2 + 0.5, 0)
}

// Apply the active preset (or a neutral free-sail state) at the current length.
function applyActive(resetCamera = true) {
  rebuildBoat(lengthFt)
  if (activePreset === 'free') {
    body = alongsideBody()
    lines = []
    controls.throttle = 0
    controls.helm = 0
    controls.rudderConfig = 'single'
    // wind keeps the user's manual setting in free mode
  } else {
    const s = buildPreset(activePreset, lengthFt)
    body = { ...s.body }
    lines = s.lines.map(l => ({ ...l, dockPoint: { ...l.dockPoint } }))
    controls.throttle = s.controls.throttle ?? 0
    controls.helm = s.controls.helm ?? 0
    controls.rudderConfig = s.controls.rudderConfig ?? 'single'
    controls.standstillAuthority = s.controls.standstillAuthority ?? defaultStandstillAuthority(controls.rudderConfig)
    wind = { speed: s.wind.speed, dirToward: s.wind.dirToward }
  }
  selectedCleat = null
  selectedLine = lines.length ? lines.length - 1 : null
  syncControlsToUI()
  if (resetCamera) fitView()
  persist()
}

// ── Physics step ──────────────────────────────────────────────────────────────

function assembleWorldForces(): WorldForce[] {
  const wf: WorldForce[] = []
  for (const l of lines) {
    const f = lineForce(body, geo, l)
    if (f) wf.push(f)
  }
  for (const c of contactForces(body, geo, segments, contactTuning)) wf.push(c)
  const fw = windForce(body, geo, windFromSpeedDir(wind.speed, wind.dirToward), windTuning)
  if (fw) wf.push(fw)
  return wf
}

function integrate(dt: number) {
  const propNet = netForces(controlForces(body, geo, controls, tuning))
  const worldNet = worldForcesToBody(body, assembleWorldForces())
  body = step(body, params, {
    fx: propNet.fx + worldNet.fx,
    fy: propNet.fy + worldNet.fy,
    mz: propNet.mz + worldNet.mz,
  }, dt)
}

// ── Render loop ───────────────────────────────────────────────────────────────

const FIXED_DT = 1 / 60
const MAX_ACCUMULATOR = 0.1
let lastTime: number | null = null
let accumulator = 0

function tick(now: number) {
  const nowS = now / 1000
  if (lastTime === null) lastTime = nowS
  const rawDt = Math.min(nowS - lastTime, MAX_ACCUMULATOR)
  lastTime = nowS
  const scale = paused ? 0 : (slowMo ? 0.25 : 1)
  accumulator += rawDt * scale
  while (accumulator >= FIXED_DT) {
    integrate(FIXED_DT)
    accumulator -= FIXED_DT
  }

  const components = controlForces(body, geo, controls, tuning)
  const worldForces = assembleWorldForces()
  drawScene(ctx, cam, segments, geo, body, lines, selectedCleat, ORIGIN_X, ORIGIN_Y)
  drawOverlay(ctx, cam, body, params, components, worldForces, { showHullDrag, showContact }, ORIGIN_X, ORIGIN_Y)
  updateHUD()
  updateTensionPanel()
  requestAnimationFrame(tick)
}

function updateHUD() {
  const speed = Math.hypot(body.vx, body.vy)
  const info = document.getElementById('info')
  if (info) {
    info.textContent =
      `vel: ${msToKnots(speed).toFixed(2)} kn  yawRate: ${(body.yawRate * 180 / Math.PI).toFixed(1)} °/s` +
      (paused ? '  [PAUSED]' : '')
  }
}

function updateTensionPanel() {
  const panel = document.getElementById('tensions')
  if (!panel) return
  if (!lines.length) { panel.innerHTML = '<div class="hint" style="margin:0">No lines made fast.</div>'; return }
  const rows = lines.map((l, i) => {
    const tN = lineTension(body, geo, l)
    const kN = tN / 1000
    const kgf = tN / 9.80665
    const sel = i === selectedLine ? ' style="color:#fde047"' : ''
    return `<div class="tension-row"${sel}><span class="name">${CLEAT_NAMES[l.cleatIndex]}</span>` +
           `<span>${kN.toFixed(2)} kN · ${Math.round(kgf)} kgf</span></div>`
  })
  panel.innerHTML = rows.join('')
}

// ── Make-fast interaction (click cleat → click dock) ──────────────────────────

let down: { x: number; y: number; panX: number; panY: number } | null = null
let dragged = false

function canvasPos(e: MouseEvent): [number, number] {
  const r = canvas.getBoundingClientRect()
  return [e.clientX - r.left, e.clientY - r.top]
}

canvas.addEventListener('mousedown', (e) => {
  const [sx, sy] = canvasPos(e)
  down = { x: sx, y: sy, panX: cam.panX, panY: cam.panY }
  dragged = false
})

canvas.addEventListener('mousemove', (e) => {
  if (!down) return
  const [sx, sy] = canvasPos(e)
  const dx = sx - down.x, dy = sy - down.y
  if (Math.hypot(dx, dy) > 4) {
    dragged = true
    cam.panX = down.panX + dx
    cam.panY = down.panY + dy
  }
})

canvas.addEventListener('mouseup', (e) => {
  if (down && !dragged) handleClick(...canvasPos(e))
  down = null
})

function handleClick(sx: number, sy: number) {
  const cleat = pickCleat(cam, geo, body, sx, sy, ORIGIN_X, ORIGIN_Y)
  if (cleat !== null) { selectedCleat = cleat; return }
  if (selectedCleat !== null) {
    // Attach the dock end to the closest point on the nearest segment edge.
    const w = screenToWorld(cam, ORIGIN_X, ORIGIN_Y, sx, sy)
    let best: { point: { x: number; y: number }; dist: number } | null = null
    for (const seg of segments) {
      const cp = closestPointOnSegment(w, { x: seg.x1, y: seg.y1 }, { x: seg.x2, y: seg.y2 })
      if (!best || cp.dist < best.dist) best = { point: cp.point, dist: cp.dist }
    }
    if (best) {
      lines.push(makeLine(body, geo, selectedCleat, best.point))
      selectedLine = lines.length - 1
    }
    selectedCleat = null
  }
}

canvas.addEventListener('wheel', (e) => {
  e.preventDefault()
  const [sx, sy] = canvasPos(e)
  const before = screenToWorld(cam, ORIGIN_X, ORIGIN_Y, sx, sy)
  const factor = e.deltaY < 0 ? 1.1 : 1 / 1.1
  cam.zoom = Math.max(0.2, Math.min(6, cam.zoom * factor))
  const [nsx, nsy] = worldToScreen(cam, ORIGIN_X, ORIGIN_Y, before.x, before.y)
  cam.panX += sx - nsx
  cam.panY += sy - nsy
}, { passive: false })

// ── DOM controls ──────────────────────────────────────────────────────────────

const $ = <T extends HTMLElement>(id: string) => document.getElementById(id) as T
const throttleEl = $<HTMLInputElement>('throttle')
const throttleVal = $<HTMLSpanElement>('throttle-val')
const helmEl = $<HTMLInputElement>('helm')
const configEl = $<HTMLSelectElement>('rudder-config')
const standstillEl = $<HTMLInputElement>('standstill')
const standstillVal = $<HTMLSpanElement>('standstill-val')
const lengthEl = $<HTMLInputElement>('length')
const lengthVal = $<HTMLSpanElement>('length-val')
const windSpeedEl = $<HTMLInputElement>('wind-speed')
const windSpeedVal = $<HTMLSpanElement>('wind-speed-val')
const windDirEl = $<HTMLInputElement>('wind-dir')
const windDirVal = $<HTMLSpanElement>('wind-dir-val')
const presetEl = $<HTMLSelectElement>('preset')
const dragEl = $<HTMLInputElement>('toggle-drag')
const contactEl = $<HTMLInputElement>('toggle-contact')
const slowmoEl = $<HTMLInputElement>('toggle-slowmo')
const playBtn = $<HTMLButtonElement>('btn-play')

const clamp = (v: number, lo: number, hi: number) => Math.max(lo, Math.min(hi, v))

function syncControlsToUI() {
  throttleEl.value = String(controls.throttle)
  throttleVal.textContent = `${Math.round(controls.throttle * 100)}%`
  helmEl.value = String(controls.helm)
  configEl.value = controls.rudderConfig
  standstillEl.value = String(controls.standstillAuthority)
  standstillVal.textContent = controls.standstillAuthority.toFixed(2)
  standstillEl.disabled = controls.rudderConfig === 'twin'
  lengthEl.value = String(lengthFt)
  lengthVal.textContent = `${lengthFt} ft`
  windSpeedEl.value = String(wind.speed)
  windSpeedVal.textContent = `${msToKnots(wind.speed).toFixed(0)} kn`
  const deg = ((Math.round((wind.dirToward * 180) / Math.PI)) % 360 + 360) % 360
  windDirEl.value = String(deg)
  windDirVal.textContent = `${deg}°`
}

// Populate preset menu.
for (const p of PRESETS) {
  const opt = document.createElement('option')
  opt.value = p.id
  opt.textContent = p.label
  presetEl.appendChild(opt)
}

throttleEl.addEventListener('input', () => {
  controls.throttle = parseFloat(throttleEl.value)
  throttleVal.textContent = `${Math.round(controls.throttle * 100)}%`
})
helmEl.addEventListener('input', () => { controls.helm = parseFloat(helmEl.value) })
standstillEl.addEventListener('input', () => {
  controls.standstillAuthority = parseFloat(standstillEl.value)
  standstillVal.textContent = controls.standstillAuthority.toFixed(2)
})
configEl.addEventListener('change', () => {
  controls.rudderConfig = configEl.value as RudderConfig
  if (controls.rudderConfig === 'twin') {
    standstillEl.disabled = true
    controls.standstillAuthority = 0
  } else {
    standstillEl.disabled = false
    controls.standstillAuthority = defaultStandstillAuthority('single')
  }
  syncControlsToUI()
})

lengthEl.addEventListener('input', () => {
  lengthVal.textContent = `${lengthEl.value} ft`
})
lengthEl.addEventListener('change', () => {
  lengthFt = parseInt(lengthEl.value, 10)
  applyActive()  // re-derive all geometry-dependent state at the new length
})

windSpeedEl.addEventListener('input', () => {
  wind.speed = parseFloat(windSpeedEl.value)
  windSpeedVal.textContent = `${msToKnots(wind.speed).toFixed(0)} kn`
  persist()
})
windDirEl.addEventListener('input', () => {
  const deg = parseInt(windDirEl.value, 10)
  wind.dirToward = (deg * Math.PI) / 180
  windDirVal.textContent = `${deg}°`
  persist()
})

presetEl.addEventListener('change', () => {
  activePreset = presetEl.value as PresetId | 'free'
  applyActive()
})

dragEl.addEventListener('change', () => { showHullDrag = dragEl.checked })
contactEl.addEventListener('change', () => { showContact = contactEl.checked })
slowmoEl.addEventListener('change', () => { slowMo = slowmoEl.checked })

playBtn.addEventListener('click', togglePlay)
$<HTMLButtonElement>('btn-fit').addEventListener('click', fitView)
$<HTMLButtonElement>('btn-centre').addEventListener('click', () => { controls.helm = 0; helmEl.value = '0' })
$<HTMLButtonElement>('btn-reset').addEventListener('click', () => applyActive(false))

function togglePlay() {
  paused = !paused
  playBtn.textContent = paused ? 'Play' : 'Pause'
}

// ── Keyboard ──────────────────────────────────────────────────────────────────

window.addEventListener('keydown', (e) => {
  switch (e.key) {
    case 'ArrowUp':    controls.throttle = clamp(controls.throttle + 0.1, -1, 1); syncControlsToUI(); e.preventDefault(); break
    case 'ArrowDown':  controls.throttle = clamp(controls.throttle - 0.1, -1, 1); syncControlsToUI(); e.preventDefault(); break
    case 'ArrowLeft':  controls.helm = clamp(controls.helm - 0.1, -1, 1); helmEl.value = String(controls.helm); e.preventDefault(); break
    case 'ArrowRight': controls.helm = clamp(controls.helm + 0.1, -1, 1); helmEl.value = String(controls.helm); e.preventDefault(); break
    case 'c': case 'C': controls.helm = 0; helmEl.value = '0'; break
    case '[': if (selectedLine !== null) easeLine(lines[selectedLine]); break
    case ']': if (selectedLine !== null) takeUpLine(lines[selectedLine]); break
    case 'Delete': case 'Backspace':
      if (selectedLine !== null) {
        lines.splice(selectedLine, 1)
        selectedLine = lines.length ? lines.length - 1 : null
      }
      e.preventDefault(); break
    case ' ': togglePlay(); e.preventDefault(); break
    case 'r': case 'R': applyActive(false); break
  }
})

// ── Persistence ───────────────────────────────────────────────────────────────

const LS_KEY = 'dockphysics.session'

function persist() {
  try {
    localStorage.setItem(LS_KEY, JSON.stringify({
      lengthFt, windSpeed: wind.speed, windDir: wind.dirToward,
    }))
  } catch { /* storage unavailable */ }
}

function restore() {
  try {
    const raw = localStorage.getItem(LS_KEY)
    if (!raw) return
    const s = JSON.parse(raw)
    if (typeof s.lengthFt === 'number') lengthFt = clamp(Math.round(s.lengthFt), 37, 49)
    if (typeof s.windSpeed === 'number') wind.speed = clamp(s.windSpeed, 0, knotsToMs(50))
    if (typeof s.windDir === 'number') wind.dirToward = s.windDir
  } catch { /* ignore corrupt state */ }
}

// ── Boot ──────────────────────────────────────────────────────────────────────

restore()
applyActive()
syncControlsToUI()
playBtn.textContent = 'Pause'
requestAnimationFrame(tick)
