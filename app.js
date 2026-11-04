'use strict';
/* HDRI Capture v2: app (UI, devices, capture flow, result) */
const C = self.HDRCore, D2R = C.D2R;
const $ = id => document.getElementById(id);
const sleep = ms => new Promise(r => setTimeout(r, ms));
const QS = new URLSearchParams(location.search);
const SIM = QS.has('sim');

/* ============================ options ============================ */
const DEF = { bracketMode: 'auto', autoStep: 2, shadow: 2, count: 5, manStep: 2, bias: -2, stability: 'hand', density: 0.7,
              auto: true, lensMode: 'auto', lens: 26, align: true, out: 4096 };
const LIMITS = {  // exposure times in 100 µs units
  hand:   { base: 166.7, long: 500,   stableMs: 220, stableDeg: 0.9, abort: 4 },
  tripod: { base: 666.7, long: 10000, stableMs: 320, stableDeg: 0.4, abort: 1.5 } };
const TOL = 5, CALIB_TOL = 2.5, CALIB_TURN = 12;
const lim = () => LIMITS[S.opts.stability] || LIMITS.hand;

const S = {
  opts: { ...DEF }, screen: 'home', state: 'idle', orient: null, cam: null, camChoice: {},
  plan: null, targets: [], shots: [], nextIdx: 0, yaw0: 0, lens: 26, k1: 0, lensMeasured: false, aligned: false,
  work: null, workCtx: null, camIdx: 0, baseReady: Promise.resolve(), calib: null,
  still: { since: 0, d: null }, session: null, live: false, result: null, prev: null, prevBase: 0, sun: null, sunCfg: null,
  out: 4096, wakeLock: null, expShown: new Set()
};
window.App = S;

function loadOpts() {
  try { Object.assign(S.opts, JSON.parse(localStorage.getItem('hdri-opts-v2') || '{}')); } catch (e) {}
}
function saveOpts() { try { localStorage.setItem('hdri-opts-v2', JSON.stringify(S.opts)); } catch (e) {} }

/* ============================ small UI helpers ============================ */
let toastTimer = 0;
function toast(text, ms = 2600) {
  const t = $('toast'); t.textContent = text; t.classList.remove('hidden');
  clearTimeout(toastTimer); toastTimer = setTimeout(() => t.classList.add('hidden'), ms);
}
function show(id) { S.screen = id; for (const s of ['home', 'capture', 'result']) $(s).classList.toggle('hidden', s !== id); }
function status(main, sub) { if (main != null) $('statusMain').textContent = main; if (sub != null) $('statusSub').textContent = sub; }
const fmtTime = t => { const s = t * 1e-4; return s >= 0.3 ? s.toFixed(1) + ' s' : '1/' + Math.round(1 / s) + ' s'; };
const fmtEV = v => (v > 0 ? '+' : v < 0 ? '−' : '') + (Math.abs(v) % 1 ? Math.abs(v).toFixed(1) : Math.abs(v));
const vibrate = p => { try { navigator.vibrate && navigator.vibrate(p); } catch (e) {} };

/* ============================ storage (IndexedDB) ============================ */
const DB = {
  open() {
    if (this._p) return this._p;
    this._p = new Promise((res, rej) => {
      const r = indexedDB.open('hdri-capture', 1);
      r.onupgradeneeded = () => { const d = r.result;
        if (!d.objectStoreNames.contains('sessions')) d.createObjectStore('sessions', { keyPath: 'id' });
        if (!d.objectStoreNames.contains('shots')) d.createObjectStore('shots', { keyPath: ['sid', 'idx'] }); };
      r.onsuccess = () => res(r.result); r.onerror = () => rej(r.error);
    });
    return this._p;
  },
  async run(stores, mode, fn) {
    const db = await this.open();
    return new Promise((res, rej) => {
      const t = db.transaction(stores, mode); let out;
      const r = fn(t); if (r && 'onsuccess' in r) r.onsuccess = () => { out = r.result; };
      t.oncomplete = () => res(out); t.onerror = () => rej(t.error); t.onabort = () => rej(t.error);
    });
  },
  put(store, v) { return this.run(store, 'readwrite', t => t.objectStore(store).put(v)); },
  get(store, k) { return this.run(store, 'readonly', t => t.objectStore(store).get(k)); },
  all(store) { return this.run(store, 'readonly', t => t.objectStore(store).getAll()); },
  shots(sid) { return this.run('shots', 'readonly', t => t.objectStore('shots').getAll(IDBKeyRange.bound([sid, 0], [sid, 1e9]))); },
  delShot(sid, idx) { return this.run('shots', 'readwrite', t => t.objectStore('shots').delete([sid, idx])); },
  delSession(sid) { return this.run(['sessions', 'shots'], 'readwrite', t => { t.objectStore('sessions').delete(sid); t.objectStore('shots').delete(IDBKeyRange.bound([sid, 0], [sid, 1e9])); }); }
};
const safe = async (p, what) => { try { return await p; } catch (e) { console.warn('storage', what, e); return undefined; } };

/* ============================ orientation ============================ */
class Orientation {
  constructor() { this.M = null; this.src = ''; this.started = false; }
  async start() {
    if (this.started) return !!this.M;
    this.started = true;
    if (SIM) {
      this.src = 'simulated'; this.sim = { yaw: 0, pitch: 0, roll: 0 }; this.t0 = performance.now();
      this.drift = +QS.get('drift') || 0;
      const upd = () => { const s = this.sim, dt = (performance.now() - this.t0) / 1000;
        this.trueM = C.lookMat(s.yaw, s.pitch, s.roll); this.M = C.lookMat(s.yaw + this.drift * dt, s.pitch, s.roll); };
      upd(); setInterval(upd, 16); return true;
    }
    if ('RelativeOrientationSensor' in window) {
      try {
        try { await Promise.all(['accelerometer', 'gyroscope'].map(n => navigator.permissions.query({ name: n }))); } catch (e) {}
        const s = new RelativeOrientationSensor({ frequency: 60, referenceFrame: 'device' });
        await new Promise((res, rej) => {
          let done = false;
          s.onreading = () => { this.M = C.quatToMat(s.quaternion); if (!done) { done = true; res(); } };
          s.onerror = e => { if (!done) { done = true; rej(e.error || e); } };
          s.start(); setTimeout(() => { if (!done) { done = true; rej(new Error('timeout')); } }, 1500);
        });
        this.sensor = s; this.src = 'gyro'; return true;
      } catch (e) { console.warn('RelativeOrientationSensor', e); try { this.sensor && this.sensor.stop(); } catch (_) {} this.M = null; }
    }
    return new Promise(res => {
      let done = false;
      window.addEventListener('deviceorientation', e => { if (e.alpha == null) return; this.M = C.eulerToMat(e.alpha, e.beta, e.gamma); if (!done) { done = true; res(true); } });
      this.src = 'deviceorientation';
      setTimeout(() => { if (!done) { done = true; res(false); } }, 2500);
    });
  }
}

/* ============================ cameras ============================ */
class RealCamera {
  constructor(video) {
    this.video = video; this.tc = document.createElement('canvas'); this.tc.width = 32; this.tc.height = 32;
    this.tctx = this.tc.getContext('2d', { willReadFrequently: true });
  }
  async start(choice = {}) {
    this.stop();
    const v = { width: { ideal: 1920 }, height: { ideal: 1440 }, frameRate: { ideal: 30 } };
    if (choice.deviceId) v.deviceId = { exact: choice.deviceId }; else v.facingMode = { ideal: 'environment' };
    this.stream = await navigator.mediaDevices.getUserMedia({ video: v, audio: false });
    this.track = this.stream.getVideoTracks()[0];
    this.video.srcObject = this.stream;
    await this.video.play().catch(() => {});
    if (!this.video.videoWidth) await new Promise(r => this.video.addEventListener('loadedmetadata', r, { once: true }));
    await sleep(350);
    this.caps = this.track.getCapabilities ? this.track.getCapabilities() : {};
    this.deviceId = this.track.getSettings().deviceId;
    if (choice.zoom && this.caps.zoom) await this.apply({ zoom: choice.zoom });
    const c = this.caps;
    this.mode = (c.exposureTime && c.exposureMode && c.exposureMode.includes('manual')) ? 'manual'
      : (c.exposureCompensation && c.exposureCompensation.max > c.exposureCompensation.min) ? 'ev' : 'none';
  }
  get el() { return this.video; }
  get width() { return this.video.videoWidth; }
  get height() { return this.video.videoHeight; }
  stop() { if (this.stream) { this.stream.getTracks().forEach(t => t.stop()); this.stream = null; } }
  settings() { return this.track ? this.track.getSettings() : {}; }
  async apply(adv) { try { await this.track.applyConstraints({ advanced: [adv] }); return true; } catch (e) { console.warn('applyConstraints', adv, e); return false; } }
  nextFrame() {
    return Promise.race([new Promise(r => this.video.requestVideoFrameCallback ? this.video.requestVideoFrameCallback(() => r()) : requestAnimationFrame(() => r())), sleep(300)]);
  }
  thumb() {
    this.tctx.drawImage(this.video, 0, 0, 32, 32);
    const d = this.tctx.getImageData(0, 0, 32, 32).data; let s = 0;
    for (let i = 0; i < d.length; i += 4) s += d[i] + d[i + 1] + d[i + 2];
    return { d, mean: s / (32 * 32 * 3) };
  }
  grab(ctx, w, h) { ctx.drawImage(this.video, 0, 0, w, h); return ctx.getImageData(0, 0, w, h).data; }
}

class SimCamera {   // renders a synthetic HDR scene; ?sim[=ev]&lens=24&sun=1800&drift=0
  constructor(canvas, orient) {
    this.cv = canvas; this.orient = orient; this.W = 720; this.H = 960; this.lens = +QS.get('lens') || 24;
    if (QS.get('sun')) C.sim.sun = +QS.get('sun');
    this.t = 166.7; this.ev = 0; this.pending = []; this.frame = 0;
    this.mode = QS.get('sim') === 'ev' ? 'ev' : 'manual';
    this.caps = this.mode === 'ev' ? { exposureMode: ['continuous'], exposureCompensation: { min: -2, max: 2, step: 1 / 3 } }
      : { exposureMode: ['continuous', 'manual'], exposureTime: { min: 0.5, max: 5000, step: 0.1 }, iso: { min: 50, max: 3200, step: 1 }, whiteBalanceMode: ['continuous'] };
  }
  async start() {
    this.cv.width = 300; this.cv.height = 400; this.ctx = this.cv.getContext('2d'); this.img = this.ctx.createImageData(300, 400);
    this.running = true; const loop = () => { if (!this.running) return; this.render(300, 400, this.img.data, 0); this.ctx.putImageData(this.img, 0, 0); setTimeout(loop, 70); }; loop();
    this.deviceId = 'sim';
  }
  get el() { return this.cv; }
  get width() { return this.W; }
  get height() { return this.H; }
  stop() { this.running = false; }
  settings() { return { exposureTime: this.t, iso: 100 }; }
  async apply(adv) {
    if (adv.exposureTime) this.pending.push({ at: this.frame + 2, t: adv.exposureTime });
    if (adv.exposureCompensation != null) this.pending.push({ at: this.frame + 2, ev: adv.exposureCompensation });
    return true;
  }
  async nextFrame() { await sleep(33); this.frame++; this.pending = this.pending.filter(p => { if (p.at > this.frame) return true; if (p.t) this.t = p.t; if (p.ev != null) this.ev = p.ev; return false; }); }
  k() {
    if (this.mode === 'ev') { const d = C.viewDir(this.orient.trueM); return 0.25 / (1 + Math.max(0, d[2]) * 3) * Math.pow(2, this.ev); }
    return this.t / 166.7 * 0.25;
  }
  render(w, h, data, noise) { if (this.orient.trueM) C.simRender(this.orient.trueM, C.focalPx(Math.max(w, h), this.lens), w, h, this.k(), data, noise, this.frame * 7 + 1); }
  thumb() { const d = new Uint8ClampedArray(24 * 32 * 4); this.render(24, 32, d, 0); let s = 0; for (let i = 0; i < d.length; i += 4) s += d[i] + d[i + 1] + d[i + 2]; return { d, mean: s / (24 * 32 * 3) }; }
  grab(ctx, w, h) { const img = ctx.createImageData(w, h); this.render(w, h, img.data, 0.02); return img.data; }
}

/* ============================ merge worker client ============================ */
const Merger = {
  w: null, seq: 0, waits: new Map(),
  start() {
    try {
      this.w = new Worker('worker.js');
      this.w.onmessage = e => { const r = e.data, p = this.waits.get(r.id); if (!p) return; this.waits.delete(r.id); r.type === 'error' ? p.rej(new Error(r.message)) : p.res(r); };
      this.w.onerror = e => { console.warn('merge worker failed, using main thread', e); this.w = null; for (const p of this.waits.values()) p.rej(new Error('worker failed')); this.waits.clear(); };
    } catch (e) { this.w = null; }
  },
  init(cfg) { self.MergeEngine.init(cfg); if (this.w) this.w.postMessage({ type: 'init', ...cfg }); },
  async merge(job) {
    const id = ++this.seq; job.id = id;
    if (this.w) return new Promise((res, rej) => { this.waits.set(id, { res, rej }); this.w.postMessage({ type: 'merge', ...job }, job.frames); });
    await sleep(0); return self.MergeEngine.merge(job);
  }
};

/* ============================ home screen ============================ */
function bindOptions() {
  document.querySelectorAll('[data-opt]').forEach(el => {
    const key = el.dataset.opt;
    if (el.classList.contains('seg')) {
      el.querySelectorAll('button').forEach(b => b.onclick = () => { const v = b.dataset.v; S.opts[key] = isNaN(+v) ? v : +v; saveOpts(); renderOptions(); });
    } else if (el.type === 'checkbox') el.onchange = () => { S.opts[key] = el.checked; saveOpts(); renderOptions(); };
    else if (el.type === 'number') el.onchange = () => { const v = +el.value; if (v > 0) S.opts[key] = v; saveOpts(); renderOptions(); };
  });
  document.querySelectorAll('.stepper button').forEach(b => b.onclick = () => { S.opts.count = C.clamp(S.opts.count + +b.dataset.step, 2, 9); saveOpts(); renderOptions(); });
  $('optBias').oninput = () => { S.opts.bias = +$('optBias').value; saveOpts(); renderOptions(); };
}
function manualEVs(o = S.opts) { return Array.from({ length: o.count }, (_, i) => +(o.bias + o.manStep * (i - (o.count - 1) / 2)).toFixed(2)); }
function renderOptions() {
  const o = S.opts;
  document.querySelectorAll('[data-opt]').forEach(el => {
    const v = o[el.dataset.opt];
    if (el.classList.contains('seg')) el.querySelectorAll('button').forEach(b => b.classList.toggle('on', String(v) === b.dataset.v));
    else if (el.type === 'checkbox') el.checked = !!v;
    else if (el.type === 'number' && document.activeElement !== el) el.value = v;
  });
  $('autoBox').classList.toggle('hidden', o.bracketMode !== 'auto');
  $('manualBox').classList.toggle('hidden', o.bracketMode !== 'manual');
  $('lensRow').classList.toggle('hidden', o.lensMode !== 'manual');
  $('cntVal').textContent = o.count;
  $('optBias').value = o.bias; $('biasVal').textContent = fmtEV(o.bias) + ' EV';
  const evs = manualEVs(), lo = Math.min(...evs), hi = Math.max(...evs);
  let html = '';
  for (let e = -12; e <= 5; e++) html += `<i class="${evs.some(v => Math.abs(v - e) < 0.5) ? 'on' : ''}${e === 0 ? ' zero' : ''}"></i>`;
  $('ladderViz').innerHTML = html;
  $('ladderTxt').textContent = `${o.count} exposures from ${fmtEV(lo)} to ${fmtEV(hi)} EV (${(hi - lo)} stops bracketed). The camera's own range adds about 8 more. Shutter speeds are fitted to the scene when you begin.`;
}
function renderSupport() {
  const items = [];
  const it = (ok, t) => items.push(`<span class="${ok ? '' : 'bad'}">${ok ? '✓' : '✗'} ${t}</span>`);
  if (SIM) items.push('<span>Simulator: synthetic scene, drag to look</span>');
  else {
    it(window.isSecureContext, 'HTTPS');
    it(!!(navigator.mediaDevices && navigator.mediaDevices.getUserMedia), 'Camera');
    it('RelativeOrientationSensor' in window || 'DeviceOrientationEvent' in window, 'Gyro');
  }
  it(typeof CompressionStream !== 'undefined', 'EXR export');
  $('support').innerHTML = items.join('');
}
let galleryURLs = [];
async function renderGallery() {
  const list = (await safe(DB.all('sessions'), 'list')) || [];
  galleryURLs.forEach(u => URL.revokeObjectURL(u)); galleryURLs = [];
  list.sort((a, b) => b.created - a.created);
  $('galleryWrap').classList.toggle('hidden', !list.length);
  const g = $('gallery'); g.innerHTML = '';
  for (const s of list.slice(0, 30)) {
    const d = new Date(s.created);
    const el = document.createElement('div'); el.className = 'gitem'; el.setAttribute('role', 'button');
    let img = '<div class="ph"></div>';
    if (s.thumb) { const u = URL.createObjectURL(s.thumb); galleryURLs.push(u); img = `<img src="${u}" alt="">`; }
    el.innerHTML = `${img}<div class="gt"><b>${d.toLocaleDateString(undefined, { day: 'numeric', month: 'short' })} · ${d.toLocaleTimeString(undefined, { hour: '2-digit', minute: '2-digit' })}</b>
      <span>${s.shotCount || 0} directions${s.status === 'done' ? '' : ' · unfinished'}${s.lens ? ' · ' + (+s.lens).toFixed(0) + ' mm' : ''}</span></div><button class="gx" aria-label="Delete">×</button>`;
    el.onclick = () => openSession(s.id);
    el.querySelector('.gx').onclick = async ev => {
      ev.stopPropagation();
      if (el.dataset.arm) { await safe(DB.delSession(s.id), 'delete'); renderGallery(); }
      else { el.dataset.arm = 1; ev.target.textContent = 'Delete?'; ev.target.style.width = 'auto'; setTimeout(() => { if (el.isConnected) { delete el.dataset.arm; ev.target.textContent = '×'; ev.target.style.width = ''; } }, 3000); }
    };
    g.appendChild(el);
  }
}

/* ============================ capture: start / setup ============================ */
async function enterCapture(resume) {
  try { if (!SIM && document.documentElement.requestFullscreen && !document.fullscreenElement) await document.documentElement.requestFullscreen({ navigationUI: 'hide' }); } catch (e) {}
  try { if (screen.orientation && screen.orientation.lock) await screen.orientation.lock('portrait'); } catch (e) {}
  try { if (navigator.wakeLock) S.wakeLock = await navigator.wakeLock.request('screen'); } catch (e) {}
  show('capture'); S.state = 'starting'; ui();
  $('counter').textContent = '—'; status('Starting…', '');
  if (!S.orient) S.orient = new Orientation();
  if (!(await S.orient.start())) { status('No motion sensor', 'Chrome on Android over HTTPS is needed for gyro data.'); return; }
  try { await startCamera(S.camChoice); }
  catch (e) { console.error(e); status('Camera unavailable', e && e.name === 'NotAllowedError' ? 'Camera permission was denied. Allow it in site settings, then try again.' : String(e && e.message || e)); return; }
  requestAnimationFrame(loop);
  if (resume) {
    await setupWork(); await applyLocks(); S.camIdx = -1; restoreBase(); await S.baseReady;
    S.state = 'aim'; ui(); updateCounter(); status('Continue', 'Aim at the remaining dots');
  } else { S.state = 'setup'; ui(); renderCamSheet(); status('Point at the horizon', 'Phone upright, then tap Begin'); }
}

async function startCamera(choice) {
  if (SIM) {
    $('video').classList.add('hidden'); $('simview').classList.remove('hidden');
    if (!S.cam) { S.cam = new SimCamera($('simview'), S.orient); await S.cam.start(); }
    return;
  }
  $('simview').classList.add('hidden'); $('video').classList.remove('hidden');
  if (!S.cam) S.cam = new RealCamera($('video'));
  await S.cam.start(choice);
  S.camChoice = { deviceId: S.cam.deviceId, zoom: choice.zoom };
}

async function renderCamSheet() {
  const box = $('camChips'); box.innerHTML = '';
  let cams = [];
  if (!SIM) try {
    cams = (await navigator.mediaDevices.enumerateDevices()).filter(d => d.kind === 'videoinput');
    const back = cams.filter(d => /back|rear|environment/i.test(d.label));
    if (back.length) cams = back;
  } catch (e) {}
  const add = (label, choice, on) => {
    const b = document.createElement('button'); b.className = 'chip' + (on ? ' on' : ''); b.textContent = label;
    b.onclick = async () => {
      if (S.state !== 'setup') return;
      $('camInfo').textContent = 'Switching camera…';
      try { await startCamera(choice); } catch (e) { toast('That camera could not be opened'); }
      renderCamSheet();
    };
    box.appendChild(b);
  };
  if (SIM) add('Simulated camera', {}, true);
  cams.forEach((d, i) => add(`Camera ${i + 1}`, { deviceId: d.deviceId }, S.cam && S.cam.deviceId === d.deviceId && !S.camChoice.zoom));
  const z = S.cam && S.cam.caps && S.cam.caps.zoom;
  if (z && z.min < 1) add(z.min.toFixed(1) + '× wide', { deviceId: S.cam.deviceId, zoom: z.min }, !!S.camChoice.zoom);
  renderCamInfo();
}
function renderCamInfo() {
  const c = S.cam; if (!c) return;
  const res = `${c.width}×${c.height}`;
  let t;
  if (c.mode === 'manual') { const e = c.caps.exposureTime; t = `<b>Manual exposure ✓</b> · ${res} · shutter ${fmtTime(e.max)}–${fmtTime(e.min)}`; }
  else if (c.mode === 'ev') t = `<b>Limited exposure control</b> · ${res}. This camera only allows ±${c.caps.exposureCompensation.max} EV in the browser, so highlights will clip. Try another camera if one shows “Manual exposure”.`;
  else t = `<b>No exposure control</b> · ${res}. The result will have low dynamic range. Try another camera.`;
  if (S.opts.lensMode === 'auto') t += '<br>After Begin you turn once to the right so the app can measure this lens.';
  $('camInfo').innerHTML = t;
}

/* ============================ exposure control ============================ */
// wait until a frame with the new exposure arrives (predicts its brightness from the previous frame)
async function waitExposure(prev, ratio) {
  const cam = S.cam, t0 = performance.now();
  let pred = null, lo = 0, hi = 0;
  if (prev && ratio && Math.abs(Math.log(ratio)) > 0.05) {
    const d = prev.d, n = d.length / 4; let s = 0, clipped = 0;
    for (let i = 0; i < d.length; i += 4) for (let c = 0; c < 3; c++) { const z = d[i + c]; s += C.toSrgb8(C.LIN[z] * ratio); if (z >= 250) clipped++; }
    pred = s / (n * 3); const tol = Math.max(3, pred * 0.12);
    lo = pred - tol; hi = pred + tol + (ratio < 1 ? 255 * clipped / (n * 3) : 0);
  }
  const pm = prev ? prev.mean : null, small = pred == null || Math.abs(pred - pm) < 3;
  let last = null, stable = 0, frames = 0, th = null;
  while (performance.now() - t0 < 1500) {
    await cam.nextFrame(); frames++;
    th = cam.thumb(); const m = th.mean;
    if (!small) {
      const moved = Math.abs(m - pm) >= 0.5 * Math.abs(pred - pm);
      if (moved && m >= lo && m <= hi) return th;
      if (moved && frames >= 2 && last != null && Math.abs(m - last) <= Math.max(1, last * 0.02)) return th;
    } else if (frames >= 4 && last != null && Math.abs(m - last) <= Math.max(1, last * 0.02)) { if (++stable >= 1) return th; }
    last = m;
  }
  return th || cam.thumb();
}
const manualSpec = (t, iso) => { const a = { exposureMode: 'manual', exposureTime: t }; if (iso) a.iso = iso; return a; };

async function buildPlan() {
  const cam = S.cam, c = cam.caps || {}, o = S.opts, L = lim();
  if (cam.mode === 'manual') {
    const st = cam.settings(), tMin = c.exposureTime.min, tMax = c.exposureTime.max;
    const isoC = c.iso && c.iso.max > c.iso.min ? c.iso : null;
    const baseMax = Math.min(tMax, L.base), longMax = Math.min(tMax, L.long);
    let t = C.clamp(st.exposureTime > 0 ? st.exposureTime : 100, tMin, baseMax);
    let iso = isoC ? C.clamp(st.iso || 100, isoC.min, isoC.max) : null;
    if (st.exposureTime > baseMax && isoC) iso = C.clamp(iso * st.exposureTime / baseMax, isoC.min, isoC.max);
    let th = cam.thumb(), prevT = null;
    for (let it = 0; it < 6; it++) {
      await cam.apply(manualSpec(t, iso));
      th = await waitExposure(th, prevT ? t / prevT : null); prevT = t;
      const m = th.mean;
      if (m > 95 && m < 140) break;
      let k = m <= 2 ? 16 : m >= 253 ? 1 / 16 : 0.2 / C.LIN[Math.round(m)];
      k = C.clamp(k, 1 / 16, 16);
      let nt = t * k;
      if (nt > baseMax && isoC && iso < isoC.max) { const ni = C.clamp(iso * nt / baseMax, isoC.min, isoC.max); nt = nt * iso / ni; iso = ni; }
      else if (nt < t && isoC && iso > isoC.min && nt < baseMax / 8) { const ni = C.clamp(iso * nt / (baseMax / 8), isoC.min, iso); nt = nt * iso / ni; iso = ni; }
      const nc = C.clamp(nt, tMin, baseMax);
      if (Math.abs(nc - t) / t < 0.05) break;
      t = nc;
    }
    let evs;
    if (o.bracketMode === 'manual') evs = manualEVs();
    else {
      evs = [0]; const st2 = o.autoStep;
      for (let ev = -st2; ev >= -18 && t * Math.pow(2, ev) >= tMin * 0.999; ev -= st2) evs.push(ev);
      const lowest = Math.min(...evs); if (t * Math.pow(2, lowest) / tMin > 1.3) evs.push(Math.log2(tMin / t));
      for (let ev = st2; ev <= o.shadow + 1e-6; ev += st2) evs.push(ev);
    }
    const times = [...new Set(evs.map(ev => C.clamp(t * Math.pow(2, ev), tMin, longMax)))].sort((a, b) => a - b);
    const uniq = []; for (const x of times) if (!uniq.length || x / uniq[uniq.length - 1] > 1.25) uniq.push(x);
    let anchor = 0, bd = Infinity; uniq.forEach((x, i) => { const d = Math.abs(Math.log(x / t)); if (d < bd) { bd = d; anchor = i; } });
    S.plan = { mode: 'manual', adaptive: o.bracketMode === 'auto', base: t, iso, times: uniq, steps: uniq.map(x => manualSpec(x, iso)),
               nominal: uniq.map(x => x / t), anchor, clipT: 0.0004, darkT: o.shadow >= 4 ? 0.004 : 0.015 };
  } else if (cam.mode === 'ev') {
    const ec = c.exposureCompensation, step = ec.step || 1 / 3, r = v => C.clamp(Math.round(v / step) * step, ec.min, ec.max);
    const n = o.bracketMode === 'manual' ? Math.max(3, o.count) : 5;
    const list = [...new Set(Array.from({ length: n }, (_, i) => +r(ec.min + (ec.max - ec.min) * i / (n - 1)).toFixed(3)).concat([0]))].sort((a, b) => a - b);
    S.plan = { mode: 'ev', adaptive: false, times: list.map(ev => Math.pow(2, ev)), steps: list.map(ev => ({ exposureMode: 'continuous', exposureCompensation: ev })),
               nominal: list.map(ev => Math.pow(2, ev)), anchor: list.indexOf(0) };
  } else {
    S.plan = { mode: 'none', adaptive: false, times: [1], steps: [null], nominal: [1], anchor: 0 };
  }
  try {
    const st = cam.settings();
    if (c.whiteBalanceMode && c.whiteBalanceMode.includes('manual') && st.colorTemperature) S.plan.wb = { whiteBalanceMode: 'manual', colorTemperature: st.colorTemperature };
  } catch (e) {}
  await applyLocks();
  S.camIdx = -1; restoreBase(); await S.baseReady;
}
async function applyLocks() { if (S.plan && S.plan.wb) await S.cam.apply(S.plan.wb); }
function restoreBase() {
  const P = S.plan, cam = S.cam;
  if (!P || !cam) { S.baseReady = Promise.resolve(); return; }
  if (S.camIdx === P.anchor) return;
  const ratio = S.camIdx >= 0 ? P.times[P.anchor] / P.times[S.camIdx] : null, prev = cam.thumb();
  S.camIdx = P.anchor;
  S.baseReady = (async () => { if (P.steps[P.anchor]) await cam.apply(P.steps[P.anchor]); await waitExposure(prev, ratio); })();
}

/* ============================ capture: begin / calibrate ============================ */
async function setupWork() {
  const cw = S.cam.width, ch = S.cam.height, long = Math.max(cw, ch), wl = S.out > 4096 ? 1600 : 1152, sc = Math.min(1, wl / long);
  S.work = { w: Math.round(cw * sc), h: Math.round(ch * sc) }; S.work.long = Math.max(S.work.w, S.work.h);
  const cv = document.createElement('canvas'); cv.width = S.work.w; cv.height = S.work.h;
  S.workCtx = cv.getContext('2d', { willReadFrequently: true });
}
async function begin() {
  if (!S.orient.M) { toast('Waiting for the motion sensor…'); return; }
  S.state = 'busy'; ui(); status('Metering…', 'Hold still');
  S.out = S.opts.out;
  await setupWork();
  await buildPlan();
  Merger.init({ global: S.plan.mode === 'manual', nominal: S.plan.nominal, anchor: S.plan.anchor });
  S.shots = []; S.nextIdx = 0; S.targets = []; S.result = null; S.aligned = false; S.k1 = 0; S.sunCfg = null; S.live = true;
  const M = S.orient.M, d = C.viewDir(M);
  S.yaw0 = C.headingDeg(d);
  S.session = { id: Date.now().toString(36), created: Date.now(), status: 'capturing', shotCount: 0 };
  renderExpbar();
  if (S.opts.lensMode === 'auto') {
    await S.cam.nextFrame();
    const fac = Math.max(1, Math.round(S.work.long / 288));
    S.calib = { A: C.grayFrom(S.cam.grab(S.workCtx, S.work.w, S.work.h), S.work.w, S.work.h, fac), MA: S.orient.M, fac,
                dir: C.dirFromYP(S.yaw0 + CALIB_TURN, C.clamp(Math.asin(d[2]) / D2R, -20, 20)) };
    S.state = 'calib'; ui(); status('Lens check', 'Turn right to the dot and hold still');
  } else { S.lens = S.opts.lens; S.lensMeasured = false; startTargets(); }
}
async function doCalib() {
  S.state = 'busy'; ui(); status('Measuring lens…', '');
  const c = S.calib;
  const B = C.grayFrom(S.cam.grab(S.workCtx, S.work.w, S.work.h), S.work.w, S.work.h, c.fac);
  const r = C.lensFromPair(c.A, B, c.MA, S.orient.M);
  if (isFinite(r.error) && r.conf > 0.2 && r.lens > 9 && r.lens < 78) { S.lens = r.lens; S.lensMeasured = true; toast(`Lens measured: ${r.lens} mm`); }
  else { S.lens = S.opts.lens; S.lensMeasured = false; toast(`Couldn't measure the lens here, using ${S.lens} mm`); }
  S.calib = null;
  startTargets();
}
function startTargets() {
  const f = C.focalPx(Math.max(S.cam.width, S.cam.height), S.lens);
  S.targets = C.makeTargets(C.fovDeg(S.cam.width, f), C.fovDeg(S.cam.height, f), S.opts.density, S.yaw0);
  S.state = 'aim'; ui(); updateCounter();
  status('Aim at the black dot', S.plan.mode === 'manual' ? `Base ${fmtTime(S.plan.base)}${S.plan.iso ? ' · ISO ' + Math.round(S.plan.iso) : ''} · ${S.plan.times.length} exposures available` : '');
  saveSession();
}

/* ============================ capture: shooting ============================ */
function nearestTarget(d, maxDeg) {
  let best = null, bd = maxDeg * D2R;
  for (const t of S.targets) { if (t.done) continue; const a = C.angleBetween(d, t.dir); if (a < bd) { bd = a; best = t; } }
  return best;
}
const nextTarget = () => S.targets.find(t => !t.done) || null;

async function captureShot(target) {
  if (S.state !== 'aim') return;
  const P = S.plan, cam = S.cam, L = lim(), n = S.work.w * S.work.h;
  S.state = 'shooting'; ui();
  await S.baseReady;
  const d0 = C.viewDir(S.orient.M), frames = [], mats = [], idxs = [];
  S.expShown = new Set(); renderExpbar(); $('expbar').classList.remove('hidden');
  let thumb = cam.thumb();
  const take = async j => {
    if (j !== S.camIdx) {
      const ratio = P.times[j] / P.times[S.camIdx];
      if (P.steps[j]) await cam.apply(P.steps[j]);
      S.camIdx = j; thumb = await waitExposure(thumb, ratio);
    } else { await cam.nextFrame(); thumb = cam.thumb(); }
    const M = S.orient.M;
    if (C.angleBetween(C.viewDir(M), d0) > L.abort * D2R) return null;
    const data = cam.grab(S.workCtx, S.work.w, S.work.h);
    frames.push(data); mats.push(M); idxs.push(j); S.expShown.add(j); renderExpbar();
    return data;
  };
  let ok = true;
  status('Hold still', '');
  try {
    const f0 = await take(P.anchor); if (!f0) throw 0;
    let last = f0;
    for (let j = P.anchor - 1; j >= 0; j--) { if (P.adaptive && C.clipStats(last, n).clip <= P.clipT) break; last = await take(j); if (!last) throw 0; }
    last = f0;
    for (let j = P.anchor + 1; j < P.times.length; j++) { if (P.adaptive && C.clipStats(last, n).dark <= P.darkT) break; last = await take(j); if (!last) throw 0; }
  } catch (e) { ok = false; }
  restoreBase();
  setTimeout(() => { if (S.state !== 'shooting') $('expbar').classList.add('hidden'); }, 500);
  if (!ok) {
    S.state = 'aim'; S.still.since = performance.now(); ui();
    status('Moved, trying again', 'Hold still until the ring closes'); vibrate([30, 50, 30]);
    return;
  }
  const order = idxs.map((_, k) => k).sort((a, b) => idxs[a] - idxs[b]);
  const sf = order.map(k => frames[k].buffer), sm = order.map(k => mats[k]), si = order.map(k => idxs[k]), ref = si.indexOf(P.anchor);
  const shot = { idx: S.nextIdx++, target: target ? S.targets.indexOf(target) : -1, M: sm[ref], M0: sm[ref], w: S.work.w, h: S.work.h,
                 rgbe: null, clip: Infinity, gain: 1, nFrames: si.length };
  if (target) target.done = true;
  S.shots.push(shot);
  shot.promise = Merger.merge({ frames: sf, mats: sm, idx: si, ref, w: S.work.w, h: S.work.h, f: C.focalPx(S.work.long, S.lens) })
    .then(r => { shot.rgbe = r.rgbe; shot.clip = r.clip; return safe(DB.put('shots', { sid: S.session.id, idx: shot.idx, rgbe: shot.rgbe, w: shot.w, h: shot.h, M: shot.M0, target: shot.target, clip: shot.clip }), 'shot'); })
    .catch(err => { console.error('merge failed', err); shot.failed = true; if (target) target.done = false; toast('A shot could not be processed. Shoot it again.'); });
  vibrate(20);
  S.state = 'aim'; S.still.since = performance.now(); ui(); updateCounter();
  const left = S.targets.filter(t => !t.done).length;
  status(left ? 'Next: black dot' : 'All directions captured', `${si.length} exposure${si.length > 1 ? 's' : ''}${P.mode === 'manual' ? ' · ' + fmtTime(P.times[si[0]]) + ' – ' + fmtTime(P.times[si[si.length - 1]]) : ''}`);
  saveSession();
  if (!left) setTimeout(finish, 350);
}

async function undo() {
  if (S.state !== 'aim' || !S.shots.length) return;
  const s = S.shots.pop();
  try { await s.promise; } catch (e) {}
  if (s.target >= 0 && S.targets[s.target]) S.targets[s.target].done = false;
  safe(DB.delShot(S.session.id, s.idx), 'undo');
  updateCounter(); status('Removed last shot', ''); saveSession();
}

let saveTimer = 0;
function saveSession(extra = {}) {
  if (!S.session) return Promise.resolve();
  Object.assign(S.session, extra, {
    opts: S.opts, plan: S.plan, yaw0: S.yaw0, lens: S.lens, k1: S.k1, lensMeasured: S.lensMeasured, work: S.work, out: S.out,
    targets: S.targets.map(t => ({ yaw: t.yaw, pitch: t.pitch, ring: t.ring, done: t.done })), shotCount: S.shots.filter(s => !s.failed).length,
    aligned: S.aligned, alignedM: S.aligned ? Object.fromEntries(S.shots.map(s => [s.idx, s.M])) : null, sun: S.sunCfg, updated: Date.now()
  });
  clearTimeout(saveTimer);
  return new Promise(res => { saveTimer = setTimeout(async () => { await safe(DB.put('sessions', S.session), 'session'); res(); }, 150); });
}

async function finish() {
  if (S.state === 'finishing' || !S.shots.length) return;
  S.state = 'finishing'; ui();
  if (S.cam) S.cam.stop();
  try { S.wakeLock && S.wakeLock.release(); } catch (e) {}
  try { if (document.fullscreenElement) await document.exitFullscreen(); } catch (e) {}
  show('result'); setBusy(true, 'Finishing the last shots…', 0);
  await Promise.all(S.shots.map(s => s.promise));
  S.shots = S.shots.filter(s => s.rgbe && !s.failed);
  if (!S.shots.length) { setBusy(false); toast('No usable shots'); goHome(); return; }
  S.cam = null;
  await processResult({ align: S.opts.align });
}

/* ============================ capture: drawing ============================ */
function ui() {
  const st = S.state;
  $('setupSheet').classList.toggle('hidden', st !== 'setup');
  $('controls').classList.toggle('hidden', !['aim', 'shooting', 'calib', 'busy'].includes(st) || !S.targets.length && st !== 'calib');
  $('btnShoot').disabled = st !== 'aim';
  $('btnUndo').disabled = !(st === 'aim' && S.shots.length);
  $('btnFinish').disabled = !(st === 'aim' && S.shots.length);
}
function updateCounter() { const done = S.targets.filter(t => t.done).length; $('counter').textContent = S.targets.length ? `${done}/${S.targets.length}` : '—'; }
function renderExpbar() {
  const P = S.plan; if (!P) return;
  $('expbar').innerHTML = P.times.map((_, j) => `<i class="${j === P.anchor ? 'base' : ''}${S.expShown.has(j) ? ' done' : ''}"></i>`).join('');
}

function loop() {
  if (S.screen !== 'capture') return;
  const M = S.orient && S.orient.M, now = performance.now(), L = lim();
  const portrait = window.innerHeight >= window.innerWidth || SIM;
  $('rotateMsg').classList.toggle('hidden', portrait);
  if (M) {
    const d = C.viewDir(M), st = S.still;
    if (!st.d || C.angleBetween(st.d, d) > L.stableDeg * D2R) { st.d = d; st.since = now; }
    S.stillFor = now - st.since;
    const stable = S.stillFor >= L.stableMs;
    if (portrait && S.state === 'aim' && S.opts.auto) {
      const tgt = nearestTarget(d, TOL); S.aimTarget = tgt;
      if (tgt && stable) captureShot(tgt);
    } else if (portrait && S.state === 'calib' && S.calib) {
      S.aimTarget = C.angleBetween(d, S.calib.dir) < CALIB_TOL * D2R ? S.calib : null;
      if (S.aimTarget && stable) doCalib();
    } else S.aimTarget = null;
  }
  drawOverlay(); drawMinimap();
  requestAnimationFrame(loop);
}

function drawOverlay() {
  const cv = $('overlay'), dpr = Math.min(2, window.devicePixelRatio || 1), sw = cv.clientWidth, sh = cv.clientHeight;
  if (cv.width !== Math.round(sw * dpr) || cv.height !== Math.round(sh * dpr)) { cv.width = Math.round(sw * dpr); cv.height = Math.round(sh * dpr); }
  const ctx = cv.getContext('2d'); ctx.setTransform(dpr, 0, 0, dpr, 0, 0); ctx.clearRect(0, 0, sw, sh);
  const M = S.orient && S.orient.M, cam = S.cam; if (!M || !cam || !cam.width) return;
  const vw = cam.width, vh = cam.height, sc = Math.max(sw / vw, sh / vh), ox = (sw - vw * sc) / 2, oy = (sh - vh * sc) / 2;
  const f = C.focalPx(Math.max(vw, vh), S.lens), cx = vw / 2, cy = vh / 2;
  const toDev = d => [M[0] * d[0] + M[3] * d[1] + M[6] * d[2], M[1] * d[0] + M[4] * d[1] + M[7] * d[2], M[2] * d[0] + M[5] * d[1] + M[8] * d[2]];
  const project = d => { const t = toDev(d); if (t[2] > -0.08) return null; return [ox + (cx + f * t[0] / -t[2]) * sc, oy + (cy - f * t[1] / -t[2]) * sc]; };
  const mx = sw / 2, my = sh / 2;
  // horizon
  ctx.lineWidth = 1; ctx.strokeStyle = 'rgba(255,255,255,.28)'; ctx.beginPath(); let pen = false;
  for (let a = -180; a <= 180; a += 3) { const p = project(C.dirFromYP(a, 0)); if (!p) { pen = false; continue; } pen ? ctx.lineTo(p[0], p[1]) : ctx.moveTo(p[0], p[1]); pen = true; }
  ctx.stroke();
  const dot = (p, kind) => {
    ctx.beginPath();
    if (kind === 'done') { ctx.fillStyle = 'rgba(236,236,238,.85)'; ctx.arc(p[0], p[1], 5, 0, 7); ctx.fill(); }
    else if (kind === 'next') {
      ctx.fillStyle = '#0b0b0c'; ctx.strokeStyle = '#ececee'; ctx.lineWidth = 3; ctx.arc(p[0], p[1], 13, 0, 7); ctx.fill(); ctx.stroke();
      ctx.beginPath(); ctx.fillStyle = '#ececee'; ctx.arc(p[0], p[1], 3.5, 0, 7); ctx.fill();
    } else { ctx.strokeStyle = 'rgba(236,236,238,.75)'; ctx.lineWidth = 2; ctx.arc(p[0], p[1], 9, 0, 7); ctx.stroke(); }
  };
  let nxt = null;
  if (S.state === 'calib' && S.calib) nxt = S.calib;
  else if (['aim', 'shooting', 'busy'].includes(S.state)) nxt = nextTarget();
  for (const t of S.targets) { const p = project(t.dir); if (p) dot(p, t.done ? 'done' : t === nxt ? 'next' : 'open'); }
  if (nxt) {
    const p = project(nxt.dir);
    if (S.state === 'calib' && p) dot(p, 'next');
    if (!p || p[0] < 24 || p[1] < 110 || p[0] > sw - 24 || p[1] > sh - 150) {
      const t = toDev(nxt.dir); let vx = t[0], vy = -t[1]; const l = Math.hypot(vx, vy) || 1; vx /= l; vy /= l;
      const r = Math.min(sw, sh) * 0.32;
      ctx.save(); ctx.translate(mx + vx * r, my + vy * r); ctx.rotate(Math.atan2(vy, vx));
      ctx.fillStyle = '#ececee'; ctx.strokeStyle = '#0b0b0c'; ctx.lineWidth = 2;
      ctx.beginPath(); ctx.moveTo(16, 0); ctx.lineTo(-9, -12); ctx.lineTo(-3, 0); ctx.lineTo(-9, 12); ctx.closePath(); ctx.fill(); ctx.stroke();
      ctx.restore();
    }
  }
  // reticle
  ctx.lineWidth = 2.5; ctx.strokeStyle = 'rgba(236,236,238,.95)'; ctx.beginPath(); ctx.arc(mx, my, 26, 0, 7); ctx.stroke();
  let a0 = -Math.PI / 2, a1 = null;
  if (S.state === 'shooting' || S.state === 'busy') { const t = performance.now() / 300; a0 = t; a1 = t + 1.6; }
  else if (S.aimTarget) a1 = a0 + Math.min(1, S.stillFor / lim().stableMs) * 2 * Math.PI;
  if (a1 != null) { ctx.lineWidth = 6; ctx.strokeStyle = '#ececee'; ctx.beginPath(); ctx.arc(mx, my, 26, a0, a1); ctx.stroke(); }
  if (S.state === 'calib') { ctx.font = '600 13px Inter, system-ui'; ctx.fillStyle = '#ececee'; ctx.textAlign = 'center'; ctx.fillText('lens check', mx, my + 50); }
}
function drawMinimap() {
  const cv = $('minimap'), ctx = cv.getContext('2d'), W = cv.width, H = cv.height;
  ctx.clearRect(0, 0, W, H);
  ctx.strokeStyle = 'rgba(11,11,12,.15)'; ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(8, H / 2); ctx.lineTo(W - 8, H / 2); ctx.stroke();
  const pos = d => { let lon = Math.atan2(d[0], d[1]) / D2R - S.yaw0; lon = ((lon + 540) % 360) - 180; return [(lon / 360 + 0.5) * (W - 20) + 10, (0.5 - Math.asin(d[2]) / D2R / 180) * (H - 16) + 8]; };
  const nxt = nextTarget();
  for (const t of S.targets) { const [x, y] = pos(t.dir); ctx.fillStyle = t.done ? '#0b0b0c' : '#b9b9be'; ctx.beginPath(); ctx.arc(x, y, t === nxt ? 7 : 5, 0, 7); ctx.fill(); if (t === nxt) { ctx.strokeStyle = '#0b0b0c'; ctx.lineWidth = 3; ctx.stroke(); } }
  if (S.orient && S.orient.M) { const [x, y] = pos(C.viewDir(S.orient.M)); ctx.strokeStyle = '#0b0b0c'; ctx.lineWidth = 3; ctx.beginPath(); ctx.arc(x, y, 11, 0, 7); ctx.stroke(); }
}

/* ============================ result ============================ */
function setBusy(on, msg, p) {
  $('busy').classList.toggle('hidden', !on);
  if (msg) $('busyMsg').textContent = msg;
  if (p != null) $('busyBar').style.width = (p * 100).toFixed(0) + '%';
  for (const id of ['btnSave', 'btnShare', 'btnJpg', 'btnAlign', 'btnRestitch', 'btnMore', 'btnDelete']) $(id).disabled = on;
}
const bar = p => { $('busyBar').style.width = (C.clamp(p, 0, 1) * 100).toFixed(0) + '%'; };

async function processResult({ align }) {
  setBusy(true, 'Preparing…', 0);
  const f = () => C.focalPx(S.work.long, S.lens);
  if (S.plan.mode !== 'manual' && S.shots.length > 1) {
    setBusy(true, 'Balancing exposures…'); await sleep(20);
    const g = C.harmonizeGains(S.shots, f(), S.k1); S.shots.forEach((s, i) => s.gain = g[i]);
  }
  if (align && S.shots.length >= 3) {
    S.shots.forEach(s => s.M = s.M0);
    setBusy(true, 'Aligning · lens', 0);
    let r = await C.fitLensK(S.shots, S.work.long, S.lens, S.k1, { wide: !S.lensMeasured, onProgress: p => bar(p * 0.35) });
    if (isFinite(r.error)) { S.lens = r.lens; S.k1 = r.k1; }
    setBusy(true, 'Aligning · shots');
    await C.refineRotations(S.shots, f(), S.k1, p => bar(0.35 + p * 0.45));
    setBusy(true, 'Aligning · fine-tune');
    r = await C.fitLensK(S.shots, S.work.long, S.lens, S.k1, { fine: true, onProgress: p => bar(0.8 + p * 0.2) });
    if (isFinite(r.error)) { S.lens = r.lens; S.k1 = r.k1; }
    S.aligned = true;
  }
  await stitchNow();
}

async function stitchNow() {
  setBusy(true, 'Stitching', 0);
  const W = S.out, H = W / 2, t0 = performance.now();
  const res = await C.stitch(S.shots, { outW: W, outH: H, f: C.focalPx(S.work.long, S.lens), k1: S.k1, yaw0: S.yaw0, onProgress: bar });
  S.result = { W, H, rgb: res.rgb, holes: res.holes, yaw0: S.yaw0, ms: performance.now() - t0 };
  S.sun = null;
  if (S.plan.mode === 'manual') {
    const clip = Math.min(...S.shots.map(s => s.clip));
    if (isFinite(clip)) S.sun = C.detectSun(res.rgb, W, H, clip, S.yaw0);
  }
  if (S.sun) { if (!S.sunCfg) S.sunCfg = { on: isOutdoor(), strength: 8 }; applySunNow(); }
  S.prev = C.downsample(res.rgb, W, H, 2048, 1024);
  S.prevBase = C.autoEV(S.prev);
  $('prevEV').value = 0;
  drawPreview(); renderResultInfo();
  setBusy(false);
  saveSession({ status: 'done', thumb: await makeThumb() });
}
function isOutdoor() { const P = S.plan; return P && P.mode === 'manual' && P.base * ((P.iso || 100) / 100) < 25; }
function applySunNow(refresh) {
  const info = C.applySun(S.result, S.sunCfg && S.sunCfg.on ? S.sun : null, S.sunCfg ? S.sunCfg.strength : 0);
  $('sunCard').classList.toggle('hidden', !S.sun);
  if (S.sun) {
    $('sunOn').checked = !!S.sunCfg.on; $('sunStrength').value = S.sunCfg.strength; $('sunVal').textContent = S.sunCfg.strength + '×';
    $('sunRow').classList.toggle('hidden', !S.sunCfg.on);
    $('sunNote').textContent = `A light source about ${(S.sun.radiusDeg * 2).toFixed(1)}° wide, ${S.sun.elevation.toFixed(0)}° above the horizon, clipped even at the fastest shutter. ` +
      (S.sunCfg.on ? `The sun is rebuilt with ${S.sunCfg.strength}× the sky's light: clear sky ≈ 8, hazy ≈ 3.` : `Turn this on if it's the sun, so Blender gets sharp shadows.`);
  }
  if (refresh) { S.prev = C.downsample(S.result.rgb, S.result.W, S.result.H, 2048, 1024); drawPreview(); }
  return info;
}
function renderResultInfo() {
  const R = S.result, P = S.plan, open = S.targets.filter(t => !t.done).length;
  const frames = S.shots.reduce((a, s) => a + (s.nFrames || 0), 0);
  $('resInfo').innerHTML = [`${R.W}×${R.H}`, `${S.shots.length} directions`, frames ? `${frames} frames` : null,
    `lens ${(+S.lens).toFixed(1)} mm`, P.mode === 'manual' && P.iso ? `ISO ${Math.round(P.iso)}` : null,
    P.mode === 'ev' ? 'limited exposure control' : null, open ? `${open} directions not shot (filled in)` : null,
    R.holes > 0.003 ? `${(R.holes * 100).toFixed(1)}% of the sphere estimated` : null].filter(Boolean).join(' · ');
  $('resLens').value = (+S.lens).toFixed(1); $('resK1').value = (+S.k1).toFixed(3);
  $('resOutSeg').querySelectorAll('button').forEach(b => b.classList.toggle('on', +b.dataset.v === S.out));
  $('btnMore').classList.toggle('hidden', !S.live);
  $('resTitle').textContent = new Date(S.session ? S.session.created : Date.now()).toLocaleString(undefined, { day: 'numeric', month: 'short', hour: '2-digit', minute: '2-digit' });
  const canShare = navigator.canShare && (() => { try { return navigator.canShare({ files: [new File([new Uint8Array(1)], 'x.exr', { type: 'image/x-exr' })] }); } catch (e) { return false; } })();
  $('btnShare').classList.toggle('hidden', !canShare);
}
function drawPreview() {
  if (!S.prev) return;
  const ev = S.prevBase + +$('prevEV').value;
  $('evVal').textContent = fmtEV(+$('prevEV').value) + ' EV';
  const cv = $('preview'), ctx = cv.getContext('2d'), img = ctx.createImageData(2048, 1024);
  C.tonemap(S.prev, 2048 * 1024, ev, img.data);
  ctx.putImageData(img, 0, 0);
  S.prevRGBA = img.data;
  if (Viewer.active) Viewer.upload(img.data, 2048, 1024);
}
async function makeThumb() {
  try {
    const cv = document.createElement('canvas'); cv.width = 512; cv.height = 256;
    cv.getContext('2d').drawImage($('preview'), 0, 0, 512, 256);
    return await new Promise(r => cv.toBlob(r, 'image/jpeg', 0.8));
  } catch (e) { return null; }
}
const stamp = () => { const d = new Date(S.session ? S.session.created : Date.now()), p = n => String(n).padStart(2, '0'); return `hdri_${d.getFullYear()}${p(d.getMonth() + 1)}${p(d.getDate())}_${p(d.getHours())}${p(d.getMinutes())}`; };
function saveBlob(blob, name) {
  const a = document.createElement('a'); a.href = URL.createObjectURL(blob); a.download = name;
  document.body.appendChild(a); a.click(); setTimeout(() => { URL.revokeObjectURL(a.href); a.remove(); }, 15000);
}
async function exportFile() {
  const fmt = $('fmtSeg').querySelector('.on').dataset.v, R = S.result;
  if (fmt === 'hdr') return { blob: C.encodeHDR(R.rgb, R.W, R.H), name: stamp() + '.hdr', type: 'image/vnd.radiance' };
  setBusy(true, 'Writing EXR', 0);
  const blob = await C.encodeEXR(R.rgb, R.W, R.H, { half: fmt === 'exr16', onProgress: bar });
  setBusy(false);
  return { blob, name: stamp() + (fmt === 'exr16' ? '_half' : '') + '.exr', type: 'image/x-exr' };
}

async function openSession(id) {
  const s = await safe(DB.get('sessions', id), 'open'), recs = await safe(DB.shots(id), 'shots');
  if (!s || !recs || !recs.length) { toast('This capture has no saved shots'); return; }
  S.session = s; S.live = false; S.plan = s.plan; S.yaw0 = s.yaw0; S.lens = s.lens; S.k1 = s.k1 || 0; S.lensMeasured = !!s.lensMeasured;
  S.work = s.work; S.out = s.out || S.opts.out; S.sunCfg = s.sun || null; S.aligned = !!s.aligned;
  S.targets = (s.targets || []).map(t => ({ ...t, dir: C.dirFromYP(s.yaw0 + t.yaw, t.pitch) }));
  S.shots = recs.sort((a, b) => a.idx - b.idx).map(r => ({ idx: r.idx, rgbe: r.rgbe, w: r.w, h: r.h, M0: r.M, M: (s.alignedM && s.alignedM[r.idx]) || r.M,
                                                          target: r.target, clip: r.clip, gain: 1 }));
  S.nextIdx = Math.max(...S.shots.map(x => x.idx)) + 1;
  show('result'); setViewMode('flat');
  await processResult({ align: S.opts.align && !S.aligned });
}
function goHome() { Viewer.stopGyro(); show('home'); renderOptions(); renderGallery(); }

/* ============================ 360° viewer ============================ */
const Viewer = {
  gl: null, active: false, yaw: 0, pitch: 0, fov: 80, gyro: false,
  init() {
    if (this.gl) return true;
    const cv = $('gl'), gl = cv.getContext('webgl', { antialias: false });
    if (!gl) return false;
    const sh = (type, src) => { const s = gl.createShader(type); gl.shaderSource(s, src); gl.compileShader(s); return s; };
    const p = gl.createProgram();
    gl.attachShader(p, sh(gl.VERTEX_SHADER, 'attribute vec2 a;varying vec2 v;void main(){v=a;gl_Position=vec4(a,0.,1.);}'));
    gl.attachShader(p, sh(gl.FRAGMENT_SHADER, `precision highp float;varying vec2 v;uniform mat3 R;uniform float th,asp,y0;uniform sampler2D T;
      void main(){vec3 d=normalize(R*vec3(v.x*th*asp,v.y*th,-1.));float lon=atan(d.x,d.y)-y0;float lat=asin(clamp(d.z,-1.,1.));
      gl_FragColor=texture2D(T,vec2(lon/6.2831853+.5,.5-lat/3.14159265));}`));
    gl.linkProgram(p); gl.useProgram(p);
    const b = gl.createBuffer(); gl.bindBuffer(gl.ARRAY_BUFFER, b); gl.bufferData(gl.ARRAY_BUFFER, new Float32Array([-1, -1, 1, -1, -1, 1, 1, 1]), gl.STATIC_DRAW);
    const loc = gl.getAttribLocation(p, 'a'); gl.enableVertexAttribArray(loc); gl.vertexAttribPointer(loc, 2, gl.FLOAT, false, 0, 0);
    this.tex = gl.createTexture(); gl.bindTexture(gl.TEXTURE_2D, this.tex);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR); gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.REPEAT); gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
    this.u = { R: gl.getUniformLocation(p, 'R'), th: gl.getUniformLocation(p, 'th'), asp: gl.getUniformLocation(p, 'asp'), y0: gl.getUniformLocation(p, 'y0') };
    this.gl = gl;
    const pts = new Map(); let pinch = null;
    cv.addEventListener('pointerdown', e => { cv.setPointerCapture(e.pointerId); pts.set(e.pointerId, [e.clientX, e.clientY]); if (pts.size === 2) { const [a, c] = [...pts.values()]; pinch = { d: Math.hypot(a[0] - c[0], a[1] - c[1]), fov: this.fov }; } });
    cv.addEventListener('pointermove', e => {
      const prev = pts.get(e.pointerId); if (!prev) return;
      pts.set(e.pointerId, [e.clientX, e.clientY]);
      if (pts.size === 2 && pinch) { const [a, c] = [...pts.values()]; this.fov = C.clamp(pinch.fov * pinch.d / Math.max(10, Math.hypot(a[0] - c[0], a[1] - c[1])), 30, 115); }
      else if (!this.gyro) { const k = this.fov / cv.clientWidth; this.yaw -= (e.clientX - prev[0]) * k; this.pitch = C.clamp(this.pitch + (e.clientY - prev[1]) * k, -89, 89); }
      this.draw();
    });
    const up = e => { pts.delete(e.pointerId); if (pts.size < 2) pinch = null; };
    cv.addEventListener('pointerup', up); cv.addEventListener('pointercancel', up);
    cv.addEventListener('wheel', e => { e.preventDefault(); this.fov = C.clamp(this.fov * Math.exp(e.deltaY * 0.001), 30, 115); this.draw(); }, { passive: false });
    return true;
  },
  upload(rgba, w, h) { const gl = this.gl; if (!gl) return; gl.bindTexture(gl.TEXTURE_2D, this.tex); gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, w, h, 0, gl.RGBA, gl.UNSIGNED_BYTE, new Uint8Array(rgba.buffer)); this.draw(); },
  matrix() {
    if (this.gyro && S.orient && S.orient.M) return C.matMul(C.axisRot(2, -this.gyroOff * D2R), S.orient.M);
    return C.lookMat(this.yaw, this.pitch, 0);
  },
  draw() {
    const gl = this.gl, cv = $('gl'); if (!gl || !this.active) return;
    const dpr = Math.min(2, devicePixelRatio || 1), w = Math.round(cv.clientWidth * dpr), h = Math.round(cv.clientHeight * dpr);
    if (cv.width !== w || cv.height !== h) { cv.width = w; cv.height = h; }
    gl.viewport(0, 0, w, h);
    const M = this.matrix();
    gl.uniformMatrix3fv(this.u.R, false, new Float32Array([M[0], M[3], M[6], M[1], M[4], M[7], M[2], M[5], M[8]]));
    gl.uniform1f(this.u.th, Math.tan(this.fov / 2 * D2R)); gl.uniform1f(this.u.asp, w / h); gl.uniform1f(this.u.y0, S.yaw0 * D2R);
    gl.drawArrays(gl.TRIANGLE_STRIP, 0, 4);
  },
  async startGyro() {
    if (!S.orient) S.orient = new Orientation();
    if (!(await S.orient.start())) { toast('No gyro available'); return; }
    this.gyroOff = this.yaw - C.headingDeg(C.viewDir(S.orient.M)); this.gyro = true; $('btnGyro').classList.add('on');
    const tick = () => { if (!this.gyro || !this.active) return; this.draw(); requestAnimationFrame(tick); }; tick();
  },
  stopGyro() { this.gyro = false; $('btnGyro').classList.remove('on'); }
};
function setViewMode(mode) {
  $('viewSeg').querySelectorAll('button').forEach(b => b.classList.toggle('on', b.dataset.v === mode));
  const sphere = mode === 'sphere' && Viewer.init();
  Viewer.active = !!sphere;
  $('preview').classList.toggle('hidden', !!sphere); $('gl').classList.toggle('hidden', !sphere); $('btnGyro').classList.toggle('hidden', !sphere);
  if (sphere) { Viewer.yaw = S.yaw0; Viewer.pitch = 0; if (S.prevRGBA) Viewer.upload(S.prevRGBA, 2048, 1024); }
  else Viewer.stopGyro();
}

/* ============================ wiring ============================ */
function wire() {
  loadOpts(); bindOptions(); renderOptions(); renderSupport(); renderGallery();
  Merger.start();
  try { navigator.storage && navigator.storage.persist && navigator.storage.persist(); } catch (e) {}
  $('btnStart').onclick = () => enterCapture(false);
  $('btnBegin').onclick = () => begin();
  $('btnCancel').onclick = () => { if (S.cam) S.cam.stop(); S.cam = null; try { document.fullscreenElement && document.exitFullscreen(); } catch (e) {} goHome(); };
  $('btnShoot').onclick = () => { if (S.state === 'aim') captureShot(nearestTarget(C.viewDir(S.orient.M), 15)); };
  $('btnUndo').onclick = undo;
  $('btnFinish').onclick = finish;
  $('btnBack').onclick = goHome;
  $('prevEV').oninput = drawPreview;
  $('viewSeg').querySelectorAll('button').forEach(b => b.onclick = () => setViewMode(b.dataset.v));
  $('btnGyro').onclick = () => Viewer.gyro ? Viewer.stopGyro() : Viewer.startGyro();
  $('fmtSeg').querySelectorAll('button').forEach(b => b.onclick = () => {
    $('fmtSeg').querySelectorAll('button').forEach(x => x.classList.toggle('on', x === b));
    $('fmtNote').textContent = { exr32: '32-bit float, ZIP compressed. Best for Blender.',
      exr16: '16-bit half float: about half the size. Values above 65,504 (a rebuilt sun) are clamped, so use EXR 32 outdoors.',
      hdr: 'Radiance RGBE: smallest file, slightly less colour precision.' }[b.dataset.v];
  });
  $('btnSave').onclick = async () => { const f = await exportFile(); saveBlob(f.blob, f.name); };
  $('btnShare').onclick = async () => {
    try { const f = await exportFile(); await navigator.share({ files: [new File([f.blob], f.name, { type: f.type })], title: 'HDRI' }); }
    catch (e) { setBusy(false); if (e.name !== 'AbortError') toast('Sharing failed. Use Save instead.'); }
  };
  $('btnJpg').onclick = () => {
    const { W, H, rgb } = S.result, cv = document.createElement('canvas'); cv.width = W; cv.height = H;
    const ctx = cv.getContext('2d'), img = ctx.createImageData(W, H);
    C.tonemap(rgb, W * H, S.prevBase + +$('prevEV').value, img.data); ctx.putImageData(img, 0, 0);
    cv.toBlob(b => saveBlob(b, stamp() + '_preview.jpg'), 'image/jpeg', 0.92);
  };
  $('sunOn').onchange = () => { S.sunCfg.on = $('sunOn').checked; applySunNow(true); saveSession(); };
  $('sunStrength').oninput = () => { S.sunCfg.strength = +$('sunStrength').value; $('sunVal').textContent = S.sunCfg.strength + '×'; };
  $('sunStrength').onchange = () => { applySunNow(true); saveSession(); };
  $('resOutSeg').querySelectorAll('button').forEach(b => b.onclick = () => { S.out = +b.dataset.v; renderResultInfo(); });
  $('btnRestitch').onclick = async () => { S.lens = +$('resLens').value || S.lens; S.k1 = +$('resK1').value || 0; await stitchNow(); };
  $('btnAlign').onclick = async () => { S.lens = +$('resLens').value || S.lens; S.k1 = +$('resK1').value || 0; await processResult({ align: true }); };
  $('btnMore').onclick = () => { S.result = null; S.prev = null; setViewMode('flat'); enterCapture(true); };
  $('btnDelete').onclick = async e => {
    const b = e.currentTarget;
    if (!b.dataset.arm) { b.dataset.arm = 1; b.textContent = 'Tap again to delete'; setTimeout(() => { delete b.dataset.arm; b.textContent = 'Delete'; }, 3000); return; }
    delete b.dataset.arm; b.textContent = 'Delete';
    if (S.session) await safe(DB.delSession(S.session.id), 'delete');
    S.session = null; S.shots = []; S.result = null; goHome();
  };
  document.addEventListener('visibilitychange', () => { if (!document.hidden && S.screen === 'capture' && S.plan && S.state === 'aim') { applyLocks(); S.camIdx = -1; restoreBase(); } });
  window.addEventListener('resize', () => Viewer.draw());

  if (SIM) {   // drag to look around in the simulator
    const ov = $('overlay'); let drag = null;
    ov.addEventListener('pointerdown', e => { if (!S.orient || !S.orient.sim) return; drag = { x: e.clientX, y: e.clientY, yaw: S.orient.sim.yaw, pitch: S.orient.sim.pitch }; ov.setPointerCapture(e.pointerId); });
    ov.addEventListener('pointermove', e => { if (!drag) return; S.orient.sim.yaw = drag.yaw - (e.clientX - drag.x) * 0.15; S.orient.sim.pitch = C.clamp(drag.pitch + (e.clientY - drag.y) * 0.15, -90, 90); });
    ov.addEventListener('pointerup', () => drag = null);
    window.simLook = (yaw, pitch, roll = 0) => Object.assign(S.orient.sim, { yaw, pitch, roll });
  }
}
wire();
