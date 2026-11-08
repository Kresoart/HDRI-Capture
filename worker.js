'use strict';
/* Merge engine: aligns the frames of one bracket, estimates exposure ratios and merges to HDR.
   Runs as a Web Worker; also loaded in the page as a fallback. */
if (typeof importScripts === 'function' && !self.HDRCore) importScripts('core.js');
self.MergeEngine = (() => {
  const C = self.HDRCore;
  let cfg = null, hist = [];
  function init(m) { cfg = m; hist = []; }
  function merge(m) {
    const n = m.w * m.h;
    const frames = m.frames.map(b => new Uint8ClampedArray(b));
    for (let k = 0; k < frames.length; k++) {
      if (k === m.ref) continue;
      const R = C.matMul(C.transpose(m.mats[k]), m.mats[m.ref]);
      if (C.rotAngle(R) > 0.08 * C.D2R) frames[k] = C.warpFrame(frames[k], m.w, m.h, R, m.f);
    }
    const ratios = C.estimateRatios(frames, n);
    let scales;
    if (cfg && cfg.global) {
      for (let k = 1; k < frames.length; k++) if (ratios[k] != null && m.idx[k] === m.idx[k - 1] + 1) (hist[m.idx[k - 1]] || (hist[m.idx[k - 1]] = [])).push(ratios[k]);
      const nom = cfg.nominal, c = [1];
      for (let j = 1; j < nom.length; j++) { const md = C.median(hist[j - 1]); c.push(c[j - 1] * (md != null ? md : nom[j] / nom[j - 1])); }
      const ladder = c.map(v => v / c[cfg.anchor] * nom[cfg.anchor]);
      scales = m.idx.map(j => ladder[j]);
    } else {
      scales = C.scalesFrom(ratios, m.idx.map(j => cfg.nominal[j]), m.ref);
    }
    const rgb = C.mergeBracket(frames, scales, n, m.ref);
    const rgbe = C.toRGBE(rgb, n);
    return { type: 'merged', id: m.id, rgbe, clip: C.LIN[250] / scales[0], frames: frames.length };
  }
  return { init, merge };
})();
if (typeof WorkerGlobalScope !== 'undefined' && self instanceof WorkerGlobalScope) {
  self.onmessage = e => {
    const m = e.data;
    if (m.type === 'init') { self.MergeEngine.init(m); return; }
    if (m.type === 'merge') {
      try { const r = self.MergeEngine.merge(m); self.postMessage(r, [r.rgbe.buffer]); }
      catch (err) { self.postMessage({ type: 'error', id: m.id, message: String(err && err.message || err) }); }
    }
  };
}
