'use strict';
/* HDRI Capture core: math and image processing, no DOM. Runs in the page and in the merge worker. */
self.HDRCore = (() => {
  const D2R = Math.PI / 180;
  const LIN = new Float32Array(256);
  for (let i = 0; i < 256; i++) { const c = i / 255; LIN[i] = c <= 0.04045 ? c / 12.92 : Math.pow((c + 0.055) / 1.055, 2.4); }
  const EXP = new Float64Array(256);
  for (let e = 0; e < 256; e++) EXP[e] = Math.pow(2, e - 136);
  const SRGB_LUT = new Uint8Array(4096);
  for (let i = 0; i < 4096; i++) { const v = i / 4095; const s = v <= 0.0031308 ? v * 12.92 : 1.055 * Math.pow(v, 1 / 2.4) - 0.055; SRGB_LUT[i] = Math.max(0, Math.min(255, Math.round(s * 255))); }
  const toSrgb8 = v => !(v > 0) ? 0 : v >= 1 ? 255 : SRGB_LUT[(v * 4095 + 0.5) | 0];
  const clamp = (v, a, b) => v < a ? a : v > b ? b : v;
  const median = a => { if (!a || !a.length) return null; const s = Float64Array.from(a).sort(); return s[s.length >> 1]; };
  const yieldUI = () => new Promise(r => setTimeout(r, 0));
  const lum = (r, g, b) => 0.2126 * r + 0.7152 * g + 0.0722 * b;

  /* ---------- rotations: row-major 3x3, device -> world (world: x east, y north, z up) */
  function quatToMat(q) {
    const [x, y, z, w] = q;
    return [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w),
            2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
            2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)];
  }
  function eulerToMat(alpha, beta, gamma) {
    const a = alpha * D2R, b = beta * D2R, g = gamma * D2R;
    const ca = Math.cos(a), sa = Math.sin(a), cb = Math.cos(b), sb = Math.sin(b), cg = Math.cos(g), sg = Math.sin(g);
    return [ca * cg - sa * sb * sg, -sa * cb, ca * sg + sa * sb * cg,
            sa * cg + ca * sb * sg, ca * cb, sa * sg - ca * sb * cg,
            -cb * sg, sb, cb * cg];
  }
  function matMul(a, b) {
    const o = new Array(9);
    for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) o[i * 3 + j] = a[i * 3] * b[j] + a[i * 3 + 1] * b[3 + j] + a[i * 3 + 2] * b[6 + j];
    return o;
  }
  const transpose = m => [m[0], m[3], m[6], m[1], m[4], m[7], m[2], m[5], m[8]];
  function axisRot(axis, a) {
    const c = Math.cos(a), s = Math.sin(a);
    if (axis === 0) return [1, 0, 0, 0, c, -s, 0, s, c];
    if (axis === 1) return [c, 0, s, 0, 1, 0, -s, 0, c];
    return [c, -s, 0, s, c, 0, 0, 0, 1];
  }
  const rotAngle = R => Math.acos(clamp((R[0] + R[4] + R[8] - 1) / 2, -1, 1));
  function dirFromYP(yawDeg, pitchDeg) {
    const y = yawDeg * D2R, p = pitchDeg * D2R;
    return [Math.cos(p) * Math.sin(y), Math.cos(p) * Math.cos(y), Math.sin(p)];
  }
  function lookMat(yawDeg, pitchDeg, rollDeg = 0) {
    const y = yawDeg * D2R, r = rollDeg * D2R, f = dirFromYP(yawDeg, pitchDeg);
    const R = [Math.cos(y), -Math.sin(y), 0];
    const U = [R[1] * f[2] - R[2] * f[1], R[2] * f[0] - R[0] * f[2], R[0] * f[1] - R[1] * f[0]];
    const cr = Math.cos(r), sr = Math.sin(r);
    const R2 = [0, 1, 2].map(i => R[i] * cr + U[i] * sr), U2 = [0, 1, 2].map(i => -R[i] * sr + U[i] * cr);
    return [R2[0], U2[0], -f[0], R2[1], U2[1], -f[1], R2[2], U2[2], -f[2]];
  }
  const viewDir = M => [-M[2], -M[5], -M[8]];
  const angleBetween = (a, b) => Math.acos(clamp(a[0] * b[0] + a[1] * b[1] + a[2] * b[2], -1, 1));
  const headingDeg = d => Math.atan2(d[0], d[1]) / D2R;
  const focalPx = (longSidePx, lensMM) => longSidePx * lensMM / 34.6;   // 34.6 mm: long side of a 4:3 frame with full-frame diagonal
  const fovDeg = (sidePx, f) => 2 * Math.atan(sidePx / 2 / f) / D2R;

  /* ---------- capture plan */
  function makeTargets(hfov, vfov, ov, yaw0 = 0) {
    const T = [];
    const add = (yaw, pitch, ring) => T.push({ yaw, pitch, ring, dir: dirFromYP(yaw0 + yaw, pitch), done: false });
    const count = pitch => { const Le = Math.max(0, Math.abs(pitch) - vfov / 2); return Math.max(3, Math.ceil(360 / (hfov * ov / Math.cos(Le * D2R)) - 1e-6)); };
    const ring = (pitch, idx) => { const n = count(pitch), off = (idx % 2) ? 180 / n : 0; for (let i = 0; i < n; i++) add(off + i * 360 / n, pitch, idx); };
    // vertical half-coverage of the horizon ring midway between two frames
    const n0 = count(0), t = Math.tan(vfov / 2 * D2R), m = Math.tan(180 / n0 * D2R);
    const ve = Math.atan(t / Math.sqrt(1 + m * m)) / D2R;
    const zenBottom = 90 - Math.min(hfov, vfov) / 2, margin = 0.06 * Math.min(hfov, vfov);
    const reach = zenBottom + margin - ve;            // pitch the top ring must reach
    const nr = reach > 0.25 * ve ? Math.ceil(reach / (2 * ve - margin)) : 0;
    ring(0, 0);
    for (let k = 1; k <= nr; k++) ring(k * reach / nr, k);
    add(0, 90, 99);
    for (let k = 1; k <= nr; k++) ring(-k * reach / nr, k);
    add(0, -90, -99);
    return T;
  }

  /* ---------- frames */
  function clipStats(F, n) {          // fraction of clipped and of very dark pixels (subsampled)
    let clip = 0, dark = 0, cnt = 0;
    for (let i = 0; i < n; i += 5) {
      const p = i * 4; if (!F[p + 3]) continue;
      const r = F[p], g = F[p + 1], b = F[p + 2], m = r > g ? (r > b ? r : b) : (g > b ? g : b);
      if (m >= 250) clip++; if (m <= 16) dark++; cnt++;
    }
    return { clip: clip / Math.max(1, cnt), dark: dark / Math.max(1, cnt) };
  }
  // re-project frame (taken at rotation Mk) into the reference camera (Mr): R = Mk^T Mr
  function warpFrame(src, w, h, R, f) {
    const out = new Uint8ClampedArray(w * h * 4), cx = (w - 1) / 2, cy = (h - 1) / 2;
    for (let y = 0; y < h; y++) {
      const ry = -(y - cy) / f;
      for (let x = 0; x < w; x++) {
        const rx = (x - cx) / f;
        const tx = R[0] * rx + R[1] * ry - R[2], ty = R[3] * rx + R[4] * ry - R[5], tz = R[6] * rx + R[7] * ry - R[8];
        if (tz > -1e-6) continue;
        const px = cx + f * tx / -tz, py = cy - f * ty / -tz;
        if (!(px >= 0 && py >= 0 && px <= w - 1 && py <= h - 1)) continue;
        let x0 = px | 0, y0 = py | 0; if (x0 > w - 2) x0 = w - 2; if (y0 > h - 2) y0 = h - 2;
        const fx = px - x0, fy = py - y0, i = (y0 * w + x0) * 4, j = i + w * 4, o = (y * w + x) * 4;
        const a = (1 - fx) * (1 - fy), b = fx * (1 - fy), c = (1 - fx) * fy, d = fx * fy;
        out[o] = src[i] * a + src[i + 4] * b + src[j] * c + src[j + 4] * d;
        out[o + 1] = src[i + 1] * a + src[i + 5] * b + src[j + 1] * c + src[j + 5] * d;
        out[o + 2] = src[i + 2] * a + src[i + 6] * b + src[j + 2] * c + src[j + 6] * d;
        out[o + 3] = 255;
      }
    }
    return out;
  }
  function estimateRatios(frames, n) {
    const out = [1];
    for (let k = 1; k < frames.length; k++) {
      const a = frames[k - 1], b = frames[k], vals = [];
      for (let i = 0; i < n; i += 3) {
        const p = i * 4; if (!a[p + 3] || !b[p + 3]) continue;
        const za = a[p + 1], zb = b[p + 1];
        if (za >= 12 && za <= 215 && zb >= 25 && zb <= 215 && a[p] < 245 && a[p + 2] < 245 && b[p] < 245 && b[p + 2] < 245) vals.push(LIN[zb] / LIN[za]);
      }
      out.push(vals.length >= 200 ? median(vals) : null);
    }
    return out;
  }
  // merge a bracket; ref = index of the reference frame used to reject ghosts (moving things, misaligned frames)
  function mergeBracket(frames, scales, n, ref = -1) {
    const K = frames.length, out = new Float32Array(n * 3), inv = scales.map(s => 1 / s), rej = new Uint8Array(K);
    const good = (F, p) => { const r = F[p], g = F[p + 1], b = F[p + 2], m = r > g ? (r > b ? r : b) : (g > b ? g : b); return F[p + 3] && m < 240 && r + g + b > 60; };
    const rad = (F, p, k) => (LIN[F[p]] + LIN[F[p + 1]] + LIN[F[p + 2]]) * inv[k];
    for (let i = 0; i < n; i++) {
      const p = i * 4; let r = 0, g = 0, b = 0, ws = 0;
      if (ref >= 0 && K > 1) {
        rej.fill(0);
        const t0 = good(frames[ref], p) ? rad(frames[ref], p, ref) : 0;
        for (let dir = -1; dir <= 1; dir += 2) {
          let trust = t0;
          for (let k = ref + dir; k >= 0 && k < K; k += dir) {
            const F = frames[k];
            if (!good(F, p)) continue;
            const v = rad(F, p, k);
            if (trust > 0 && Math.abs(Math.log(v / trust)) > 0.35) rej[k] = 1; else trust = v;
          }
        }
      }
      for (let k = 0; k < K; k++) {
        if (rej[k]) continue;
        const F = frames[k]; if (!F[p + 3]) continue;
        const zr = F[p], zg = F[p + 1], zb = F[p + 2];
        const zmax = zr > zg ? (zr > zb ? zr : zb) : (zg > zb ? zg : zb), zavg = (zr + zg + zb) / 765;
        let w = 1;
        if (k > 0) { if (zmax >= 245) continue; if (zmax > 215) w *= (245 - zmax) / 30; }
        if (k < K - 1 && zavg < 0.1) w *= Math.max(zavg, 0.001) / 0.1;
        w *= (0.2 + 0.8 * (1 - Math.abs(2 * zavg - 1))) * scales[k];
        const s = inv[k];
        r += w * LIN[zr] * s; g += w * LIN[zg] * s; b += w * LIN[zb] * s; ws += w;
      }
      if (ws > 0) { out[i * 3] = r / ws; out[i * 3 + 1] = g / ws; out[i * 3 + 2] = b / ws; }
      else if (ref >= 0) { const F = frames[ref], s = inv[ref]; out[i * 3] = LIN[F[p]] * s; out[i * 3 + 1] = LIN[F[p + 1]] * s; out[i * 3 + 2] = LIN[F[p + 2]] * s; }
    }
    return out;
  }
  function scalesFrom(ratios, nominal, anchor) {
    const c = [1];
    for (let k = 1; k < nominal.length; k++) c.push(c[k - 1] * (ratios[k] != null ? ratios[k] : nominal[k] / nominal[k - 1]));
    return c.map(v => v / c[anchor] * nominal[anchor]);
  }
  function toRGBE(rgb, n) {
    const out = new Uint8Array(n * 4);
    for (let i = 0; i < n; i++) {
      const r = rgb[i * 3], g = rgb[i * 3 + 1], b = rgb[i * 3 + 2];
      const m = r > g ? (r > b ? r : b) : (g > b ? g : b);
      if (!(m > 1e-32)) continue;
      let e = Math.ceil(Math.log2(m)), sc = Math.pow(2, -e) * 256;
      if (m * sc >= 256) { e++; sc *= 0.5; }
      if (e < -127) continue; if (e > 127) { e = 127; sc = Math.pow(2, -e) * 256; }
      out[i * 4] = Math.min(255, r * sc) | 0; out[i * 4 + 1] = Math.min(255, g * sc) | 0; out[i * 4 + 2] = Math.min(255, b * sc) | 0; out[i * 4 + 3] = e + 128;
    }
    return out;
  }
  function rgbeLumAt(D, i) { const e = D[i * 4 + 3]; if (!e) return 0; const s = EXP[e]; return lum((D[i * 4] + .5) * s, (D[i * 4 + 1] + .5) * s, (D[i * 4 + 2] + .5) * s); }
  // linear luminance image, box-downsampled by integer factor
  function grayFrom(rgba, w, h, fac) {
    const W = Math.floor(w / fac), H = Math.floor(h / fac), out = new Float32Array(W * H);
    for (let y = 0; y < H; y++) for (let x = 0; x < W; x++) {
      let s = 0;
      for (let yy = 0; yy < fac; yy++) for (let xx = 0; xx < fac; xx++) { const p = ((y * fac + yy) * w + x * fac + xx) * 4; s += lum(LIN[rgba[p]], LIN[rgba[p + 1]], LIN[rgba[p + 2]]); }
      out[y * W + x] = s / (fac * fac);
    }
    return { g: out, w: W, h: H };
  }
  // lens (35 mm eq.) from two frames with known rotation
  function lensFromPair(A, B, MA, MB) {
    const R = matMul(transpose(MA), MB), w = A.w, h = A.h, cx = (w - 1) / 2, cy = (h - 1) / 2, long = Math.max(w, h);
    const evalF = F => {
      const f = focalPx(long, F); let e = 0, n = 0, tot = 0;
      for (let y = 2; y < h - 2; y += 2) for (let x = 2; x < w - 2; x += 2) {
        tot++;
        const rx = (x - cx) / f, ry = -(y - cy) / f;
        const tx = R[0] * rx + R[1] * ry - R[2], ty = R[3] * rx + R[4] * ry - R[5], tz = R[6] * rx + R[7] * ry - R[8];
        if (tz > -1e-3) continue;
        const px = cx + f * tx / -tz, py = cy - f * ty / -tz;
        if (!(px >= 0 && py >= 0 && px < w - 1 && py < h - 1)) continue;
        const x0 = px | 0, y0 = py | 0, fx = px - x0, fy = py - y0, i = y0 * w + x0;
        const a = A.g[i] * (1 - fx) * (1 - fy) + A.g[i + 1] * fx * (1 - fy) + A.g[i + w] * (1 - fx) * fy + A.g[i + w + 1] * fx * fy;
        e += Math.min(1, Math.abs(Math.log(a + 0.003) - Math.log(B.g[y * w + x] + 0.003))); n++;
      }
      return n > tot * 0.12 ? e / n : Infinity;
    };
    const scan = [];
    for (let i = 0; i <= 44; i++) { const F = 9 * Math.pow(80 / 9, i / 44); scan.push([F, evalF(F)]); }
    let best = scan.reduce((a, b) => b[1] < a[1] ? b : a);
    const med = median(scan.map(s => s[1]).filter(isFinite));
    for (let F = best[0] * 0.95; F <= best[0] * 1.05; F += best[0] * 0.005) { const e = evalF(F); if (e < best[1]) best = [F, e]; }
    const conf = med ? 1 - best[1] / med : 0;
    return { lens: +best[0].toFixed(1), error: best[1], conf };
  }

  /* ---------- projection / sampling (with radial distortion k1) */
  function prep(shots, f, useGain = true, k1 = 0) {
    return shots.map(s => {
      const M = s.M, cx = (s.w - 1) / 2, cy = (s.h - 1) / 2;
      const halfDiag = Math.atan(Math.hypot(cx, cy) / f) * (1 + Math.max(0, -k1));
      return { M, v0: -M[2], v1: -M[5], v2: -M[8], cosLim: Math.cos(Math.min(halfDiag + 0.03, 3.0)), cx, cy, f, k1,
               rn2: f * f / (cx * cx + cy * cy), w: s.w, h: s.h, D: s.rgbe, gain: useGain && s.gain ? s.gain : 1,
               clip: (s.clip > 0 && isFinite(s.clip) ? s.clip : Infinity) * (useGain && s.gain ? s.gain : 1), shot: s };
    });
  }
  function sample(q, dx, dy, dz, o) {
    if (dx * q.v0 + dy * q.v1 + dz * q.v2 < q.cosLim) return 0;
    const M = q.M, tz = M[2] * dx + M[5] * dy + M[8] * dz;
    if (tz > -1e-6) return 0;
    const iz = -1 / tz;
    let u = (M[0] * dx + M[3] * dy + M[6] * dz) * iz, v = (M[1] * dx + M[4] * dy + M[7] * dz) * iz;
    if (q.k1) { const s = 1 + q.k1 * (u * u + v * v) * q.rn2; u *= s; v *= s; }
    const px = q.cx + q.f * u, py = q.cy - q.f * v, w = q.w, h = q.h;
    if (!(px >= 0 && py >= 0 && px <= w - 1 && py <= h - 1)) return 0;
    const ex = Math.min(px, w - 1 - px) / q.cx, ey = Math.min(py, h - 1 - py) / q.cy;
    const wt = ex * ey; if (wt <= 0) return 0;
    let x0 = px | 0, y0 = py | 0; if (x0 > w - 2) x0 = w - 2; if (y0 > h - 2) y0 = h - 2;
    const fx = px - x0, fy = py - y0, D = q.D;
    let r = 0, g = 0, b = 0, i = (y0 * w + x0) * 4, e, s;
    e = D[i + 3]; if (e) { s = EXP[e] * (1 - fx) * (1 - fy); r += (D[i] + .5) * s; g += (D[i + 1] + .5) * s; b += (D[i + 2] + .5) * s; }
    i += 4; e = D[i + 3]; if (e) { s = EXP[e] * fx * (1 - fy); r += (D[i] + .5) * s; g += (D[i + 1] + .5) * s; b += (D[i + 2] + .5) * s; }
    i += w * 4 - 4; e = D[i + 3]; if (e) { s = EXP[e] * (1 - fx) * fy; r += (D[i] + .5) * s; g += (D[i + 1] + .5) * s; b += (D[i + 2] + .5) * s; }
    i += 4; e = D[i + 3]; if (e) { s = EXP[e] * fx * fy; r += (D[i] + .5) * s; g += (D[i + 1] + .5) * s; b += (D[i + 2] + .5) * s; }
    o[0] = r * q.gain; o[1] = g * q.gain; o[2] = b * q.gain; o[3] = px; o[4] = py; o[5] = ex < ey ? ex : ey;
    return wt * wt;
  }
  function eqDir(x, y, W, H, yaw0Rad) {
    const lon = ((x + 0.5) / W - 0.5) * 2 * Math.PI + yaw0Rad, lat = (0.5 - (y + 0.5) / H) * Math.PI;
    return [Math.cos(lat) * Math.sin(lon), Math.cos(lat) * Math.cos(lon), Math.sin(lat)];
  }

  // Stitching: each output pixel blends the shots that see it with weight edgeDistance^power. A high power gives a
  // narrow transition halfway through each overlap, so misaligned details are cut at a seam instead of
  // showing up twice. Clipped shots give way to shots that measured the light. power = 1 is a classic wide feather.
  async function stitch(shots, opt) {
    const W = opt.outW, H = opt.outH, yaw0 = (opt.yaw0 || 0) * D2R, P = opt.blend === 'feather' ? 0 : (opt.power || 24);
    const Q = prep(shots, opt.f, true, opt.k1 || 0);
    const out = new Float32Array(W * H * 3), cov = new Uint8Array(W * H), o = new Float64Array(6);
    const MAXC = 24, cw = new Float64Array(MAXC), crad = new Float64Array(MAXC * 3), csat = new Uint8Array(MAXC);
    const sl = new Float64Array(W), cl = new Float64Array(W);
    for (let x = 0; x < W; x++) { const lon = ((x + 0.5) / W - 0.5) * 2 * Math.PI + yaw0; sl[x] = Math.sin(lon); cl[x] = Math.cos(lon); }
    let tick = Date.now();
    for (let y = 0; y < H; y++) {
      const lat = (0.5 - (y + 0.5) / H) * Math.PI, cz = Math.cos(lat), dz = Math.sin(lat);
      for (let x = 0; x < W; x++) {
        const dx = cz * sl[x], dy = cz * cl[x], i = (y * W + x) * 3;
        let n = 0, anyUnsat = false;
        for (let k = 0; k < Q.length && n < MAXC; k++) {
          const q = Q[k];
          if (dx * q.v0 + dy * q.v1 + dz * q.v2 < q.cosLim) continue;
          const w0 = sample(q, dx, dy, dz, o);
          if (!(w0 > 0)) continue;
          cw[n] = P ? Math.pow(o[5], P) : w0;
          crad[n * 3] = o[0]; crad[n * 3 + 1] = o[1]; crad[n * 3 + 2] = o[2];
          const sat = lum(o[0], o[1], o[2]) >= 0.9 * q.clip; csat[n] = sat ? 1 : 0; if (!sat) anyUnsat = true;
          n++;
        }
        if (!n) continue;
        cov[y * W + x] = 1;
        let r = 0, g = 0, b = 0, ws = 0;
        if (!anyUnsat) {   // clipped in every shot: keep the one that measured the most light
          let best = 0, bl = -1; for (let k = 0; k < n; k++) { const L = lum(crad[k * 3], crad[k * 3 + 1], crad[k * 3 + 2]); if (L > bl) { bl = L; best = k; } }
          out[i] = crad[best * 3]; out[i + 1] = crad[best * 3 + 1]; out[i + 2] = crad[best * 3 + 2]; continue;
        }
        for (let k = 0; k < n; k++) { const w = csat[k] ? cw[k] * 1e-6 : cw[k]; r += w * crad[k * 3]; g += w * crad[k * 3 + 1]; b += w * crad[k * 3 + 2]; ws += w; }
        if (ws > 0) { out[i] = r / ws; out[i + 1] = g / ws; out[i + 2] = b / ws; }
      }
      if (Date.now() - tick > 50) { opt.onProgress && opt.onProgress(y / H); await yieldUI(); tick = Date.now(); }
    }
    let holes = 0; for (let i = 0; i < W * H; i++) if (!cov[i]) holes++;
    if (holes) fillHoles(out, cov, W, H);
    return { rgb: out, holes: holes / (W * H) };
  }
  function fillHoles(img, cov, W, H) {
    const levels = [{ c: img, a: Float32Array.from(cov), W, H }];
    while (true) {
      const L = levels[levels.length - 1];
      if (L.W <= 1 && L.H <= 1) break;
      const W2 = Math.max(1, Math.ceil(L.W / 2)), H2 = Math.max(1, Math.ceil(L.H / 2));
      const c = new Float32Array(W2 * H2 * 3), a = new Float32Array(W2 * H2);
      for (let y = 0; y < L.H; y++) for (let x = 0; x < L.W; x++) {
        const i = y * L.W + x, w = L.a[i]; if (!w) continue;
        const j = (y >> 1) * W2 + (x >> 1);
        c[j * 3] += L.c[i * 3] * w; c[j * 3 + 1] += L.c[i * 3 + 1] * w; c[j * 3 + 2] += L.c[i * 3 + 2] * w; a[j] += w;
      }
      let missing = 0;
      for (let j = 0; j < W2 * H2; j++) { if (a[j]) { c[j * 3] /= a[j]; c[j * 3 + 1] /= a[j]; c[j * 3 + 2] /= a[j]; a[j] = 1; } else missing++; }
      levels.push({ c, a, W: W2, H: H2 });
      if (!missing) break;
    }
    for (let l = levels.length - 2; l >= 0; l--) {
      const L = levels[l], P = levels[l + 1];
      for (let y = 0; y < L.H; y++) for (let x = 0; x < L.W; x++) {
        const i = y * L.W + x; if (L.a[i]) continue;
        const j = (y >> 1) * P.W + (x >> 1);
        L.c[i * 3] = P.c[j * 3]; L.c[i * 3 + 1] = P.c[j * 3 + 1]; L.c[i * 3 + 2] = P.c[j * 3 + 2]; L.a[i] = 1;
      }
    }
  }

  /* ---------- alignment */
  function gridSamples(shots, f, gw, gh, minW, useGain, k1) {
    const Q = prep(shots, f, useGain, k1), o = new Float64Array(6), res = [];
    for (let y = 0; y < gh; y++) for (let x = 0; x < gw; x++) {
      const d = eqDir(x, y, gw, gh, 0); let list = null;
      for (let k = 0; k < Q.length; k++) {
        const w = sample(Q[k], d[0], d[1], d[2], o);
        if (w > minW) { const L = lum(o[0], o[1], o[2]); if (L > 1e-6 && L < 0.9 * Q[k].clip) (list || (list = [])).push(k, Math.log(L)); }
      }
      if (list && list.length >= 4) res.push(list);
    }
    return res;
  }
  function harmonizeGains(shots, f, k1 = 0) {
    const N = shots.length, S = gridSamples(shots, f, 360, 180, 0.01, false, k1), pairs = new Map();
    for (const l of S) for (let a = 0; a < l.length; a += 2) for (let b = a + 2; b < l.length; b += 2) {
      let i = l[a], j = l[b], d = l[a + 1] - l[b + 1]; if (i > j) { [i, j] = [j, i]; d = -d; }
      const key = i * N + j; let arr = pairs.get(key); if (!arr) pairs.set(key, arr = []); arr.push(d);
    }
    const edges = [];
    for (const [key, arr] of pairs) if (arr.length >= 30) edges.push([Math.floor(key / N), key % N, median(arr)]);
    const G = new Float64Array(N);
    for (let it = 0; it < 400; it++) {
      const sum = new Float64Array(N), cnt = new Float64Array(N);
      for (const [i, j, d] of edges) { sum[i] += G[j] - d; cnt[i]++; sum[j] += G[i] + d; cnt[j]++; }
      for (let i = 0; i < N; i++) if (cnt[i]) G[i] = 0.5 * G[i] + 0.5 * sum[i] / cnt[i];
      let m = 0; for (let i = 0; i < N; i++) m += G[i]; m /= N; for (let i = 0; i < N; i++) G[i] -= m;
    }
    return Array.from(G, g => Math.exp(g));
  }
  function lensError(shots, longSide, lens, k1, gw = 360, gh = 180) {
    const S = gridSamples(shots, focalPx(longSide, lens), gw, gh, 0.02, true, k1);
    let err = 0, n = 0;
    for (const l of S) for (let a = 0; a + 3 < l.length; a += 2) { err += Math.min(1, Math.abs(l[a + 1] - l[a + 3])); n++; }
    return n >= 300 ? err / n : Infinity;
  }
  async function fitLensK(shots, longSide, lens0, k10, opt = {}) {
    let F = lens0, k = k10 || 0, best = lensError(shots, longSide, F, k), done = 0;
    const lin = (a, b, n) => Array.from({ length: n }, (_, i) => a + (b - a) * i / (n - 1));
    const plan = [];
    if (opt.wide) plan.push(['F', () => Array.from({ length: 25 }, (_, i) => lens0 * 0.6 * Math.pow(2.6, i / 24))]);
    if (!opt.fine) { plan.push(['F', () => lin(F * 0.94, F * 1.06, 9)]); plan.push(['k', () => [-0.12, -0.08, -0.04, 0, 0.04, 0.08, 0.12]]); }
    plan.push(['F', () => lin(F * 0.98, F * 1.02, 9)]); plan.push(['k', () => lin(k - 0.03, k + 0.03, 7)]);
    const total = plan.reduce((s, p) => s + p[1]().length, 0);
    for (const [what, gen] of plan) for (const v of gen()) {
      const e = what === 'F' ? lensError(shots, longSide, v, k) : lensError(shots, longSide, F, v);
      if (e < best) { best = e; if (what === 'F') F = v; else k = v; }
      done++; opt.onProgress && opt.onProgress(done / total); await yieldUI();
    }
    return { lens: +F.toFixed(2), k1: +k.toFixed(3), error: best };
  }
  // per-shot rotation refinement against overlapping neighbours (fixes gyro drift / handheld error)
  async function refineRotations(shots, f, k1, onProgress, passes = 2) {
    const Q = prep(shots, f, true, k1), N = Q.length, o = new Float64Array(6);
    const half = Q.map(q => Math.acos(q.cosLim));
    const nb = Q.map((q, i) => { const l = []; for (let j = 0; j < N; j++) if (j !== i && q.v0 * Q[j].v0 + q.v1 * Q[j].v1 + q.v2 * Q[j].v2 > Math.cos(Math.min(Math.PI, half[i] + half[j]))) l.push(j); return l; });
    const PT = shots.map((s, i) => {
      const q = Q[i], gx = 34, gy = Math.max(8, Math.round(gx * s.h / s.w)), arr = [];
      for (let yy = 0; yy < gy; yy++) for (let xx = 0; xx < gx; xx++) {
        const px = Math.round((xx + 0.5) * (s.w - 1) / gx), py = Math.round((yy + 0.5) * (s.h - 1) / gy);
        const L = rgbeLumAt(s.rgbe, py * s.w + px) * q.gain; if (!(L > 1e-6)) continue;
        const xd = (px - q.cx) / f, yd = -(py - q.cy) / f; let u = xd, v = yd;
        if (k1) for (let it = 0; it < 4; it++) { const sc = 1 + k1 * (u * u + v * v) * q.rn2; u = xd / sc; v = yd / sc; }
        const n = Math.hypot(u, v, 1); arr.push(u / n, v / n, -1 / n, Math.log(L));
      }
      return Float64Array.from(arr);
    });
    const err = i => {
      const M = Q[i].M, P = PT[i], list = nb[i]; let e = 0, n = 0;
      for (let p = 0; p < P.length; p += 4) {
        const rx = P[p], ry = P[p + 1], rz = P[p + 2];
        const dx = M[0] * rx + M[1] * ry + M[2] * rz, dy = M[3] * rx + M[4] * ry + M[5] * rz, dz = M[6] * rx + M[7] * ry + M[8] * rz;
        for (let t = 0; t < list.length; t++) {
          const w = sample(Q[list[t]], dx, dy, dz, o);
          if (w > 0.003) { const L = lum(o[0], o[1], o[2]); if (L > 1e-6) { e += Math.min(0.6, Math.abs(Math.log(L) - P[p + 3])); n++; } }
        }
      }
      return n >= 40 ? e / n : null;
    };
    const setM = (i, M) => { const q = Q[i]; q.M = M; q.v0 = -M[2]; q.v1 = -M[5]; q.v2 = -M[8]; shots[i].M = M; };
    let done = 0;
    for (let pass = 0; pass < passes; pass++) for (let i = 0; i < N; i++) {
      let base = err(i);
      if (base != null) for (const step of [0.6, 0.3, 0.15, 0.07]) {
        let improved = true, guard = 0;
        while (improved && guard++ < 3) {
          improved = false;
          for (let ax = 0; ax < 3; ax++) for (const sg of [1, -1]) {
            const prev = Q[i].M; setM(i, matMul(prev, axisRot(ax, sg * step * D2R)));
            const e = err(i);
            if (e != null && e < base - 1e-5) { base = e; improved = true; } else setM(i, prev);
          }
        }
      }
      done++; onProgress && onProgress(done / (N * passes)); await yieldUI();
    }
  }

  /* ---------- sun recovery */
  function detectSun(rgb, W, H, clip, yaw0Deg = 0) {
    const thr = clip * 0.85, yMax = Math.floor(H * 0.55), n = W * yMax, mask = new Uint8Array(n);
    let any = 0;
    for (let i = 0; i < n; i++) if (lum(rgb[i * 3], rgb[i * 3 + 1], rgb[i * 3 + 2]) >= thr) { mask[i] = 1; any++; }
    if (!any) return null;
    const dA = (2 * Math.PI / W) * (Math.PI / H), seen = new Uint8Array(n), stack = [];
    let best = null;
    for (let s = 0; s < n; s++) {
      if (!mask[s] || seen[s]) continue;
      let om = 0, sx = 0, sy = 0, sz = 0; stack.push(s); seen[s] = 1;
      while (stack.length) {
        const i = stack.pop(), y = (i / W) | 0, x = i - y * W;
        const d = eqDir(x, y, W, H, yaw0Deg * D2R), w = Math.cos((0.5 - (y + 0.5) / H) * Math.PI) * dA;
        om += w; sx += d[0] * w; sy += d[1] * w; sz += d[2] * w;
        const nbs = [y * W + (x + 1) % W, y * W + (x + W - 1) % W, y > 0 ? i - W : -1, y < yMax - 1 ? i + W : -1];
        for (const j of nbs) if (j >= 0 && mask[j] && !seen[j]) { seen[j] = 1; stack.push(j); }
      }
      if (!best || om > best.omega) { const L = Math.hypot(sx, sy, sz); best = { omega: om, dir: [sx / L, sy / L, sz / L] }; }
    }
    best.radiusDeg = Math.sqrt(best.omega / Math.PI) / D2R;
    best.thr = thr;
    best.elevation = Math.asin(best.dir[2]) / D2R;
    if (best.radiusDeg > 12 || best.elevation < -3) return null;
    return best;
  }
  // paints a sun disc (0.5° radius) whose normal irradiance = strength x sky horizontal irradiance
  function applySun(R, sun, strength) {
    const { rgb, W, H } = R;
    if (R.sunPatch) { for (const p of R.sunPatch) { rgb[p[0]] = p[1]; rgb[p[0] + 1] = p[2]; rgb[p[0] + 2] = p[3]; } R.sunPatch = null; }
    if (!sun || !(strength > 0)) return null;
    const dA = (2 * Math.PI / W) * (Math.PI / H);
    let E = 0;
    for (let y = 0; y < H / 2; y++) {
      const lat = (0.5 - (y + 0.5) / H) * Math.PI, dO = Math.cos(lat) * dA, sL = Math.sin(lat);
      for (let x = 0; x < W; x++) { const i = (y * W + x) * 3, L = lum(rgb[i], rgb[i + 1], rgb[i + 2]); if (L < sun.thr) E += L * sL * dO; }
    }
    const En = strength * E, rho = 0.5 * D2R, cR = Math.cos(rho), sd = sun.dir;
    const sunLat = Math.asin(sd[2]), list = []; let om = 0;
    const y0 = Math.max(0, Math.floor((0.5 - (sunLat + rho * 1.5) / Math.PI) * H)), y1 = Math.min(H - 1, Math.ceil((0.5 - (sunLat - rho * 1.5) / Math.PI) * H));
    for (let y = y0; y <= y1; y++) for (let x = 0; x < W; x++) {
      const d = eqDir(x, y, W, H, R.yaw0 * D2R);
      if (d[0] * sd[0] + d[1] * sd[1] + d[2] * sd[2] >= cR) { list.push(y * W + x); om += Math.cos((0.5 - (y + 0.5) / H) * Math.PI) * dA; }
    }
    if (!list.length) return null;
    const L = En / om, c = [1.03, 0.99, 0.93], cl = lum(...c), patch = [];
    for (const p of list) { const i = p * 3; patch.push([i, rgb[i], rgb[i + 1], rgb[i + 2]]); for (let k = 0; k < 3; k++) rgb[i + k] = Math.max(rgb[i + k], L * c[k] / cl); }
    R.sunPatch = patch;
    return { E, En, L };
  }

  /* ---------- previews */
  function downsample(rgb, W, H, w, h) {
    const out = new Float32Array(w * h * 3), cnt = new Float32Array(w * h);
    for (let y = 0; y < H; y++) { const yy = Math.min(h - 1, (y * h / H) | 0);
      for (let x = 0; x < W; x++) { const xx = Math.min(w - 1, (x * w / W) | 0), j = yy * w + xx, i = (y * W + x) * 3;
        out[j * 3] += rgb[i]; out[j * 3 + 1] += rgb[i + 1]; out[j * 3 + 2] += rgb[i + 2]; cnt[j]++; } }
    for (let j = 0; j < w * h; j++) if (cnt[j]) { out[j * 3] /= cnt[j]; out[j * 3 + 1] /= cnt[j]; out[j * 3 + 2] /= cnt[j]; }
    return out;
  }
  function autoEV(rgb) {
    const L = []; for (let i = 0; i < rgb.length; i += 3 * 7) L.push(lum(rgb[i], rgb[i + 1], rgb[i + 2]));
    const m = median(L.filter(v => v > 0)) || 0.18; return Math.log2(0.18 / m);
  }
  function tonemap(rgb, n, ev, rgba) {
    const k = Math.pow(2, ev);
    const tm = x => { x *= k; return (x * (2.51 * x + 0.03)) / (x * (2.43 * x + 0.59) + 0.14); };
    for (let i = 0; i < n; i++) { rgba[i * 4] = toSrgb8(tm(rgb[i * 3])); rgba[i * 4 + 1] = toSrgb8(tm(rgb[i * 3 + 1])); rgba[i * 4 + 2] = toSrgb8(tm(rgb[i * 3 + 2])); rgba[i * 4 + 3] = 255; }
  }

  /* ---------- writers */
  async function deflate(u8) {
    const s = new Blob([u8]).stream().pipeThrough(new CompressionStream('deflate'));
    return new Uint8Array(await new Response(s).arrayBuffer());
  }
  const _f = new Float32Array(1), _u = new Uint32Array(_f.buffer);
  function toHalf(v) {
    if (!(v > -65504)) v = -65504; if (!(v < 65504)) v = v === v ? 65504 : 0;
    _f[0] = v; const x = _u[0], sign = (x >>> 16) & 0x8000; let e = ((x >>> 23) & 0xff) - 112, m = x & 0x7fffff;
    if (e <= 0) { if (e < -10) return sign; m = (m | 0x800000) >> (1 - e); return sign | ((m + 0x1000) >> 13); }
    return sign | ((e << 10) + ((m + 0x1000) >> 13));
  }
  async function encodeEXR(rgb, W, H, opt = {}) {
    const zip = opt.zip !== false && typeof CompressionStream !== 'undefined', half = !!opt.half, bps = half ? 2 : 4;
    const bytes = [], enc = new TextEncoder();
    const str = s => { for (const b of enc.encode(s)) bytes.push(b); bytes.push(0); };
    const i32 = v => bytes.push(...new Uint8Array(new Int32Array([v]).buffer));
    const f32 = v => bytes.push(...new Uint8Array(new Float32Array([v]).buffer));
    const u8 = v => bytes.push(v & 255);
    bytes.push(0x76, 0x2f, 0x31, 0x01); i32(2);
    const attr = (name, type, size, fn) => { str(name); str(type); i32(size); fn(); };
    attr('channels', 'chlist', 3 * 18 + 1, () => { for (const c of ['B', 'G', 'R']) { str(c); i32(half ? 1 : 2); u8(0); u8(0); u8(0); u8(0); i32(1); i32(1); } u8(0); });
    attr('compression', 'compression', 1, () => u8(zip ? 3 : 0));
    attr('dataWindow', 'box2i', 16, () => { i32(0); i32(0); i32(W - 1); i32(H - 1); });
    attr('displayWindow', 'box2i', 16, () => { i32(0); i32(0); i32(W - 1); i32(H - 1); });
    attr('lineOrder', 'lineOrder', 1, () => u8(0));
    attr('pixelAspectRatio', 'float', 4, () => f32(1));
    attr('screenWindowCenter', 'v2f', 8, () => { f32(0); f32(0); });
    attr('screenWindowWidth', 'float', 4, () => f32(1));
    u8(0);
    const header = new Uint8Array(bytes);
    const lpb = zip ? 16 : 1, nChunks = Math.ceil(H / lpb), chunks = new Array(nChunks);
    const encodeChunk = async c => {
      const y0 = c * lpb, lines = Math.min(lpb, H - y0), raw = new Uint8Array(lines * W * 3 * bps), dv = new DataView(raw.buffer);
      let p = 0;
      for (let l = 0; l < lines; l++) { const row = (y0 + l) * W * 3;
        for (const ch of [2, 1, 0]) for (let x = 0; x < W; x++) {
          const v = rgb[row + x * 3 + ch];
          if (half) { dv.setUint16(p, toHalf(v), true); p += 2; } else { dv.setFloat32(p, v, true); p += 4; }
        } }
      let data = raw;
      if (zip) {
        const n = raw.length, t = new Uint8Array(n); let a = 0, b = (n + 1) >> 1;
        for (let i = 0; i < n; i += 2) { t[a++] = raw[i]; if (i + 1 < n) t[b++] = raw[i + 1]; }
        let prev = t[0];
        for (let i = 1; i < n; i++) { const cur = t[i]; t[i] = (cur - prev + 384) & 255; prev = cur; }
        const z = await deflate(t);
        if (z.length < n) data = z;
      }
      const ch = new Uint8Array(8 + data.length), cdv = new DataView(ch.buffer);
      cdv.setInt32(0, y0, true); cdv.setInt32(4, data.length, true); ch.set(data, 8);
      chunks[c] = ch;
    };
    for (let c = 0; c < nChunks; c += 16) {
      const jobs = []; for (let k = c; k < Math.min(nChunks, c + 16); k++) jobs.push(encodeChunk(k));
      await Promise.all(jobs); opt.onProgress && opt.onProgress(Math.min(1, (c + 16) / nChunks));
    }
    const table = new Uint8Array(nChunks * 8), tdv = new DataView(table.buffer);
    let off = header.length + table.length;
    for (let c = 0; c < nChunks; c++) { tdv.setBigUint64(c * 8, BigInt(off), true); off += chunks[c].length; }
    return new Blob([header, table, ...chunks], { type: 'image/x-exr' });
  }
  function encodeHDR(rgb, W, H) {
    const rgbe = toRGBE(rgb, W * H), parts = [new TextEncoder().encode(`#?RADIANCE\n# HDRI Capture\nFORMAT=32-bit_rle_rgbe\nEXPOSURE=1.0\n\n-Y ${H} +X ${W}\n`)];
    if (W < 8 || W > 32767) { parts.push(rgbe); return new Blob(parts, { type: 'image/vnd.radiance' }); }
    const buf = new Uint8Array(W * 5 + 8), comp = new Uint8Array(W);
    for (let y = 0; y < H; y++) {
      let p = 0; buf[p++] = 2; buf[p++] = 2; buf[p++] = W >> 8; buf[p++] = W & 255;
      for (let c = 0; c < 4; c++) {
        for (let x = 0; x < W; x++) comp[x] = rgbe[(y * W + x) * 4 + c];
        let cur = 0;
        while (cur < W) {
          let beg = cur, run = 0, oldRun = 0;
          while (run < 4 && beg < W) { beg += run; oldRun = run; run = 1; while (beg + run < W && run < 127 && comp[beg] === comp[beg + run]) run++; }
          if (oldRun > 1 && oldRun === beg - cur) { buf[p++] = 128 + oldRun; buf[p++] = comp[cur]; cur = beg; }
          while (cur < beg) { const nr = Math.min(128, beg - cur); buf[p++] = nr; for (let i = 0; i < nr; i++) buf[p++] = comp[cur + i]; cur += nr; }
          if (run >= 4) { buf[p++] = 128 + run; buf[p++] = comp[beg]; cur += run; }
        }
      }
      parts.push(buf.slice(0, p));
    }
    return new Blob(parts, { type: 'image/vnd.radiance' });
  }

  /* ---------- synthetic scene (simulator / tests) */
  const fract = v => v - Math.floor(v);
  const hash = i => fract(Math.sin(i * 12.9898 + 4.1) * 43758.5453);
  const sim = { sun: 1800, sunDir: dirFromYP(40, 30) };
  function simEnv(dx, dy, dz, o) {
    const lon = Math.atan2(dx, dy), lat = Math.asin(clamp(dz, -1, 1)), S = sim.sunDir;
    if (sim.mover) { let dl = lon - sim.mlon; dl -= Math.round(dl / (2 * Math.PI)) * 2 * Math.PI;
      if (Math.abs(dl) < 0.06 && lat > 0.0 && lat < 0.12) { o[0] = 2.5; o[1] = 0.25; o[2] = 0.2; return; } }
    if (dz >= 0) {
      const ca = dx * S[0] + dy * S[1] + dz * S[2];
      const idx = Math.floor((lon + Math.PI) / (2 * Math.PI) * 24), hgt = 0.06 + 0.3 * hash(idx);
      if (lat < hgt) {
        const base = 0.08 + 0.35 * hash(idx + 7);
        const u = fract((lon + Math.PI) / (2 * Math.PI) * 24 * 5), v = fract(lat * 45);
        if (u > 0.3 && u < 0.7 && v > 0.3 && v < 0.7 && hash(idx * 31 + Math.floor(lat * 45)) > 0.55) { o[0] = 6; o[1] = 4.2; o[2] = 2; return; }
        o[0] = base * 1.1; o[1] = base; o[2] = base * 0.85; return;
      }
      if (ca > Math.cos(0.025)) { o[0] = sim.sun; o[1] = sim.sun * 0.95; o[2] = sim.sun * 0.85; return; }
      const t = Math.sqrt(dz), glow = 3 * Math.exp(-(1 - ca) * 120);
      const cloud = Math.max(0, Math.sin(lon * 7 + dz * 13) * Math.sin(lon * 3 - dz * 21)) * 0.6;
      o[0] = 1.6 + (0.35 - 1.6) * t + glow + cloud; o[1] = 1.7 + (0.55 - 1.7) * t + glow * 0.95 + cloud; o[2] = 1.9 + (1.1 - 1.9) * t + glow * 0.85 + cloud;
      return;
    }
    const px = dx / -dz, py = dy / -dz, dist = Math.hypot(px, py);
    let v = ((Math.floor(px * 1.5) + Math.floor(py * 1.5)) & 1) ? 0.3 : 0.06;
    if (dist > 12) v = 0.18 + (v - 0.18) * Math.max(0, 1 - (dist - 12) / 12);
    o[0] = v * 1.05; o[1] = v; o[2] = v * 0.9;
  }
  function simRender(M, f, w, h, k, out, noise = 0, seed = 1) {
    const o = new Float64Array(6), cx = (w - 1) / 2, cy = (h - 1) / 2; let s = seed >>> 0;
    const rnd = () => { s = (s * 1664525 + 1013904223) >>> 0; return s / 4294967296 - 0.5; };
    for (let y = 0; y < h; y++) for (let x = 0; x < w; x++) {
      const ux = (x - cx) / f, uy = -(y - cy) / f;
      let dx = M[0] * ux + M[1] * uy - M[2], dy = M[3] * ux + M[4] * uy - M[5], dz = M[6] * ux + M[7] * uy - M[8];
      const L = Math.hypot(dx, dy, dz); dx /= L; dy /= L; dz /= L;
      simEnv(dx, dy, dz, o);
      const i = (y * w + x) * 4, nz = noise ? 1 + noise * rnd() : 1;
      out[i] = toSrgb8(o[0] * k * nz); out[i + 1] = toSrgb8(o[1] * k * nz); out[i + 2] = toSrgb8(o[2] * k * nz); out[i + 3] = 255;
    }
  }

  return { D2R, LIN, EXP, clamp, median, lum, quatToMat, eulerToMat, matMul, transpose, axisRot, rotAngle, dirFromYP, lookMat, viewDir,
           angleBetween, headingDeg, focalPx, fovDeg, makeTargets, clipStats, warpFrame, estimateRatios, mergeBracket, scalesFrom, toRGBE,
           rgbeLumAt, grayFrom, lensFromPair, prep, sample, eqDir, stitch, fillHoles, gridSamples, harmonizeGains, lensError, fitLensK,
           refineRotations, detectSun, applySun, downsample, autoEV, tonemap, toHalf, encodeEXR, encodeHDR, sim, simEnv, simRender, toSrgb8, yieldUI };
})();
