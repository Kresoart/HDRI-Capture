# HDRI Capture

A web app that captures 360° HDRIs with an Android phone in Chrome and exports a **32-bit float OpenEXR** (or 16-bit half / Radiance `.hdr`) ready for Blender.

**Use it:** open https://kresoart.github.io/HDRI-Capture/ in Chrome on Android, then allow camera and motion access.

## What it does
- **Adaptive brackets (Auto):** each direction shoots only the exposures it needs. Darker frames are added until nothing clips, brighter frames until shadows are clean. **Manual** mode sets the count (2–9), EV step and range centre yourself.
- **Fast capture:** the next frame's brightness is predicted, so a new exposure is grabbed as soon as it arrives. The base exposure stays set between directions, merging runs in a background worker, and auto-shoot triggers after about 0.2 s of stillness.
- **Lens measuring:** one small turn at the start measures the field of view, so the main camera and the ultra-wide both work. The ultra-wide needs about 8–10 directions instead of about 32.
- **More highlight range:** once the fastest shutter is reached, the darkest frames drop to the lowest ISO. Each stop of ISO the scene was metered above minimum becomes an extra stop of headroom for lamps and the sun.
- **Handheld friendly:** the frames of each bracket are re-aligned using the gyro, and anything that moved between the frames of a bracket is rejected (deghosting). After capture, auto-align refines lens, distortion and every shot's rotation from the overlaps, and each shot's brightness is matched to its neighbours.
- **Clean seams:** each part of the sphere comes from the shot that saw it best. Overlaps switch over in a narrow band instead of being averaged, so slightly misaligned details are cut at a seam rather than doubled. Clipped shots give way to shots that measured the light.
- **Sun recovery:** a phone can't capture the sun without clipping. The app detects the clipped disc and rebuilds it at a chosen strength relative to the sky (clear ≈ 8×).
- **360° preview** with drag, pinch and gyro look-around.
- **Saved captures:** shots are stored on the phone (IndexedDB) as you go. Reopen, re-align or re-export them later.

## Blender
World → Surface → Environment Texture → open the `.exr`. The centre of the image faces the first direction you shot.

## Camera support
- **Manual exposure (`exposureTime`):** the full range. Most recent Samsung, Pixel and other phones support this on the main camera.
- **Exposure compensation only:** about ±2 EV. Shots are balanced against each other afterwards, but highlights clip.
- **No exposure control:** a low dynamic range result.

## Develop / test without a phone
Serve the folder (e.g. `python3 -m http.server`) and open `index.html?sim`. A synthetic HDR scene stands in for the camera; drag to look around.
Options: `?sim=ev` (limited exposure camera), `&lens=13` (ultra-wide), `&sun=200000` (clipped sun), `&drift=0.1` (gyro drift in °/s), `&mover=1` (an object moving during capture). In the console, `simLook(yaw, pitch, roll, yawErr, pitchErr, rollErr)` aims the simulated phone and adds a gyro error.

Files: `index.html` (layout), `style.css`, `app.js` (UI, camera, capture flow), `core.js` (HDR math, stitching, alignment, EXR writer), `worker.js` (background merge).
