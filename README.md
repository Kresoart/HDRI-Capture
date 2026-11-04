# HDRI Capture

A web app that captures 360° HDRIs with an Android phone in Chrome. It shoots bracketed exposures in every direction, uses the gyro to place each shot, merges the exposures into high dynamic range and stitches them into an equirectangular **32-bit float OpenEXR** (ZIP-compressed) or a Radiance `.hdr`.

**Use it:** open the GitHub Pages link in Chrome on Android, then allow camera and motion access.

## How it works
1. **Metering:** locks manual exposure, ISO and white balance where the browser allows it.
2. **Guided capture:** shows dots for a horizon ring, upper and lower rings, the zenith and the nadir. Hold the centre ring on a dot and the app shoots the bracket (3, 5 or 7 exposures, from −10 to +2 EV).
3. **Merge:** linearises each frame, measures the real exposure ratios from the image data, and does a weighted HDR merge.
4. **Stitch:** projects every direction into an equirectangular map with feathered blending. Gaps are filled with push-pull interpolation.
5. **Auto-fit lens:** searches for the field of view at which the overlapping shots agree best, which fixes seams.

## Camera support
- **Manual exposure (`exposureTime`)** gives the full bracket range. Most recent Android phones support this in Chrome.
- **Exposure compensation only** limits the range to about ±2 EV. Shots are balanced against each other afterwards, but the sun will clip.
- **No exposure control** gives a low dynamic range result.

## Test without a phone
Open `index.html?sim` (or `?sim=ev`) on a desktop. A synthetic HDR scene stands in for the camera, and you drag to look around.
