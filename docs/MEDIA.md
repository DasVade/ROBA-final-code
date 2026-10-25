# Selected real hardware media

All photos and original video frames were supplied by the owner. All 17 archive figures were viewed; the three real assembly photos were inspected before selection. The source archive was reconciled with the existing public repository: the driver station and final report are byte-identical; the archive Final_Code firmware matches the repository after newline normalization (the byte differences are line endings). The repository sketches remain the build baseline. Original sources and credits are unchanged.

Selected images: `real2.jpg` → robot-cover.jpg, `real.jpg` → robot-detail.jpg, `model.jpg` → cad-model.jpg, `circuit.jpg` → circuit.jpg. Images are orientation-corrected, resized proportionally and JPEG-compressed for display. The original files remain in the local source archive; no synthetic imagery was added. No team-list or competition bracket figure is published in the new presentation.

Videos received together on October 24, 2026 were inspected at eight timestamps spanning each clip before captioning. `f9ad7077e4d004b857de5500ef0f0b52.mp4` → wall-traversal.mp4 (32.6 s): robot travels beside the right arena boundary and turns. `f38e2fc6ee08b249e4880a0488aac80d.mp4` → arena-maneuver.mp4 (17.9 s): corridor travel and a heading change. `5a1a08aa0adf212177431299c01d56f6.mp4` → contact-sequence.mp4 (16.2 s): approach and repeated contact motion near a field target.

Transcodes retain full clips and timing, H.264/yuv420p at 720 px height with proportional width, CRF 24, fast-start metadata, and no audio. Posters are actual frames at 1 s. Originals remain untouched at the supplied local paths; video binaries are hosted only in the website repository, not duplicated in the firmware repository. Media provenance hashes and original dimensions are recorded in the source repository's docs/media_manifest.json.

No active-mode telemetry is visible. Captions therefore describe observed motion, not measured localization accuracy, closed-loop success rates or guaranteed task completion. The original report's YouTube links remain available separately; no unsupported one-to-one identification between those public uploads and these local recordings is asserted.

The new architecture SVG is a source-based diagram, not a photograph or simulation. UART carries distance data; I²C is the optional TopHat packet-count link. The supplied report's drawing is retained separately and not silently treated as the final wiring specification.
