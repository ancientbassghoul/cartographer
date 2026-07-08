# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Working style

- **Always create a task list** at the start of any multi-step implementation, using the TaskCreate tool. Mark each task `in_progress` when you start it and `completed` as soon as it's done. This lets the user see live progress.
- Add log lines freely when diagnosing issues — the user is happy to re-run and share output.
- Never commit unless the user explicitly asks.

### PROGRESS.md — the resume/handoff file

`PROGRESS.md` is the single file to read first on resume and to keep current. It is three-fold: (1) a detailed memory note so Claude re-loads the project's large context after a clear; (2) short-term instructions to resume in-flight work; and (3) a record the user draws on to describe his path with this task (e.g. a presentation).

Because of (3), keep the **documentation** parts (the session log / what's been tried) VERY concise and narrative — "We wanted X. We tried Y. It failed because Z. So we tried W." — never the boring implementation details. The **up-next** part may be as detailed as needed; but once an up-next item is done or abandoned, translate it INTO the concise "tried that" one-liner style and move it to the documentation part. Detailed designs live in `plans/*.md` (referenced from PROGRESS.md), not inline.

### CRITICAL CODING STANDARD: NO SILENT FALLBACKS

You are strictly forbidden from implementing silent fallbacks, hidden try-except downgrades, or automatic failover mechanisms anywhere in this codebase. If a model, pipeline component, or hardware context fails to initialize or execute, the system must either fail-fast (crash with an explicit error) or explicitly update a visible state flag that is logged and exposed to the UI.

Apply the following rules to all design planning and code generation:

1. Architecture Review: If you identify a scenario where a fallback mechanism seems structurally beneficial (e.g., pivoting from MASt3R-SLAM to Feature VO if a build fails, or falling back from Qwen to DINOv2), you must present it to me as a design proposal first. Do not write the code until I explicitly approve the fallback logic.
2. Explicit State & Telemetry: Any approved fallback path must be completely transparent. The system state must explicitly track which path is active (e.g., `self.tracking_mode = "MASt3R"` vs `self.tracking_mode = "FEATURE_VO"`). 
3. Visible Alerts: When a fallback pathway is triggered at runtime, it must emit a critical log warning and modify a telemetry field that can be rendered in the visualizer overlay, so the operator immediately knows the system is running in a degraded or alternative state.
4. Fail-Fast Assertions: If an unapproved error or OOM condition occurs, prefer raising an explicit exception over gracefully absorbing the failure with a generic catch-all.

### IMAGE INTEGRITY AND RESOLUTION GUARDRAIL

Whenever any script or process processes an image as an input (whether it is a static reference template or a live video frame):
1. **No Silent Code Downscaling:** You are strictly forbidden from introducing arbitrary downscaling, cropping, or resolution caps in the script code (e.g., shrinking a 1280x720 frame to 700x392 to save compute) without explicitly proposing the change and validating it with the user first.
2. **Mandatory Model Preprocessing Disclosure:** If a model's native architecture strictly requires a specific input tensor size (like OWLv2's 960x960 grid or DINOv2's 14-pixel patch alignments), you must explicitly state this resolution transformation in the code documentation and logs. You must maximize the source asset's data fidelity before it enters the model processor.
3. If you are ever in doubt about whether a resolution change degrades the data, **STOP** and validate the execution parameters with the user.

### CRITICAL AUTONOMY STANDARD: NO MANUAL-FLIGHT DATA LEAKAGE INTO AUTONOMOUS LIMITS

You are strictly forbidden from using any specific value observed during a manual flight, a dry-run, or a recorded flight as a hardcoded limit, threshold, target, or trigger for the autonomous drone. The autonomous system must detect every condition (ceiling, wall, opening, obstacle, …) from GENERAL, SELF-CALIBRATING signals it computes LIVE in the current room — never a pre-known answer for a specific flight or room.

**The discriminator is room-specific ANSWER vs platform/signal BEHAVIOR — not "measured number vs not."**

1. **FORBIDDEN (a room-specific answer baked as a constant):** any value that encodes *the answer for THIS room/flight* — e.g. "stop ascending at altitude Y = −2.3", "176 frames forward until the wall", "stop after 4.2 s", "the ceiling is at Z = …", a precomputed target xyz, or seeding a detector with a measured plateau value. If a number encodes the answer for this room, it must not exist in the code — it must be detected LIVE.
2. **ALLOWED (general parameters + platform/signal characteristics):**
   - General robustness params that do NOT encode the answer — durations (e.g. a 1.5 s stall window), ratios (e.g. rate < 15 % of the LIVE-measured rise rate), noise floors, physics constants.
   - **Platform/signal CHARACTERISTICS** — properties of the drone/camera/physics that hold in ANY room, legitimately LEARNED (e.g. from `learn_to_fly.py`) and used: optical-flow SIGNATURES of events ("ceiling contact while ascending → vertical flow `dy_med` → ~0"; "wall contact while moving forward → looming radial `expansion` collapses from its live free-forward level → ~0; this ONE signal unifies a textureless wall that freezes the image AND a textured wall that shows a slow vertical climb"), and drone CONTROL DYNAMICS / maneuver magnitudes ("~N presses of `s` backs off a wall", "arm = tap `1` then hold ~10 frames", "press `c` to reset attitude BEFORE a forward push so a wall reads as a clean expansion-collapse", ramp rates). These generalize, and some (e.g. the back-off count) are impractical to calibrate live with SLAM running. Implemented in `flow_contact_detector.py` (detection, self-calibrating) + `flight_playbook.json` (control recipes).
3. **Best-practice rider:** even for a legitimate signature, prefer RELATIVE/self-calibrating use in the live detector where easy ("flow dropped below ~15 % of the ascent flow JUST measured in this climb") over a baked absolute. A learned platform constant is acceptable only where live calibration is impractical.
4. **Dry-run / manual logging / `learn_to_fly.py` are for VALIDATION + LEARNING ONLY.** Use them to confirm the detection LOGIC fires and to characterize platform signatures. A room-specific MEASURED value (this flight's ceiling altitude/frame) must NEVER be fed back as a constant into the live logic.
5. **Rationale:** the drone must generalize to any unseen room. Baking in one flight's answer is overfitting / cheating and defeats the autonomy. When in doubt whether a constant leaks the room's answer (vs a platform signature), **STOP** and validate with the user.

## Overview
The user is a candidate for an AI Assisted App Developer. This is the task he was given:

You are handed an unfamiliar room — the XLAB — and a drone streaming a single monocular video feed. Your objective is to design a stack that autonomously enters this environment, maps it, and returns the estimated 3D location of an object of interest inside the map it has built.
You have complete architectural freedom. We are looking for ingenuity, architectural clarity, and a tinkerer's instinct.
What You Get
•	The XLAB Unity build (identical to the ATLAS Jrs Exercise 002 environment).
•	sample_drone_interface.py — NDI video receiver + socket control channel (keyboard-equivalent commands to Unity).
•	That's it. No depth, no IMU, no pose telemetry, no ground-truth scale.

The Loop You Must Close
Phase 1 — Human Recon.
Fly the XLAB manually. Look around. Pick one object inside the lab that you will designate as your target. A weapon is the canonical choice, but any distinct, visually-groundable object is acceptable.
Phase 2 — Autonomous Survey.
The drone takes over. Your stack decides where to fly, when to turn, when to enter, and when to stop. Along the way it should accumulate enough visual evidence to represent the environment geometrically — and to recognize the target when it reappears.
Phase 3 — Localize & Report.
Output a map of the XLAB (2D occupancy, sparse 3D point cloud, topological layout — your call) with the estimated position of the target object marked inside it. Include an uncertainty or residual if your approach produces one.

Metric scale recovery is NOT required — internal consistency is what we inspect. Processing time and computational efficiency are NOT evaluated.

After this works, we'll have to wrap it in a nice GUI. Make it look like an app.
