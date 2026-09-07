# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Working style

- **Always create a task list** at the start of any multi-step implementation, using the TaskCreate tool. Mark each task `in_progress` when you start it and `completed` as soon as it's done. This lets the user see live progress.
- Add log lines freely when diagnosing issues — the user is happy to re-run and share output.
- Never commit unless the user explicitly asks. **When the user does ask to commit: right before creating the commit, update both `PROGRESS.md` and `STATE.md` to reflect current state, then create the commit, then push.** This push is pre-authorized by this standing rule specifically for this update→commit→push sequence — do not stop to ask permission for the push itself each time; the user has explicitly asked for it to be automatic. (Ordinary git safety practice still applies: review what's staged, and stop and ask if anything looks destructive or off-scope beyond this normal commit.)

### STATE.md + PROGRESS.md — the resume/handoff files

**`STATE.md`** holds **the ONE thing being worked on right now**, and nothing else: the current status, the immediate next step, and a pointer to the standing rules. **Hard cap ~120 lines.** If an item is not the live item, it does not belong here — it belongs in `PROGRESS.md`'s backlog. This file is read at the start of every session, so every line in it is paid for repeatedly.

**`PROGRESS.md`** is the full session-by-session history and presentation record — read it only when `STATE.md` doesn't have enough depth, or when reconstructing the "why" behind a past decision (e.g. for a presentation). It is two-fold: (1) a detailed memory note so Claude can re-load deep project context after a clear if `STATE.md` isn't enough, and (2) a record the user draws on to describe his path with this task.

Keep the **documentation** parts of `PROGRESS.md` (the session log / what's been tried) VERY concise and narrative — "We wanted X. We tried Y. It failed because Z. So we tried W." — never the boring implementation details. Detailed designs live in `plans/*.md` (referenced from PROGRESS.md), not inline. `STATE.md`'s live-item section may be as detailed as needed; but once that item is done or abandoned, translate it INTO the concise "tried that" one-liner style and move it into `PROGRESS.md`'s session log, then refresh `STATE.md` with whatever is genuinely next. Open work that is NOT next goes to `PROGRESS.md`'s `## Future (backlog)`; measurement tables go to `## Reference — don't re-derive`. **The same content must never sit in both files** — one of them owns it, the other links to it.

**Every plan MUST end with three closing steps, always the last items in the task list:** (1) **update `STATE.md` and `PROGRESS.md`** — fold completed/abandoned work into `PROGRESS.md`'s concise narrative, refresh `STATE.md`'s resume pointer, and reference any new `plans/*.md`; (2) **get ready for a context clear** — leave the tree, `STATE.md`, and `PROGRESS.md` in a clean, self-describing state so the next session can resume cold from `STATE.md` alone (self-tests noted, loose ends captured, nothing important living only in this conversation). (3) **PRUNE — delete what this session answered.** See the rule below. Treat these three steps as non-negotiable — a plan is not complete until they are done.

### The pruning rule (added session 68, after `STATE.md` reached 431 lines)

These files bloat for a structural reason: **the workflow has an append step and no delete step.** Adding a watch item costs a paragraph; closing one costs a flight plus a log read. So the cheap operation ran every session and the expensive one almost never did — by session 68, `STATE.md` carried watch lists from sessions 56, 57 and 61 all still marked "flown, never reviewed" (thirteen sessions of sediment), and its headline had been stale and actively misleading for two sessions because nobody re-reads the top of a 400-line file.

Therefore:

1. **Every watch item MUST name the event that closes it** — "check on the next flight", "check on the next replay", "check when X is built". A watch item with no closing event is not a watch item; it is a wish, and it does not get written down.
2. **When that event happens, the item is DELETED from wherever it lives.** It resolves to exactly one of three outcomes, all of which remove it: *confirmed* → one line in `PROGRESS.md`'s session log, item deleted; *broken* → becomes a real bug with its own plan, item deleted; *not observed* → deleted, and the session log says it was never observed. "Still open, watch again next time" is NOT an outcome — that is how sediment forms.
3. **Three-session limit.** If an item has survived three sessions without anyone looking at it, it was not real. Delete it, or promote it to a dated backlog entry with an owner event. Do not carry it silently.
4. **Prune before you append.** At the start of the closing steps, re-read what is already in `STATE.md` and ask of each line: *is this still the live item?* Anything that is not gets moved or deleted BEFORE the new session's content is written. Never append to a file you have not just pruned.
5. **If `STATE.md` exceeds ~120 lines, the pruning step failed** — stop and fix it rather than committing a longer file. Same for a stale headline: if `STATE.md`'s top section describes a problem that has since been solved or abandoned, that is a bug of the same class as shipping dead code.

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
