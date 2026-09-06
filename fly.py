import json
import os
import glob
import time
import subprocess


def _check_for_crash_recovery(cartographer_dir, python_exe, diag_dir):
    """Session 55 (crash survivability): a hard machine bugcheck (a GPU-driver TDR, confirmed the cause
    of flight 20260902_165340's loss) bypasses every graceful-stop sentinel below AND the report-compile
    step at the bottom of this file -- so a run manifest written "in_progress" at launch and only ever
    marked "completed" down there is a reliable crash signal: if the NEXT launch finds one still
    "in_progress", that run never got a clean teardown. Also independently flags any un-finalized
    *_visualizer.mp4 (the video's own crash signature, in case an OLDER run predates this manifest).
    Prints a loud, explicit notice and OFFERS to run salvage_flight.py now -- never runs it silently
    (operator's call, matching CLAUDE.md's transparency requirement for anything touching prior-flight
    data)."""
    import sys
    sys.path.insert(0, cartographer_dir)
    from salvage_flight import is_mp4_finalized

    crashed_ts = set()
    for mp in glob.glob(os.path.join(diag_dir, "*_run.json")):
        try:
            with open(mp, "r", encoding="utf-8") as f:
                manifest = json.load(f)
        except (OSError, json.JSONDecodeError):
            continue
        if manifest.get("status") == "in_progress":
            crashed_ts.add(manifest.get("launch_ts") or os.path.basename(mp)[: -len("_run.json")])
    for mp in glob.glob(os.path.join(diag_dir, "*_visualizer.mp4")):
        if mp.endswith(".recovered.mp4") or os.path.exists(mp[: -len(".mp4")] + ".recovered.mp4"):
            continue    # already repaired earlier -- don't re-flag it forever
        try:
            if not is_mp4_finalized(mp):
                crashed_ts.add(os.path.basename(mp)[: -len("_visualizer.mp4")])
        except (OSError, ValueError):
            continue
    if not crashed_ts:
        return

    print("\n" + "=" * 78)
    print("[!] CRASH RECOVERY: the previous run did not shut down cleanly. Flight(s) needing salvage:")
    for ts in sorted(crashed_ts):
        print(f"      python salvage_flight.py {ts}")
    print("    (recovers the video + reconstructs a map backdrop if needed -- never touches an")
    print("     original artifact. Run any of the above yourself at any time, or answer below.)")
    print("=" * 78)
    try:
        answer = input("Salvage them now before launching this flight? [y/N] ").strip().lower()
    except EOFError:
        answer = "n"
    if answer == "y":
        for ts in sorted(crashed_ts):
            subprocess.run([python_exe, "salvage_flight.py", ts], cwd=cartographer_dir)


def main():
    cartographer_dir = r"d:\EXTEND\C2_SIM\XLAB\cartographer"
    xlab_dir = r"d:\EXTEND\C2_SIM\XLAB\XLAB"

    python_exe = os.path.join(cartographer_dir, "venv", "Scripts", "python.exe")
    xlab_exe = os.path.join(xlab_dir, "Xlab.exe")

    diag_dir = os.path.join(cartographer_dir, "OUTPUT", "diag")
    os.makedirs(diag_dir, exist_ok=True)
    _check_for_crash_recovery(cartographer_dir, python_exe, diag_dir)

    # Session 55: a run manifest, written "in_progress" now and marked "completed" only once the
    # teardown below actually finishes -- a hard crash leaves this stuck at "in_progress", which is
    # exactly the signal _check_for_crash_recovery looks for on the NEXT launch.
    from datetime import datetime as _dt
    launch_ts = _dt.now().strftime("%Y%m%d_%H%M%S")
    run_manifest_path = os.path.join(diag_dir, f"{launch_ts}_run.json")
    with open(run_manifest_path, "w", encoding="utf-8") as f:
        json.dump({"launch_ts": launch_ts, "pid": os.getpid(), "status": "in_progress"}, f)

    # Graceful-stop sentinel: the autopilot polls this path (--stop-file) and exits its loop CLEANLY when the
    # file appears, so its shutdown runs — emitting the replay MAP backdrop + closing the timeline — instead of
    # being hard-killed (which skips that shutdown and leaves the report with a blank scene). A parent can't
    # deliver a console Ctrl+C to a CREATE_NEW_CONSOLE child, so a file sentinel is the reliable stop channel.
    # Unique per run so a stale file can't pre-stop it; cleared up front just in case.
    stop_file = os.path.join(cartographer_dir, "OUTPUT", f".stop_{os.getpid()}")
    if os.path.exists(stop_file):
        os.remove(stop_file)
    # Same sentinel idea for perception_worker: it holds the SLAM map + point cloud in-process and
    # only exports it (the .ply/.npz/topdown) on a NORMAL loop exit -- a hard TerminateProcess (what
    # step 2 below does to everything else) skips that entirely. Own sentinel + own wait/terminate,
    # sequenced AFTER the autopilot's own clean shutdown (which still needs perception's published
    # pose/plan while it's flying its last leg).
    perception_stop_file = os.path.join(cartographer_dir, "OUTPUT", f".stop_perception_{os.getpid()}")
    if os.path.exists(perception_stop_file):
        os.remove(perception_stop_file)
    # Same again for visualizer: --record's cv2.VideoWriter only writes the MP4's moov atom (the
    # frame index -- without it the file is unplayable, confirmed by reproducing exactly that via a
    # hard TerminateProcess) on a normal loop exit. Nothing else depends on visualizer staying alive,
    # so its graceful-stop step can run in any order relative to autopilot's/perception's.
    visualizer_stop_file = os.path.join(cartographer_dir, "OUTPUT", f".stop_visualizer_{os.getpid()}")
    if os.path.exists(visualizer_stop_file):
        os.remove(visualizer_stop_file)

    # Only compile a report from a timeline THIS run produced (mtime after launch) — never a previous flight's.
    launch_t = time.time()

    autopilot = None            # handled separately from `processes`: it must stop CLEANLY (it writes the report)
    perception = None           # handled separately too: it must stop CLEANLY (exports the map/point cloud)
    visualizer = None           # handled separately too: it must stop CLEANLY (releases the --record video)
    processes = []              # every OTHER service (hard-terminated on teardown)
    print("[+] Launching background services into individual logging windows...")

    try:
        NEW_CONSOLE = subprocess.CREATE_NEW_CONSOLE   # separate window per service, just like a batch file
        # Session 62: --log turns on perception_worker's diag CSVs (per-frame SLAM/loop/phase timing).
        # It was never passed here, so pipe.enable_diag() never fired on a real flight and NO perception
        # timing CSV existed after 2026-06-26 -- the SLAM-choke table in STATE.md had to be reconstructed
        # from autopilot-side plan payloads. Matches how autopilot.py is launched on the next line.
        perception = subprocess.Popen([python_exe, "perception_worker.py", "--no-display", "--log", "--stop-file", perception_stop_file], cwd=cartographer_dir, creationflags=NEW_CONSOLE)
        # The autopilot writes the flight report; give it the stop-file so it can flush its map + timeline on exit.
        # Session 63: --finish-stops lets the autopilot end the data-collection half of the flight the
        # moment the survey is complete (config `end_at_postlude`), by touching the SAME two graceful-stop
        # sentinels this file's teardown uses. Perception flushes its map/.npz/.ply + CSVs and visualizer
        # releases the MP4 right then; the autopilot itself keeps holding a neutral hover, and everything
        # below still runs normally when the operator presses ENTER.
        autopilot = subprocess.Popen([python_exe, "autopilot.py", "--explore", "--log",
                                      "--stop-file", stop_file,
                                      "--finish-stops", f"{perception_stop_file},{visualizer_stop_file}"],
                                     cwd=cartographer_dir, creationflags=NEW_CONSOLE)
        visualizer = subprocess.Popen([python_exe, "visualizer.py", "--record", "--stop-file", visualizer_stop_file], cwd=cartographer_dir, creationflags=NEW_CONSOLE)
        processes.append(subprocess.Popen([python_exe, "io_bridge.py"], cwd=cartographer_dir, creationflags=NEW_CONSOLE))

        time.sleep(1.0)
        processes.append(subprocess.Popen([xlab_exe], cwd=xlab_dir))

        print("\n>>> STACK RUNNING - INSPECT LOG WINDOWS LIVE <<<")
        print(">>> Focus the io_bridge window and press [m] to hand control to the autopilot")
        print(">>>   (any manual flight key aborts autonomy back to manual).")
        input("Press [ENTER] here once the flight finishes to stop the stack and generate the report...\n")

    finally:
        # 1) Ask the autopilot to stop CLEANLY (drop the sentinel) and give it time to emit the map backdrop +
        #    close the timeline. Only hard-terminate as a last resort if it hangs.
        print("[+] Requesting a clean autopilot shutdown (flush the report + map backdrop)...")
        if autopilot is not None and autopilot.poll() is None:
            try:
                open(stop_file, "w").close()
            except OSError:
                pass
            try:
                autopilot.wait(timeout=15)
            except subprocess.TimeoutExpired:
                print("[!] Autopilot did not exit in time -> terminating (report may lack the map backdrop).")
                autopilot.terminate()

        # 1b) NOW ask perception to stop CLEANLY too (autopilot no longer needs its published state) and
        #     give it time to export the map/point cloud. Only hard-terminate as a last resort if it hangs.
        print("[+] Requesting a clean perception shutdown (flush the map + point cloud)...")
        if perception is not None and perception.poll() is None:
            try:
                open(perception_stop_file, "w").close()
            except OSError:
                pass
            try:
                perception.wait(timeout=15)
            except subprocess.TimeoutExpired:
                print("[!] Perception did not exit in time -> terminating (no map/point-cloud export).")
                perception.terminate()

        # 1c) Same for visualizer (--record's MP4 needs a clean release() to be playable).
        print("[+] Requesting a clean visualizer shutdown (flush the recording)...")
        if visualizer is not None and visualizer.poll() is None:
            try:
                open(visualizer_stop_file, "w").close()
            except OSError:
                pass
            try:
                visualizer.wait(timeout=15)
            except subprocess.TimeoutExpired:
                print("[!] Visualizer did not exit in time -> terminating (recording will be corrupted).")
                visualizer.terminate()

        # 2) Tear down the remaining services + the sim (reverse launch order: sim first, io_bridge last, so the
        #    autopilot's final HOLD above still had a live bus to publish onto).
        print("[+] Terminating the remaining flight-stack processes...")
        for p in reversed(processes):
            if p.poll() is None:
                p.terminate()

        for f in (stop_file, perception_stop_file, visualizer_stop_file):
            if os.path.exists(f):
                try:
                    os.remove(f)
                except OSError:
                    pass

        # 3) Compile the replay ONLY from a timeline this run produced (guards against reporting a stale flight
        #    if the autopilot never logged, e.g. it failed to start).
        timelines = [f for f in glob.glob(os.path.join(diag_dir, "*_timeline.jsonl"))
                     if os.path.getmtime(f) >= launch_t]
        if timelines:
            latest_log = max(timelines, key=os.path.getmtime)
            print(f"[+] Compiling report for: {os.path.basename(latest_log)}")
            subprocess.run([python_exe, "flight_replay.py", latest_log, "--open"], cwd=cartographer_dir)
        else:
            print("[!] No timeline was produced this run (autopilot never logged?) -> no report compiled.")

        # 4) Session 55: this teardown ran (a HARD crash would have skipped everything above), so the
        #    manifest is genuinely clean -- mark it, closing the crash-recovery window this run opened.
        try:
            with open(run_manifest_path, "w", encoding="utf-8") as f:
                json.dump({"launch_ts": launch_ts, "pid": os.getpid(), "status": "completed",
                          "timeline": (os.path.basename(latest_log) if timelines else None)}, f)
        except OSError:
            pass

if __name__ == "__main__":
    main()
