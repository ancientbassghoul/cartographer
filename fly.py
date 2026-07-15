import os
import glob
import time
import subprocess

def main():
    cartographer_dir = r"d:\EXTEND\C2_SIM\XLAB\cartographer"
    xlab_dir = r"d:\EXTEND\C2_SIM\XLAB\XLAB"

    python_exe = os.path.join(cartographer_dir, "venv", "Scripts", "python.exe")
    xlab_exe = os.path.join(xlab_dir, "Xlab.exe")

    # Graceful-stop sentinel: the autopilot polls this path (--stop-file) and exits its loop CLEANLY when the
    # file appears, so its shutdown runs — emitting the replay MAP backdrop + closing the timeline — instead of
    # being hard-killed (which skips that shutdown and leaves the report with a blank scene). A parent can't
    # deliver a console Ctrl+C to a CREATE_NEW_CONSOLE child, so a file sentinel is the reliable stop channel.
    # Unique per run so a stale file can't pre-stop it; cleared up front just in case.
    stop_file = os.path.join(cartographer_dir, "OUTPUT", f".stop_{os.getpid()}")
    if os.path.exists(stop_file):
        os.remove(stop_file)

    # Only compile a report from a timeline THIS run produced (mtime after launch) — never a previous flight's.
    launch_t = time.time()

    autopilot = None            # handled separately from `processes`: it must stop CLEANLY (it writes the report)
    processes = []              # every OTHER service (hard-terminated on teardown)
    print("[+] Launching background services into individual logging windows...")

    try:
        NEW_CONSOLE = subprocess.CREATE_NEW_CONSOLE   # separate window per service, just like a batch file
        processes.append(subprocess.Popen([python_exe, "perception_worker.py", "--no-display"], cwd=cartographer_dir, creationflags=NEW_CONSOLE))
        # The autopilot writes the flight report; give it the stop-file so it can flush its map + timeline on exit.
        autopilot = subprocess.Popen([python_exe, "autopilot.py", "--explore", "--log", "--stop-file", stop_file], cwd=cartographer_dir, creationflags=NEW_CONSOLE)
        processes.append(subprocess.Popen([python_exe, "visualizer.py"], cwd=cartographer_dir, creationflags=NEW_CONSOLE))
        # --log-commands: always-on outgoing-packet log (post-ramp, MANUAL vs AUTO) so every flight leaves
        # a diffable record of the stick smoothing (session 18). Cheap; NullLog-equivalent overhead when idle.
        processes.append(subprocess.Popen([python_exe, "io_bridge.py", "--log-commands"], cwd=cartographer_dir, creationflags=NEW_CONSOLE))

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

        # 2) Tear down the remaining services + the sim (reverse launch order: sim first, io_bridge last, so the
        #    autopilot's final HOLD above still had a live bus to publish onto).
        print("[+] Terminating the remaining flight-stack processes...")
        for p in reversed(processes):
            if p.poll() is None:
                p.terminate()

        if os.path.exists(stop_file):
            try:
                os.remove(stop_file)
            except OSError:
                pass

        # 3) Compile the replay ONLY from a timeline this run produced (guards against reporting a stale flight
        #    if the autopilot never logged, e.g. it failed to start).
        diag_dir = os.path.join(cartographer_dir, "OUTPUT", "diag")
        timelines = [f for f in glob.glob(os.path.join(diag_dir, "*_timeline.jsonl"))
                     if os.path.getmtime(f) >= launch_t]
        if timelines:
            latest_log = max(timelines, key=os.path.getmtime)
            print(f"[+] Compiling report for: {os.path.basename(latest_log)}")
            subprocess.run([python_exe, "flight_replay.py", latest_log, "--open"], cwd=cartographer_dir)
        else:
            print("[!] No timeline was produced this run (autopilot never logged?) -> no report compiled.")

if __name__ == "__main__":
    main()
