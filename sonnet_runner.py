#!/usr/bin/env python
"""Chunked implementation runner: feed one plan chunk at a time to `claude -p --model sonnet`.

Splits a spec on its `## CHUNK <n>` headings. Everything BEFORE the first chunk heading is the
GLOBAL CONTEXT (execution guidelines + contracts) and is prepended to every chunk prompt; each
chunk then runs in its own cold `claude` session, in order.

Why each piece is here (all of it earned by a real failure mode):
  * UTF-8 reads       -- Path.read_text() uses the LOCALE codec. On a cp1252 box a UTF-8 spec is
                         decoded WITHOUT raising and silently mojibakes every em-dash/arrow, i.e.
                         exactly the anchors a chunk must match verbatim. Never drop the encoding.
  * verification gate -- `claude -p` exits 0 when the model refuses, stalls, or writes code whose
                         tests fail. "the process exited" is NOT "the chunk landed", so after each
                         chunk this runner executes the module self-tests ITSELF and halts on failure.
  * empty-chunk gate  -- and that self-test gate passes TRIVIALLY on an untouched tree, so it is
                         BLIND to the one failure it most needs to see: a chunk that wrote nothing
                         at all. The usual cause is a session killed mid-flight by a usage limit,
                         which can still exit 0. Left unchecked the runner marches into the NEXT
                         chunk, whose declared dependency was never built. An empty diff between
                         chunk trees is therefore a FAILURE, not a note.
  * git tree snapshots-- a per-chunk .patch so one bad chunk can be isolated and reverted without
                         losing the others. Uses `git write-tree` (writes an object; no commit, no
                         ref move) -- the runner never commits.
  * --start-at        -- chunk edits are INSERTIONS and are NOT idempotent; re-running from 1 would
                         double-apply them. Resume instead.
  * transcripts       -- --verbose streams to a console you will lose; tee it to disk.
"""

import argparse
import re
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

REPO = Path(__file__).resolve().parent

DEFAULT_PLAN = "test_plan.md"
# Every module in the repo that has a self-test. A chunk can only be gated by a suite that actually
# RUNS after it, so this list is the gate's reach: anything missing here is a module a chunk may
# silently break.
DEFAULT_SUITES = ["autopilot.py", "frontier_planner.py", "visual_recovery.py", "flight_replay.py",
                  "ground_grid.py", "map_store.py", "salvage_flight.py", "perception_worker.py",
                  "visualizer.py", "perception_timing_report.py"]

# The suites run under the PROJECT VENV, not under whatever interpreter launched this runner.
# `perception_worker.py` imports torch/MASt3R, which exist only in the venv -- running it with a bare
# system python raises ModuleNotFoundError, and the gate would report a FAILURE that is really a
# missing dependency (or, worse, get excluded from the list to make the noise go away, silently
# narrowing the gate). The venv can run every suite; the system python cannot.
VENV_PY = REPO / "venv" / "Scripts" / "python.exe"

# Grep/Glob are REQUIRED: the spec gives anchors as source strings rather than line numbers, and
# autopilot.py is ~8.7k lines (a bare Read truncates).
ALLOWED_TOOLS = "Read,Edit,Write,Grep,Glob,Bash"

# A structural guardrail rather than a prose one: the spec tells the agent not to commit, but a
# denied tool beats an instruction. Bash stays broad so the self-tests and the revert-proof
# exercise still work; only repo-mutating git verbs are blocked.
DISALLOWED_TOOLS = (
    "Bash(git commit:*),Bash(git push:*),Bash(git add:*),"
    "Bash(git reset:*),Bash(git checkout:*),Bash(git stash:*),Bash(git rebase:*)"
)

CHUNK_RE = re.compile(r"(?m)(?=^## CHUNK \d+)")   # line-anchored: never split mid-line or in a fence


def _utf8_stdout():
    """stdout is cp1252 on this box; printing a spec full of em-dashes would mangle or raise."""
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")
        except (AttributeError, ValueError):
            pass


def git(*args, check=True):
    return subprocess.run(["git", *args], cwd=REPO, check=check,
                          capture_output=True, text=True, encoding="utf-8", errors="replace")


def index_is_clean():
    """True when nothing is staged. Snapshotting stages everything and then resets, which would
    discard a pre-existing staged state -- so refuse to snapshot unless the index starts clean."""
    return git("diff", "--cached", "--quiet", check=False).returncode == 0


def snapshot_tree():
    """A tree object for the CURRENT working tree, including untracked files. No commit, no ref
    move. Returns the tree SHA and leaves the index as it was found (clean)."""
    git("add", "-A", "--", ".")
    tree = git("write-tree").stdout.strip()
    git("reset", "-q")
    return tree


def split_plan(plan_path):
    text = Path(plan_path).read_text(encoding="utf-8")   # NEVER drop the encoding -- see module docstring
    parts = CHUNK_RE.split(text)
    return parts[0].strip(), [p.strip() for p in parts[1:] if p.strip()]


def chunk_title(chunk_text):
    return chunk_text.splitlines()[0].strip() if chunk_text else "(untitled)"


def suite_python():
    """The interpreter the self-tests run under -- see VENV_PY. Falls back to this process's own
    interpreter ONLY when the venv is absent, and says so LOUDLY: a gate that has quietly become
    narrower than it looks is worse than no gate, because it still prints PASS."""
    if VENV_PY.exists():
        return str(VENV_PY)
    print(f"*** WARNING: {VENV_PY} not found -- running the self-test gate with {sys.executable} "
          f"instead. Suites with GPU/torch dependencies (perception_worker.py) will report an "
          f"IMPORT failure that is a missing dependency, NOT a code defect. ***")
    return sys.executable


def run_suites(suites, log_fh, timeout):
    """Run each module's --self-test. Returns [] on success, else a list of failure descriptions.

    Two independent signals, because they have drifted apart before: a non-zero exit code, and the
    word FAIL anywhere in the output. A passing suite emits neither.
    """
    failures = []
    py = suite_python()
    for suite in suites:
        if not (REPO / suite).exists():
            failures.append(f"{suite}: MISSING")
            continue
        try:
            r = subprocess.run([py, suite, "--self-test"], cwd=REPO,
                               capture_output=True, text=True, encoding="utf-8",
                               errors="replace", timeout=timeout)
        except subprocess.TimeoutExpired:
            failures.append(f"{suite}: TIMEOUT after {timeout}s")
            continue
        out = (r.stdout or "") + (r.stderr or "")
        log_fh.write(f"\n===== self-test: {suite} (rc={r.returncode}) =====\n{out}\n")
        bad = [ln for ln in out.splitlines() if re.search(r"\bFAIL\b", ln)]
        if r.returncode != 0 or bad:
            failures.append(f"{suite}: " + (bad[0].strip()[:120] if bad else f"exit {r.returncode}"))
    return failures


def build_prompt(global_context, chunk_text, idx, total):
    prior = ("None -- this is the first chunk." if idx == 1
             else f"Chunks 1..{idx - 1} are ALREADY APPLIED in the working tree.")
    return (
        f"GLOBAL CONTEXT (applies to every chunk):\n{global_context}\n\n"
        f"WORKING-TREE STATE:\n{prior}\n"
        f"Do NOT re-implement, re-verify or refactor a previously applied chunk. If this chunk\n"
        f"lists a dependency on an earlier chunk, that dependency is already satisfied on disk.\n\n"
        f"CURRENT TASK -- chunk {idx} of {total}:\n{chunk_text}\n\n"
        f"INSTRUCTIONS:\n"
        f"1. Execute ONLY this chunk, strictly to the signatures and anchors given above.\n"
        f"2. Do NOT start, preview or plan any subsequent chunk.\n"
        f"3. Do NOT commit, stage, stash or otherwise mutate git state.\n"
        f"4. Finish by running the self-tests this chunk names and reporting the full PASS/FAIL list.\n"
    )


def run_chunk(prompt, model, timeout, log_path):
    """Stream `claude -p` to the console AND to a transcript. Returns the exit code."""
    cmd = ["claude", "-p", prompt, "--verbose", "--model", model,
           "--allowedTools", ALLOWED_TOOLS, "--disallowedTools", DISALLOWED_TOOLS]
    with open(log_path, "w", encoding="utf-8", errors="replace") as fh:
        fh.write(f"$ claude -p <{len(prompt)} chars> --model {model}\n\n")
        try:
            proc = subprocess.Popen(cmd, cwd=REPO, stdout=subprocess.PIPE,
                                    stderr=subprocess.STDOUT, text=True,
                                    encoding="utf-8", errors="replace", bufsize=1)
        except FileNotFoundError:
            msg = "ERROR: `claude` was not found on PATH."
            print(msg)
            fh.write(msg + "\n")
            return 127
        deadline = time.monotonic() + timeout
        try:
            for line in proc.stdout:
                sys.stdout.write(line)
                fh.write(line)
                if time.monotonic() > deadline:
                    proc.kill()
                    msg = f"\nERROR: chunk exceeded --timeout ({timeout}s); killed."
                    print(msg)
                    fh.write(msg + "\n")
                    return 124
            return proc.wait()
        except KeyboardInterrupt:
            proc.kill()
            raise


def main():
    _utf8_stdout()
    ap = argparse.ArgumentParser(
        description="Run a chunked plan through `claude -p`, one chunk per cold session.")
    ap.add_argument("--plan", default=DEFAULT_PLAN, help=f"plan/spec markdown (default: {DEFAULT_PLAN})")
    ap.add_argument("--start-at", type=int, default=1, metavar="N",
                    help="resume at chunk N (1-based). Chunk edits are insertions and NOT idempotent "
                         "-- after a failure, fix the tree and resume; never re-run from 1.")
    ap.add_argument("--stop-after", type=int, default=None, metavar="N", help="stop after chunk N")
    ap.add_argument("--model", default="sonnet")
    ap.add_argument("--timeout", type=int, default=1800, help="per-chunk seconds (default 1800)")
    ap.add_argument("--suites", default=",".join(DEFAULT_SUITES),
                    help="comma-separated self-test modules run after EVERY chunk (empty to disable)")
    ap.add_argument("--no-verify", action="store_true", help="skip the self-test gate (not recommended)")
    ap.add_argument("--no-git", action="store_true", help="skip the per-chunk .patch snapshots")
    ap.add_argument("--allow-empty-chunk", default="", metavar="N[,N...]",
                    help="comma-separated 1-based chunk indices permitted to produce an EMPTY diff "
                         "(default: none -- a chunk that changes no files halts the run)")
    ap.add_argument("--list", action="store_true", help="list the chunks and exit")
    ap.add_argument("--dry-run", action="store_true",
                    help="write every prompt to disk without invoking claude")
    args = ap.parse_args()

    plan_path = Path(args.plan)
    if not plan_path.is_absolute():
        plan_path = REPO / plan_path
    if not plan_path.exists():
        sys.exit(f"ERROR: plan not found: {plan_path}")

    global_context, chunks = split_plan(plan_path)
    total = len(chunks)
    if not total:
        sys.exit(f"ERROR: no '## CHUNK <n>' headings found in {plan_path}")

    if args.list:
        print(f"{plan_path}\nglobal context: {len(global_context)} chars\n{total} chunks:")
        for i, c in enumerate(chunks, 1):
            print(f"  {i:>2}. [{len(c):>6} chars] {chunk_title(c)}")
        return

    suites = [s.strip() for s in args.suites.split(",") if s.strip()]
    verify = not args.no_verify and bool(suites)

    # Fail-fast on a malformed index list rather than silently ignoring it -- an --allow-empty-chunk
    # the user THINKS is armed but isn't would disarm the guard exactly when it is being relied on.
    try:
        allow_empty = {int(n) for n in args.allow_empty_chunk.split(",") if n.strip()}
    except ValueError:
        sys.exit(f"ERROR: --allow-empty-chunk expects comma-separated integers, got "
                 f"{args.allow_empty_chunk!r}")

    use_git = not args.no_git and not args.dry_run
    if use_git and not index_is_clean():
        print("WARNING: the git index has staged changes, so per-chunk patches are DISABLED "
              "(snapshotting would discard that staged state). Commit or unstage first, "
              "or pass --no-git to silence this.")
        use_git = False
    if not use_git and not args.dry_run:
        # A disabled safety check must be LOUD, never silent: the empty-chunk gate below lives
        # inside the snapshot block, so no snapshots means no gate.
        print("WARNING: per-chunk git snapshots are OFF, so the EMPTY-CHUNK GUARD CANNOT RUN. "
              "A chunk that exits 0 having written nothing will pass unnoticed and the next chunk "
              "will build on a dependency that was never built. Watch each chunk's change summary "
              "yourself, or re-enable snapshots (clean index, drop --no-git).")

    run_dir = REPO / "OUTPUT" / "sonnet_runs" / datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir.mkdir(parents=True, exist_ok=True)

    print(f"plan          : {plan_path}")
    print(f"global context: {len(global_context)} chars")
    print(f"chunks        : {total} (running {args.start_at}..{args.stop_after or total})")
    print(f"model         : {args.model}")
    print(f"verify after  : {', '.join(suites) if verify else 'DISABLED'}")
    print(f"git patches   : {'on' if use_git else 'off'}")
    print(f"artifacts     : {run_dir}")

    prev_tree = snapshot_tree() if use_git else None
    (run_dir / "global_context.md").write_text(global_context, encoding="utf-8")

    for idx, chunk_text in enumerate(chunks, start=1):
        if idx < args.start_at:
            continue
        if args.stop_after is not None and idx > args.stop_after:
            break

        print(f"\n{'=' * 78}\n Chunk {idx}/{total}: {chunk_title(chunk_text)}\n{'=' * 78}")
        prompt = build_prompt(global_context, chunk_text, idx, total)
        (run_dir / f"chunk_{idx:02d}_prompt.md").write_text(prompt, encoding="utf-8")

        if args.dry_run:
            print(f"[dry-run] prompt written ({len(prompt)} chars); claude NOT invoked.")
            continue

        log_path = run_dir / f"chunk_{idx:02d}.log"
        resume_hint = (f"  resume after fixing:  python sonnet_runner.py "
                       f"--plan {args.plan} --start-at {idx}")

        rc = run_chunk(prompt, args.model, args.timeout, log_path)
        if rc != 0:
            print(f"\nChunk {idx} FAILED: claude exited {rc}. Halting.")
            print(f"  transcript: {log_path}")
            print(resume_hint)
            sys.exit(1)

        if use_git:
            cur_tree = snapshot_tree()
            patch = run_dir / f"chunk_{idx:02d}.patch"
            # write_bytes, NOT write_text: on Windows write_text maps \n -> os.linesep (CRLF), which
            # corrupts the patch for every LF-terminated file in the repo -- `git apply` then rejects
            # it with "patch does not apply". Found the hard way on chunk 8 (frontier_planner.py is LF;
            # config.yaml is CRLF, so chunk 1's patch had applied by luck). subprocess text=True already
            # normalises git's output to \n, so these bytes are LF and must stay that way.
            patch.write_bytes(git("diff", prev_tree, cur_tree).stdout.encode("utf-8"))
            stat = git("diff", "--stat", prev_tree, cur_tree).stdout.strip()
            print(f"\n[chunk {idx}] changes:\n{stat if stat else '  (no file changes)'}")
            print(f"[chunk {idx}] patch: {patch}")
            print(f"[chunk {idx}] revert just this chunk:  git apply -R {patch}")

            # EMPTY-CHUNK GATE. `claude` exited 0 but touched nothing, so the chunk did not land.
            # The self-test gate below cannot see this -- an untouched tree passes trivially -- and
            # continuing would run the NEXT chunk against a dependency that was never built.
            if not stat and idx not in allow_empty:
                print(f"\nChunk {idx} FAILED: claude exited 0 but changed NO FILES, so this chunk "
                      f"did NOT land.")
                print("  Most likely cause: the session ran out of tokens (usage limit) and was cut "
                      "off before it wrote anything. A refusal or a silent stall looks identical.")
                print(f"  Nothing to revert -- the diff is empty, so the tree is exactly as it was "
                      f"before chunk {idx} started.")
                print(f"  Fix: top up / wait for the limit to reset, then RE-RUN THIS CHUNK. Do not "
                      f"re-run from chunk 1 -- chunk edits are insertions and are not idempotent.")
                print(f"  transcript: {log_path}")
                print("  If the transcript shows the chunk DID do work, check `git status` before "
                      "resuming -- something reverted it.")
                print(f"  If this chunk legitimately changes no files, re-run with "
                      f"--allow-empty-chunk {idx}")
                print(resume_hint)
                sys.exit(1)

            prev_tree = cur_tree

        if verify:
            print(f"[chunk {idx}] verifying: {', '.join(suites)}")
            with open(log_path, "a", encoding="utf-8", errors="replace") as fh:
                failures = run_suites(suites, fh, args.timeout)
            if failures:
                print(f"\nChunk {idx} FAILED VERIFICATION -- claude exited 0 but the suites did not pass:")
                for f in failures:
                    print(f"  - {f}")
                print(f"  transcript: {log_path}")
                if use_git:
                    print(f"  revert this chunk:  git apply -R {run_dir / f'chunk_{idx:02d}.patch'}")
                print(resume_hint)
                sys.exit(1)
            print(f"[chunk {idx}] self-tests: ALL PASS")

        print(f"Chunk {idx} complete.")

    print(f"\nPipeline complete. Artifacts: {run_dir}")


if __name__ == "__main__":
    main()
