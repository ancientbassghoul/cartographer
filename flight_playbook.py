"""flight_playbook.py — loader + step-player for flight_playbook.json (the autopilot's control vocabulary).

The playbook holds PLATFORM control dynamics (maneuver recipes + drive magnitudes), kept as DATA so the
autopilot's perception (self-calibrating detection, in code) stays cleanly separate from its action
vocabulary. See flight_playbook.json's _comment + CLAUDE.md "CRITICAL AUTONOMY STANDARD": these are
how-the-airframe-responds constants (legitimate), never this room's answer (detected live).

  * presets : continuous drive vectors for state-machine states (ascend, forward, hold).
  * recipes : ordered, finite-duration step sequences (arm, reset_attitude, turn_unit, back_off).
  * rules   : operating rules (e.g. reset_attitude_before_forward — press 'c' before any forward push
              so wall contact reads as a clean expansion-collapse, per the user's hard rule).

RecipePlayer steps a recipe out over wall-clock time, yielding the active field dict each tick and
reporting completion — the autopilot overlays that onto the full control vector it publishes at 20 Hz.
"""

import argparse
import json
import os

REPO = os.path.dirname(os.path.abspath(__file__))
DEFAULT_PATH = os.path.join(REPO, "flight_playbook.json")


class FlightPlaybook:
    """Parsed flight_playbook.json with typed accessors. Fail-fast on a missing recipe/preset."""

    def __init__(self, data):
        self.presets = data.get("presets", {})
        self.recipes = data.get("recipes", {})
        self.rules = data.get("rules", {})

    @classmethod
    def load(cls, path=None):
        path = path or DEFAULT_PATH
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        return cls(data)

    def preset(self, name) -> dict:
        if name not in self.presets:
            raise KeyError(f"playbook preset '{name}' not found (have {list(self.presets)})")
        return dict(self.presets[name])

    def recipe(self, name) -> list:
        if name not in self.recipes:
            raise KeyError(f"playbook recipe '{name}' not found (have {list(self.recipes)})")
        return [dict(s) for s in self.recipes[name]]

    def rule(self, name, default=None):
        return self.rules.get(name, default)

    def player(self, name) -> "RecipePlayer":
        return RecipePlayer(self.recipe(name), name=name)


class RecipePlayer:
    """Plays a finite recipe (list of {<fields>, duration_s}) out over time. `fields(now)` returns
    (active_field_dict, done): the fields for the current step until its duration elapses, then advances;
    once past the last step it returns ({}, True). `now` is a monotonic seconds clock."""

    def __init__(self, steps, name=""):
        self.name = name
        self.steps = steps
        self.i = 0
        self._t0 = None

    def reset(self):
        self.i = 0
        self._t0 = None

    @property
    def done(self) -> bool:
        return self.i >= len(self.steps)

    def fields(self, now: float):
        if self.done:
            return {}, True
        if self._t0 is None:
            self._t0 = now
        step = self.steps[self.i]
        if (now - self._t0) >= float(step.get("duration_s", 0.0)):
            self.i += 1
            self._t0 = now
            return self.fields(now)
        return ({k: v for k, v in step.items() if k != "duration_s"}, False)


def main():
    ap = argparse.ArgumentParser(description="Inspect the flight playbook")
    ap.add_argument("--path", default=None)
    ap.add_argument("--print", dest="do_print", action="store_true", help="print presets/recipes/rules")
    args = ap.parse_args()
    pb = FlightPlaybook.load(args.path)
    if args.do_print or True:
        print(f"[playbook] {args.path or DEFAULT_PATH}")
        print("  presets:")
        for k, v in pb.presets.items():
            print(f"    {k}: {v}")
        print("  recipes:")
        for k, steps in pb.recipes.items():
            dur = sum(float(s.get("duration_s", 0.0)) for s in steps)
            print(f"    {k}: {len(steps)} step(s), ~{dur:.2f}s -> {steps}")
        print(f"  rules: {pb.rules}")


if __name__ == "__main__":
    main()
