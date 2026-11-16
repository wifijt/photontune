#!/usr/bin/env python3
#
# Copyright (C) 2026  wifijt
#
# This program is free software: you can redistribute it and/or modify it under
# the terms of the GNU General Public License as published by the Free Software
# Foundation, either version 3 of the License, or (at your option) any later
# version.  This program is distributed WITHOUT ANY WARRANTY; see the GNU
# General Public License at <https://www.gnu.org/licenses/> for details.
#
"""The reference decision, and the failure message, with no camera attached.

    python3 reftest.py

sabotage_test.py needs hardware because it tests what photontune does TO a
camera. This tests what photontune CONCLUDES from what a camera gave it, which
is pure arithmetic over Sample objects and needs nothing but the module.

Every fixture is a real measurement, copied out of
/var/log/photontune/photontune.log. The run it is named for is the one that
reported "no gain ... saw the tags ... add light" about a camera that had just
read 1.00 tags at four consecutive gains.
"""
import os
import sys

# The module sits beside this file. Spelled out because -I (and running this
# from somewhere else) drops the script's own directory from sys.path.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import photontune as pt

FAILED = []


def check(name, got, want):
    ok = got == want
    print("  %-56s %s" % (name, "ok" if ok else "FAIL got=%r want=%r"
                                                  % (got, want)))
    if not ok:
        FAILED.append(name)


def sample(gain, tags, ambiguity, frames=33, multitag=0, exposure=863.0):
    """A Sample as trial() would have returned it."""
    s = pt.Sample(exposure, gain)
    s.frames = frames
    s.multitag_solves = multitag
    s.tag_counts = [tags] * frames
    s.ambiguities = [ambiguity] * frames if ambiguity is not None else []
    return s


class FakeState(object):
    """Only what _why_nothing_worked reads."""
    def __init__(self, trials, t_budget=863.0):
        self.trials = trials
        self.t_budget = t_budget


CFG = pt.Config()                       # min_tags 2.0, max_ambiguity 0.20
GAINS = [0, 20, 40, 60, 80, 100]

#: Camera "Left", 2026-11-16 18:02:26 - the run that produced the bad message.
LEFT_A = [sample(100, 0.89, 0.937), sample(0, 0.40, 0.537),
          sample(20, 1.00, 0.800), sample(40, 1.00, 0.429),
          sample(60, 1.00, 0.429), sample(80, 1.00, 0.460)]
#: The same camera, same tags, 64 seconds later.
LEFT_B = [sample(100, 1.00, 0.173)]
#: Camera "Right" in both runs: one tag, dead on.
RIGHT = [sample(100, 1.00, 0.000)]
#: Camera "Left", 17:54 - the bench with nothing in front of it.
EMPTY = [sample(g, 0.00, None) for g in (100, 0, 20, 40, 60, 80, 100)]


def test_the_top_of_the_grid_is_not_the_scene():
    """The reference gate, on the two samples that disagreed about one camera."""
    print("\nthe reference is one sample, and samples vary")
    _eff, one, why = pt._as_reference(LEFT_A[0], CFG)
    check("gain-100 sample is rejected as a reference", why is None, False)
    check("...and is not read as a one-tag scene", one, False)

    _eff, one, why = pt._as_reference(LEFT_B[0], CFG)
    check("the SAME camera 64 s later IS usable", why is None, True)
    check("...and is read as a one-tag scene", one, True)

    eff, one, why = pt._as_reference(RIGHT[0], CFG)
    check("Right is a clean one-tag scene", (one, why is None), (True, True))
    check("...so the floor drops from min_tags to 1", (CFG.min_tags, eff),
          (2.0, 1.0))


def test_a_better_sample_exists_in_the_walk():
    """What the second look is for: a usable sample below the top of the grid.

    Not every camera is rescued by it - Left's ambiguity never came under the
    0.20 limit at ANY gain, so Left's run should still fail. That is the
    correct answer and the test says so: the second look must not manufacture
    a pass out of a scene that has not got one.
    """
    print("\nthe second look, and what it must NOT rescue")
    usable = [s for s in LEFT_A if pt._as_reference(s, CFG)[2] is None]
    check("no gain in Left's run A is a usable reference", usable, [])

    # Same shape of run, but with the ambiguity a workable tag gives.
    better = [sample(100, 0.89, 0.937), sample(40, 1.00, 0.120)]
    usable = [s for s in better if pt._as_reference(s, CFG)[2] is None]
    check("a mid-grid sample with good ambiguity IS usable", len(usable), 1)
    check("...and it is the mid-grid one, not the top", usable[0].gain, 40)


def test_the_message_tells_the_three_cases_apart():
    """The sentence that sent a student looking for lamps."""
    print("\nwhy nothing worked - three causes, three answers")

    empty = pt._why_nothing_worked(FakeState(EMPTY), CFG, GAINS)
    check("nothing seen: says NOTHING was detected",
          "NOTHING was detected" in empty, True)
    check("nothing seen: still sends them to the light", "add light" in empty, True)

    left = pt._why_nothing_worked(FakeState(LEFT_A), CFG, GAINS)
    check("tags seen: does NOT claim nothing saw a tag",
          "NOTHING was detected" in left, False)
    check("tags seen: does NOT send them for lamps", "add light" in left, False)
    check("tags seen: says tags WERE seen", "tags WERE seen" in left, True)
    check("tags seen: quotes the best count", "up to 1.00" in left, True)
    check("tags seen: names ambiguity as the blocker",
          "ambiguity never came under" in left, True)
    check("tags seen: quotes the best ambiguity", "best 0.429" in left, True)
    check("tags seen: says what to do", "closer to face-on" in left, True)
    # note_problem's own reference_unusable entry is appended to this
    # sentence by the summary. Said here as well it printed twice in one line.
    check("tags seen: does not duplicate the reference note",
          "did not itself pass" in left, False)

    # Tags visible and unambiguous, but fewer than --min-tags asks for, and
    # the one-tag rescue did not fire (a two-tag floor on a one-tag scene).
    short = pt._why_nothing_worked(FakeState([sample(40, 1.00, 0.05)]),
                                   CFG, GAINS)
    check("short of the floor: blames the floor, not the light",
          "--min-tags" in short and "add light" not in short, True)
    check("short of the floor: no reference note here either",
          "did not itself pass" in short, False)


def test_the_message_never_contradicts_its_own_log():
    """The invariant, stated once: if a trial saw a tag, do not say none did."""
    print("\nthe invariant")
    for name, trials in (("Left run A", LEFT_A), ("Right", RIGHT),
                         ("one good sample", [sample(40, 1.00, 0.05)])):
        msg = pt._why_nothing_worked(FakeState(trials), CFG, GAINS)
        saw = max(t.mean_tags for t in trials)
        check("%s (saw %.2f): no 'NOTHING was detected'" % (name, saw),
              "NOTHING was detected" in msg, False)
    msg = pt._why_nothing_worked(FakeState(EMPTY), CFG, GAINS)
    check("empty scene (saw 0.00): DOES say nothing was detected",
          "NOTHING was detected" in msg, True)


if __name__ == "__main__":
    for fn in (test_the_top_of_the_grid_is_not_the_scene,
               test_a_better_sample_exists_in_the_walk,
               test_the_message_tells_the_three_cases_apart,
               test_the_message_never_contradicts_its_own_log):
        fn()
    print("")
    if FAILED:
        print("FAILED %d: %s" % (len(FAILED), ", ".join(FAILED)))
        sys.exit(1)
    print("all reference/message tests pass")
