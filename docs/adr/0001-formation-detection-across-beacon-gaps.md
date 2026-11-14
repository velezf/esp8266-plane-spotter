# ADR 0001: Formation detection across beacon gaps

Status: **Parts 1 and 2 accepted and implemented. Part 3 is a proposal: not accepted, no code written.**
Date: 2026-11-14
Decider: Frank Velez
Base commit: the groundwork commit on `feat/formation-diagnostics`
(`feat(firmware): flag repeated OpenSky state vectors; ...`). Every claim about
the code below was read at that commit.

## Context

The formation feature exists for one sighting in particular: a military flight
loitering nearby whose members take turns beaconing. At any moment one ship has
a live ADS-B position and the others are dark. The firmware never tagged that
flight as a formation.

Two facts about the data explain why.

- **OpenSky repeats the last known state vector.** After its receivers lose an
  aircraft it keeps serving the same vector, identical every poll, for minutes.
- **`detectFormations()` pairs contacts within a single poll.** A formation
  latches only after `FORMATION_MIN_POLLS` consecutive polls in which the
  members are clustered. Pairing a repeated position against a partner's live
  one reads as kilometres of false spacing. Two repeated vectors agree with
  their own previous poll and would satisfy the latch on one real sample.

The same repeats caused a second fault. `trackRotorcraft()` treated a frozen
position as holding station, so a helicopter that had flown away latched as
loitering.

## Decision

### 1. Flag repeated vectors and keep them out of loiter and pairing (accepted)

A blip is `stale` when the response `time` minus the vector's `time_position`
exceeds `STALE_POS_S`. The authority for the value is the `#ifndef` default in
`plane_spotter.ino`. An unknown age reads as fresh.

- `trackRotorcraft()` treats a stale poll as no evidence of holding station and
  pushes the anchor clock forward by the gap.
- `detectFormations()` pairs only freshly sampled candidates
  (`formationSampled()`). A stale member is out of formation for that poll.
- Under `LOG_BLIP_DUMP`, `[blip]` lines carry a `STALE` suffix and a
  `[form] pair` line prints separation and velocity difference for every near
  pair, joined or not.

This is groundwork and diagnostics. **It does not detect the flight described in
Context**, because that flight never has two fresh members on two consecutive
polls.

### 2. No "hold" on the formation latch (accepted; the hold was built and removed)

An earlier version of the branch let a latched member keep its count for up to
three polls while it, or a possible partner, was stale. It was removed.

- It could only key on "some candidate somewhere is stale", because a member
  does not know who its partner is. An unrelated stale contact therefore kept a
  genuinely split formation tagged for up to 90 s.
- It only helped after a latch. A flight that alternates beacons never latches
  in the first place, and its gaps exceed 90 s anyway.
- It added a counter to two tables and a parameter through three functions.

Consequence accepted with the removal: a pair that loses a member to a coverage
gap loses its tag and sounds the formation voice again when it re-forms. If
that proves noisy, suppress the repeat voice; do not bring the hold back.

### 3. Pair across time instead of within one poll (proposed)

Stop requiring both ships to be live at the same moment.

- Remember each rotorcraft or military contact's last fresh position, velocity
  and time. `helis[]` already holds an anchor; `milSeen[]` holds no position.
- Each poll, consider every candidate that was fresh within a window of a few
  minutes. Move the older one forward by its age before measuring separation.
- For a loitering flight, skip the projection: two distinct airframes whose
  last fresh positions fall inside one loiter circle within the window are a
  group.

A stale ship then takes part with an estimated position instead of being
dropped, and nothing depends on unrelated contacts.

Open questions that decide the design, none of them answered by data yet:

- When a ship goes dark, does its vector freeze, vanish from the response, or
  come back with a jump?
- How long is each ship dark, and do the live periods overlap at all?
- Straight-line projection is wrong for an orbiting ship after a few tens of
  seconds. Is the loiter-circle rule enough on its own for the case that
  matters?
- False groupings: two unrelated helicopters using the same pad minutes apart,
  or aircraft in trail on the same approach.

## Alternatives considered

- **Leave pairing as it was on `main`.** Rejected: false spacing from repeats
  prevents latching, and two frozen contacts can latch falsely.
- **The hold (see Decision 2).** Built, tested, removed.
- **Build Decision 3 now against a synthetic alternating pair.** Deferred. The
  design would rest on one remembered sighting and the open questions above.

## Evidence

- **Host harness** over the shipped bodies of `trackRotorcraft()` and
  `detectFormations()`: 18 checks pass. Removing the stale filter from
  `formationSampled()` makes it fail, so the checks can fail. The harness lives
  outside the repo, as `CLAUDE.md` "Testing logic on the host" describes.
- **Board capture, 2026-11-14, about three hours** (with gaps where the host
  Mac slept): 227 polls, 789 contact lines, 106 distinct aircraft.
  - 151 lines (19%) were flagged stale, from 12 aircraft.
  - In the first 2.5 hours, 107 of 109 stale lines were identical to that
    aircraft's previous poll, and 3 unflagged lines were identical to the
    previous poll. The flag and the repeats agree almost exactly.
  - Every stale aircraft was airport surface traffic: parked, taxiing, or at
    runway elevation at roll speed. Receivers lose an aircraft on the ground
    and the last vector is repeated, typically for about 10 polls.
  - No false positives on airborne traffic.
  - Two helicopters appeared briefly; neither went stale. No military contact
    appeared. No `[form]` line was logged. No resets or fetch errors.
- **Not exercised on the board:** the loiter fix and the fresh-only pairing.
  The evidence for both is the host harness only.

## Follow-ups

Tracked in [`docs/RESUME.md`](../RESUME.md).
