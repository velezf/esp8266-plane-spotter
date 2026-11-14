# Resume pointer

Live status, open branches and backlog. Design reasoning lives in
[`docs/adr/`](adr/); this file only says where things stand.

Last updated: 2026-11-14

## Status

- `main`: WEAPONS page fails closed when there is no target or the feed is
  stale. Merged and pushed to `fork`.
- The board was last flashed from `feat/formation-diagnostics` (before the
  hold-free tree was committed; same source). It includes the WEAPONS change.

## Open branches

| Branch | State | Next step |
|---|---|---|
| `feat/formation-diagnostics` | PARKED. One firmware commit (stale flag, loiter fix, fresh-only pairing, logging) plus the ADR and this file. Not merged, not pushed. | Owner decides whether to merge the groundwork to `main` now or with the detection work. |
| `feat/weapons-no-target` | CLOSED. Merged to `main`. | Delete the branch and its worktree under `.claude/worktrees/`. |

## Backlog

1. **Detect a formation whose members take turns beaconing.** The reason the
   formation feature exists, and not done. Proposal and open questions:
   [ADR 0001](adr/0001-formation-detection-across-beacon-gaps.md), Decision 3.
2. **Capture a real alternating-beacon flight** with `LOG_BLIP_DUMP` on. This
   is the data ADR 0001 Decision 3 is waiting for. Keep the host awake
   (`caffeinate`) for the whole capture.
3. **Log the position age in seconds** on `[blip]` lines, not only the `STALE`
   flag, so a capture shows how long each ship was dark. Do this before item 2.
4. **Exercise the loiter fix and fresh-only pairing on the board.** Host-tested
   only so far; no rotorcraft or military contact went stale during the
   2026-11-14 capture.
5. **Runway ghosts on the radar.** A landed aircraft's last vector is repeated
   for minutes at roll speed and is still treated as live by the radar and the
   target pages. Expected effect: its blip creeps forward from the same spot
   after every poll. Not confirmed on the panel. The `stale` flag could freeze
   or drop such a blip and keep it out of target selection.
6. **`STALE_POS_S` is untuned for the case it was written for.** It is safe for
   ordinary traffic (ADR 0001 Evidence); it has never met a stale rotorcraft or
   military contact.
