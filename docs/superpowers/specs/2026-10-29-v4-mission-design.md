# Mesh v4: one base, science target, orbiter relay to Earth

Spec for Claude Code · 2026-10-29 · builds on `main` (v3 merged, commit `9f03aa9` or later)

Lives in the repo as `docs/superpowers/specs/2026-10-29-v4-mission-design.md` (the `start_v4.sh` script puts it
there, creates the branch and starts Claude Code).

**Deadline: open day, Sat 31 Oct 2026, 9:00-17:00.** Timeline (stop wherever time runs out; every step
leaves a working demo):

| When | Target |
|---|---|
| Thu 29 Oct | Steps 0, 1, 2 |
| Fri 30 Oct, by 18:00 | Steps 3, 4 (all MUST done) |
| Fri 30 Oct, evening | Steps 5, 6 if MUST is solid |
| **Fri 30 Oct, 23:00** | **Code freeze**: after this only bug fixes, no new features |
| Sat 31 Oct, before 9:00 | One full Gazebo dry run of the demo |

---

## Working rules (read first, follow every step)

1. **Branch.** Work only on a new branch `v4-mission`, created from the latest `origin/main`:
   `git fetch origin && git checkout -b v4-mission origin/main`.
   `main` is the base (origin) of this work, but **never** commit to, merge into or push to `main`.
   Never merge `v4-mission` anywhere. If `main` gets new commits while you work, tell the user and ask
   before bringing them in (`git merge origin/main` into `v4-mission`, never the other way round).
2. **Ask before you commit.** At the end of each step, show the user: what changed (files + one line each),
   the test results, then **ask permission** before `git commit`. Ask again before any `git push`.
   One commit per step (more only if the user asks).
3. **One step at a time, in order.** Do not start the next step until the user says so.
   If a step fails, stop and report; don't jump ahead.
4. **Time matters.** Smallest change that meets the step's checks. No refactors, no extra features,
   no renames beyond what the step lists. Anything not listed goes into "Later" (bottom of this file).
5. **Keep it working.** After every step: `python3 -m pytest src/*/test -q` passes, and the step's
   self-test checks pass (`ros2 run mesh_protocol selftest`, fake mode, ~8 min). Fake mode
   (`fake:=true`) must keep working for everything.
6. **Explain like to a beginner.** After each step, add a section to `docs/v4/EXPLAINER.md`:
   - the real-life problem it copies (2 lines),
   - files changed, one line each,
   - how it works (3-5 lines, no jargon without a one-line definition),
   - what to say to a visitor (1-2 sentences),
   - 2 likely visitor questions with answers.
   Also add any error you hit (message + fix) to `ERRORS.md`.
7. **Unchanged rules from v3:** shared names, constants, keys and packet format live only in
   `mesh_common`; no new Python dependencies; nodes never crash on a bad packet; the simulated radio path
   `mesh/<u>/tx -> radio_channel -> mesh/<v>/air -> injector -> mesh/<v>/rx` stays.
   Only touch the custody / store / election code where a step says so.

## What we want to show (the demo in one paragraph)

A small scout rover leaves the **base** (a lander at the crater edge) and drives **by itself** to a
**science target** past the far rim of the crater. The rim cuts its radio, so the drones form a relay
chain (0 → 1 → 2 → 3 drones as it drives, 1 spare). At the target it takes a photo. Odometry and the photo
travel through the mesh to the base. The base holds them until the **orbiter** passes overhead, sends
what fits in the pass (priority first), and the data reaches **Earth** 8 s later (1 s = 1 min).
Scripted failures hit along the way and nothing is lost. A **scoreboard** compares mesh OFF vs ON.

```
EARTH <== 8 s delay == ORBITER <== pass window (~15 s / 60 s), 10 bundles/s == BASE (x = -40)
                                                                                 |  goals / stop
                                                                       drones (1 chain, 3 + 1 spare)
                                                                                 |  odometry, photo
                                                                               ROVER --> TARGET (x = +30)
```

## Priorities

| Tier | Steps | Rule |
|---|---|---|
| MUST | 0, 1, 2, 3, 4 | Needed for the open-day story |
| SHOULD | 5, 6 | Do if MUST is done and tested |
| LATER | bottom of file | Do **not** start without the user asking |

---

## Step 0: Create the branch and check the starting point (MUST, no code changes)

`main` already holds v3, verified on Ubuntu 24.04 + Jazzy + Gazebo (see `docs/V3_CHANGES.md`, steps 6-7).
This step only confirms the laptop still matches that before we change anything.

```bash
cd ~/mars_ws && git fetch origin && git checkout v4-mission   # start_v4.sh already created it from origin/main
rm -rf build install log && colcon build --symlink-install && source install/setup.bash
python3 -m pytest src/*/test -q                 # expect 86 passed
ros2 run mesh_protocol selftest                 # expect 39/39
```

**Done when:** both pass. If something fails, stop, report it and fix only that (ask before committing).
No Gazebo run needed here; the next Gazebo check is at the end of Step 1.

## Step 1: One base + science target, 20 m radio (MUST)

**Why:** two bases make no sense in the story, and with one base and a 32 m radio, one drone covers the
whole crater (measured with `planner.min_chain`). With a 20 m radio and the target at x = +30 the rover
needs 0 → 1 → 2 → 3 drones as it drives, so all 4 drones matter (3 in the chain + 1 spare).

**Changes:**
- `mesh_common`: units become `rover`, `drone_1..4`, `base`. Remove `base_b`; `COMMANDER = RECEIVER = "base"`.
  `GZ_NAMES["base"] = "base_station"`. `START_XY["base"] = (-40, 0)`. Add `TARGET_XY = (30.0, 0.0)`.
  `RADIO_RANGE = 20.0`. Keys: `mesh/base/cmd`, `mesh/base/acks`, `mesh/base/telemetry`, `mesh/base/photo`,
  `mesh/base/photo_cmd` (rename the `KEY_*_A` / `KEY_*_B` constants to plain names).
  Remove `GOAL_X` (use `TARGET_XY`).
- `mars_sim/scripts/build_relay_world.py` + `worlds/`: remove `base_station_2`; add a static **target marker**
  (a reddish rock or a flag, ~1 m) at `TARGET_XY`. Regenerate the world.
- `leader.py` + `mesh_link_model/planner.py`: plan **one** chain rover → base (keep the "spare drone shortens the
  longest hop" rule). `connected()` checks only the base.
- `mesh_node.py`: the base does everything base A and base B did (commands, ACKs, odometry, photo).
- `mission.py`: drive to `TARGET_XY` (still velocity commands for now; Step 2 changes that).
- `mesh_dashboard`: one base panel; draw the target on the map.
- `tools.py`, launch file, tests, self-test: rename / remove base B. Self-test checks that mentioned
  "both bases" now mean "the base".

**Checks:**
- New unit test: with the real terrain, `min_chain` needs 0 drones at x = -24, 1 at x = -12, 2 at x = 0, 3 at x = +30.
- Self-test passes (adjusted). With mesh ON the chain uses 3 drones at the target and 1 drone is spare.
- Gazebo: one base, the target marker visible, rover drives to the target with mesh ON.

## Step 2: The rover drives itself (MUST)

**Why:** with minutes of delay to Earth, real rovers can't be steered live; they get a goal and drive
themselves. This also gives the strongest DTN moment: with mesh OFF the rover still reaches the target,
and its data waits in its own store.

**Changes:**
- `mesh_common`: new routed bundle type `GOAL` (priority 0, lifetime 60 s), payload
  `{"goto": [x, y]}` or `{"stop": true}`. Add it to `TYPES` and `ROUTED`. `COMMAND` (velocity, 1 s) stays
  for manual driving (`send_cmd`).
- `mesh_node.py` (rover): on `GOAL goto`, drive itself to the point using its own position (`self.pos`, like
  GPS) with the steering now in `mission.py` (look-ahead point, same speed). The 2 s watchdog applies only
  to manual `COMMAND` driving. On `GOAL stop`: stop at once. At the target: stop, **take the photo
  automatically** and send it to the base (bulk priority). ACK the `GOAL` like a command.
  Move the steering into a small pure function (`mesh_protocol/autodrive.py`) with a unit test.
- `mission.py`: sends one `GOAL goto TARGET_XY` through the base, repeats it until the ACK arrives,
  then only reports progress. Keep the "stuck assist" for Gazebo bumps.

**Checks:**
- Self-test, mesh OFF: the rover reaches the target anyway; its held odometry grows; nothing reaches the base.
- Then mesh ON: drones deploy, all held odometry reaches the base (late), the photo arrives with sha256 OK.
- A `GOAL stop` sent while driving stops the rover.

## Step 3: Orbiter + Earth (MUST)

**Why:** real rovers send most data home through orbiters that are overhead only for short passes, and a
message takes ~4-24 min to reach Earth. Scale: 1 s demo = 1 min Mars.

**Changes:**
- `mesh_common`: `SPACE_UNITS = ["orbiter", "earth"]`, `ALL_UNITS = UNITS + SPACE_UNITS` (use `ALL_UNITS`
  where every unit needs a key/publisher: radio channel, injector, launch, stats). Constants:
  `ORBIT_PERIOD = 60`, `ORBIT_HEIGHT`, `ORBIT_SPEED`, `ELEVATION_MASK = 20` (degrees), `LIGHT_DELAY = 8.0`.
  Keys: `mesh/space/state` (orbiter position, pass open, next pass in s), `mesh/earth/telemetry`,
  `mesh/earth/photo`.
- `mesh_link_model/space_link.py` (new, pure logic + tests): `orbiter_pos(t)` = a point moving in a straight
  line high over the world along x, wrapping every `ORBIT_PERIOD`; `elevation(base, orbiter)`;
  `pass_open(t, terrain)` = elevation ≥ mask **and** terrain line of sight; `next_pass_in(t)`.
  Tune `ORBIT_HEIGHT` / `ORBIT_SPEED` so a pass lasts about 15 s per 60 s; print the real numbers.
- `link_model.py`: add two links to the table: `base <-> orbiter` (up only during a pass) and
  `orbiter <-> earth` (always up, field `"delay": LIGHT_DELAY`). No other unit links to the orbiter
  (the scout rover's radio can't reach orbit; only the base has the big antenna). Publish `mesh/space/state`.
- `radio_channel.py`: a link with `delay` delivers its copies that many seconds later (timer queue, one thread).
- `mesh_node.py`, **space links only**, the drone mesh must behave exactly as before:
  1. *Contact plan:* the base treats `orbiter` as reachable while `mesh/space/state` says the pass is open;
     orbiter and earth treat each other as always reachable. No HELLO needed on space links.
  2. *Long resend timer:* custody resend wait on the orbiter <-> earth hop = `2 * LIGHT_DELAY + 1` s.
  3. *Long ACK life:* a `CUSTODY_ACK` on the orbiter <-> earth hop gets lifetime `LIGHT_DELAY + 2`.
  - Fixed routes: base → earth via orbiter; orbiter → earth direct.
  - The base, after delivering `TELEMETRY` or `PHOTO_CHUNK` from the rover, originates a copy to `earth`
    (payload keeps the rover's original `sent_time`).
  - `earth` publishes what arrives on `mesh/earth/*` with the data's age. `orbiter` and `earth` do not
    pass on control floods (`STATUS`, `LEADER`, `MESH_CMD`).
- `store.py`: allow a per-next-hop resend wait (for fix 2).
- Launch file: start `orbiter` and `earth` nodes; argument `space:=true` (default true).

**Checks:**
- Unit tests: pass windows open/close as planned; no resend to earth before `2 * LIGHT_DELAY + 1` s;
  a held bundle at the base is sent when the pass opens.
- Self-test stage: odometry reaches Earth only after a pass opens, about `LIGHT_DELAY` s after leaving the
  orbiter; no duplicate deliveries at Earth. All earlier self-test checks still pass.

## Step 4: Data limit per pass (MUST)

**Why:** rovers collect more data than a pass can carry, so priority decides what goes home first.

**Changes:**
- `mesh_common`: `PASS_RATE = 10` bundles/s.
- `mesh_node.py` (base): bundles whose next hop is `orbiter` use their own send budget of `PASS_RATE`
  (the existing priority order picks which go first).
- Publish in base stats: `held_for_earth`, `sent_to_orbiter`.

**Checks:** with a photo taken just before a pass, odometry goes first and the photo only partly fits;
the rest goes in the next pass; Earth gets the full photo with sha256 OK in the end.

## Step 5: Scenario runner + failures (SHOULD)

**Why:** one scripted run makes the demo the same every time, and each event copies a real failure.

**Changes:**
- `mesh_protocol/scenario.py` + `config/scenario.yaml`: a timed list of events, run with
  `ros2 run mesh_protocol scenario` (prints each event as it fires and publishes it on `mesh/events`).
- Events (reuse `mesh/failure/control` where it exists):
  - `burst`: loss/corruption 50 % for N s, then back (add timed mode to `passthrough_injector.py`).
  - `kill` / `revive` a unit: drone, leader (`leader`), or `base` (= base safe mode). Already exists; check it
    works for the base.
  - `radiation`: one bundle in a given unit's store gets a flipped bit. That unit re-checks the CRC before
    sending, drops the damaged copy, counts it (`radiation_dropped`) and logs an event. (Custody was already
    handed over, so that one bundle is lost: that is the point to explain.)
- Default scenario (adjust times after the first run): burst at 15 s, kill a relay drone at 40 s,
  base safe mode 65-80 s, radiation flip at 50 s.

**Checks:** scenario runs start to end in fake mode; every event shows in the dashboard event log;
with mesh ON no odometry is lost except the one radiation bundle.

## Step 6: Dashboard: Earth panel + scoreboard (SHOULD)

**Changes:**
- Earth panel: items received, age of newest data, orbiter "overhead / below horizon", countdown to the next
  pass, base `held_for_earth`, Earth photo progress.
- Scoreboard: `scenario.py` writes a result file per run (`runs/mesh_off.json`, `runs/mesh_on.json`):
  odometry delivered %, photo OK yes/no, bundles lost. The dashboard shows both side by side.
  Numbers come only from real counters.

**Checks:** a mesh OFF run then a mesh ON run fill both columns; values match the node counters.

## After the steps (only when the user asks)

- Update `README.md`, `docs/RELAY_DEMO.md` (4-terminal demo for v4) and write `docs/V4_CHANGES.md`
  (v3 → v4 table, keys, decisions, known limits), in the style of `V3_CHANGES.md`.
- Final full self-test + Gazebo run. Still **no merge to main**: the team decides that.

---

## LATER (do not start unless the user asks)

| Idea | Why later |
|---|---|
| Dust storm (radio range ramps 24 → 20 m, more loss on long links, leader re-plans) | Most visual, but touches range in link model + planner live; a drone kill already shows re-routing |
| Orbiter model gliding across the Gazebo sky | Purely visual |
| Earth sends the GOAL through the orbiter during a pass | Nice autonomy story, more routing work |
| Drone battery + swap with the spare | Good story, too big for now |
| `docs/OPTIMIZATIONS.md` items (learning resend timer, make-before-break, photo NAK, ...) | After open day |
| Solar conjunction, error-correcting codes, bandwidth per direction | Little extra to show |
