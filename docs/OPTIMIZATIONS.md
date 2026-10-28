# Possible optimizations (after v3)

Ideas to make the drones and the mesh protocol better. None of them is needed for the open-day demo.
Each item says what it fixes, roughly how big it is, and where it goes. Start with
[`V3_CHANGES.md`](V3_CHANGES.md) for how v3 works now.

Size: **S** = an afternoon, **M** = a day or two, **L** = a week or more.

## Worth doing first

These fix things we have actually seen in the self-test.

| # | Idea | Fixes | Size | Where |
|---|---|---|---|---|
| 1 | **Keep checking while drones fly.** The leader skips its "will a chain break in the next 3 s?" check while it is still moving drones one after another. Keep checking, and re-plan in the middle of a sequence. Only re-plan when the break is predicted against the in-flight positions *and* the targets, so it doesn't re-plan every 0.5 s while a drone is between spots. | A fast rover (2.5× demo speed) can outrun a 4-drone reshuffle: base A lost the rover for ~5 s in tests | S–M | `leader.py` `Coordinator.update` / `_move` |
| 2 | **Make-before-break hand-over.** When a chain changes, first fly a spare drone into the new spot, and release the old relay only after the new link is up (the leader sees it in STATUS neighbour lists). | Brief relay gaps during re-plans | M | `leader.py` `_move` |
| 3 | **Plan further ahead.** Use flight time: plan for where the rover will be when the drones *arrive* (rover speed × flight time), not a fixed 1.5–3 s ahead. | Drones arriving just after the rover has left their coverage | S | `leader.py` `_deploy`, `LOOK_AHEAD` |

## Drones

| # | Idea | What it gives | Size |
|---|---|---|---|
| 4 | **Real flight in Gazebo.** A multicopter plugin and controller on the drone model, driven by velocity commands, instead of stepped `set_pose` teleports. | Real physics and inertia; drones look like drones | L |
| 5 | **Smoother paths.** Ease in and out (accelerate, cruise, slow down) instead of constant speed. Fly a short curve instead of a straight line plus "stay 2 m above ground". | Nicer to watch | S |
| 6 | **Avoid other drones.** Keep a minimum gap between drones in flight (today two drones can pass through each other). | Realism | M |
| 7 | **Battery and rotation.** Drones report battery in STATUS; the leader swaps a low drone out with a parked one, then sends it home. | A good story for visitors, longer missions | M |
| 8 | **Height choice per spot.** The planner tries fixed heights (3 / 6 / 10 / 15 m). Picking the lowest height that still has line of sight would shorten climbs. | Shorter flights | S |

## Connection protocol

| # | Idea | What it gives | Size |
|---|---|---|---|
| 9 | **Resend timer that learns.** Measure the time between sending a bundle and getting its `CUSTODY_ACK` per link, and set the resend wait from that (like TCP's RTO), instead of a fixed 0.3 s doubling to 2 s. | Fewer useless resends on good links, faster recovery on bad ones | S |
| 10 | **Link quality from real losses.** The route table uses hop count, then a quality worked out from distance. Also count how many resends each link needed and prefer links that lose less. | Routes avoid flaky links | M |
| 11 | **Batched custody acks.** One `CUSTODY_ACK` naming several bundles, sent at most every 50 ms per neighbour. | About half the packets during the photo transfer | S |
| 12 | **Two paths for commands.** When two independent routes exist, send drive commands down both. The duplicate filter already drops the second copy. | One broken link no longer costs an ACK | M |
| 13 | **Photo: ask for missing chunks.** If base B still lacks chunks a few seconds after the last one arrived, it asks the rover to resend just those (like CFDP's NAK). | Fixes a photo stuck at 102/103 when a full relay store dropped a chunk | S–M |
| 14 | **Order mesh ON/OFF commands.** Give `MESH_CMD` a sequence number. The leader ignores older ones and echoes the newest in its heartbeat. Base A stops repeating only when the echo matches. | Two fast `mesh_ctl on/off` can't end in the wrong mode | S |
| 15 | **Survive a node restart.** Start sequence numbers from the clock (or add a boot id), so the duplicate filters of other nodes don't drop a restarted node's new bundles. | Restarting one node mid-demo works | S |
| 16 | **Keep expiring while killed.** A killed node should still expire its store, so revive doesn't produce a burst of expired pings. | Cleaner event log | S |
| 17 | **Persistent store.** Write held bundles to disk, so a restarted node still delivers them (real DTN does this). | Realism; little value in the simulation | M |
| 18 | **Security.** Sign bundles (like BPSec) so a fake `LEADER` can't take over the mesh. | Needed only outside the simulation | L |

## Dashboard

| # | Idea | What it gives | Size |
|---|---|---|---|
| 19 | **Serve the photo once.** Today every snapshot carries the image (~140 KB, 4 times a second). Serve it on its own URL instead. | Lighter page | S |
| 20 | **Show flights.** Draw each drone's path to its target while it flies. | Easier to follow re-plans on the projector | S |
| 21 | **Replay.** Record `mesh/*` to a file and play a run back in the dashboard. | Demo without running the whole simulation; debugging | M |
