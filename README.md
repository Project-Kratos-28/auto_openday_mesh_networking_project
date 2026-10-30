<!-- mars-mesh-readme -->
# Mars Rover Relay Mesh: Simulation

![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-blue) ![Gazebo Harmonic](https://img.shields.io/badge/Gazebo-Harmonic-orange) ![Ubuntu 24.04](https://img.shields.io/badge/Ubuntu-24.04-E95420) ![Python](https://img.shields.io/badge/Python-3.12-3776AB)

A rover drives to a **science target** on the far side of a Mars crater. The crater rim blocks its small radio, so it loses contact with the **base station**. **Four relay drones** fly out and line up behind the rover like stepping stones, and our own **delay-tolerant mesh protocol** (on [Zenoh](https://zenoh.io)) hops the data from drone to drone back to the base. The rover sends its odometry, a live camera feed and photos. A visitor can even send a picture from their phone through the same mesh. Everything is shown on a mission-control web dashboard.

> **Final demo version = git tag `demo-final`.** On demo day everyone runs exactly this:
> ```bash
> cd ~/mars_ws && git fetch --tags --force && git checkout demo-final
> ```
> Quick start: [section 4](#4-quick-start-demo-day). What to say and click: [section 5](#5-the-demo-script).

**Contents:** [1 What this is](#1-what-this-is) · [2 Key ideas](#2-key-ideas-in-plain-words) · [3 Current state](#3-current-state-of-the-project) · [4 Quick start](#4-quick-start-demo-day) · [5 Demo script](#5-the-demo-script) · [6 Setup](#6-setup-on-a-new-laptop-once) · [7 World](#7-the-world) · [8 Network](#8-how-the-network-works) · [9 Walkthroughs](#9-what-happens-when-end-to-end-walkthroughs) · [10 Dashboard](#10-the-dashboard) · [11 Code map](#11-code-map-which-file-does-what) · [12 Design decisions](#12-design-decisions-and-why) · [13 Commands](#13-commands-and-options) · [14 Developing](#14-developing-and-changing-things) · [15 Repo layout](#15-repository-layout) · [16 Git workflow](#16-team-workflow-git) · [17 Known limits](#17-known-limits) · [18 Visitor Q&A](#18-questions-visitors-ask-and-short-answers) · [19 Troubleshooting](#19-troubleshooting) · [20 Roadmap](#20-roadmap) · [21 More docs](#21-more-docs)

---


## 1. What this is

```
   base ─────────────── (crater rim blocks the radio) ─────────────── 🚩 science target
 commands, photos        ✈ d1 ───── ✈ d2 ───── ✈ d3 ───── ✈ d4
      ▲                      relay chain of drones
      └────────────────────────── rover ● ────────────────────────────┘
                              (drives from the base to the flag)
```

1. The **rover** starts next to the **base** (rover x = -32 m, base x = -40 m) and drives through the crater toward the **science target** (x = +30 m). It stops 2 m before the flag.
2. The base sends **velocity commands** to the rover, and the rover answers each with an **ACK**. The rover sends its **odometry**, its **live camera feed** and **photos** back to the base.
3. A radio link only works with a **clear line of sight over the terrain** and within **20 m**. Inside the crater the rim blocks the base, so the rover **stops by itself** after 2 s without a command (safety watchdog).
4. Switch the **mesh ON** (dashboard toggle). One drone is the elected **leader**. It flies the others to relay spots so there is one chain from the rover to the base, and moves them ahead of the rover as it drives. The link comes back and the rover drives on. Odometry held during the outage arrives **late instead of being lost**.
5. A **failure injector** can drop or corrupt packets, and you can kill drones. Broken packets are caught by a CRC check, lost ones are resent, and a new leader takes over in about 2 s.

The protocol works like NASA's **Delay/Disruption Tolerant Networking (DTN)**: hop-by-hop custody transfer, store and forward, per-packet lifetimes and priorities, and routes planned by the leader.

## 2. Key ideas in plain words

Read this first if you have to explain the project. Every word here is used later in the README.

| Word | What it means here |
|---|---|
| **Base station** | The "lander" or control point. It sends drive commands and receives everything the rover sends back. |
| **Rover** | The robot that drives to the science target. It has a radio, wheels, a safety watchdog and a camera. |
| **Drone / relay** | A flying unit that does one job: pass packets along. It carries no science instruments. |
| **Mesh network** | Every unit can pass on packets for the others, so data can hop base → drone → drone → rover and back. No unit needs to reach the base directly. |
| **Hop** | One radio jump between two neighbouring units. A route of 3 hops goes through 2 drones in the middle. |
| **Radio range / line of sight** | Two units can talk only if they are within **20 m** *and* the straight line between their antennas stays above the ground. Hills block radio, so being close is not enough. |
| **Link** | A working radio connection between two units. The link model recomputes all links 5 times a second. |
| **Packet / bundle** | One message, as JSON with a CRC32 checksum. In DTN a packet is called a *bundle*; we use both words for the same thing. |
| **CRC32** | A checksum added to every packet. If noise flips a bit, the check fails and the packet is thrown away instead of being believed. |
| **DTN (Delay/Disruption-Tolerant Networking)** | NASA's idea for space networks, where links come and go and signals are slow. Instead of "send and hope", each node **stores** data and **forwards** it when a link exists. We built a simplified version. |
| **Custody transfer** | A node keeps its copy of a bundle until the *next hop* confirms (`CUSTODY_ACK`) that it now holds it. If no confirmation arrives it resends (after 0.3 s, then 0.6, 1.2, up to 2 s). Responsibility moves hop by hop, like passing a baton. |
| **Store and forward** | If there is no usable next hop right now, the bundle waits in the node's store (max 500) and goes when a link appears. This is why odometry recorded during an outage still arrives, late. |
| **Lifetime** | Every bundle has an expiry time. Old drive commands (1 s) are useless and dangerous, so they die. Odometry (120 s) is valuable, so it waits. |
| **Priority** | 0 = expedited (commands, control), 1 = normal (odometry), 2 = bulk (photo chunks, camera frames). When a node is busy, higher priority goes first. |
| **TTL** | Hop limit (8). Each forward lowers it, so a lost packet cannot circle forever. |
| **Flood vs routed** | *Flooded* packets (`STATUS`, `LEADER`, `MESH_CMD`) are copied by every unit once, without custody: they repeat every 0.5 to 1 s anyway. *Routed* packets follow a path chosen by the leader and use custody. |
| **Route table** | For every unit and destination, "send it to this neighbour next". Computed by the leader as the shortest path by hops. |
| **Leader election** | The drones agree on one **leader** drone to do the planning, with no fixed boss. The leader sends a heartbeat (`LEADER`) every 0.5 s that is valid for a **lease** of 2 s. If it goes quiet, the lowest-id drone that is still heard takes over with `term + 1`. The higher term always wins, so there is one leader again after a split. |
| **Watchdog** | The rover stops by itself if it gets no drive command for 2 s. This is the safe behaviour when the link is lost. |
| **Odometry** | The rover's own measure of where it is and how fast it goes, sent to the base twice a second. |
| **ACK** | The rover's "I got your command" reply. The base uses it to know the link works both ways. |
| **sha256** | A fingerprint of a whole file. The base compares it with the fingerprint the rover sent, so a photo that arrives in pieces is proven intact. |
| **Zenoh** | The messaging library our programs use to talk to each other (publish / subscribe on named keys such as `mesh/base/cmd`). It is only the plumbing between our programs: the simulated **radio** is our own code (link model + radio channel). |
| **Gazebo / SDF** | Gazebo Harmonic is the 3D physics simulator. SDF files describe the world and models. ROS 2 is used for packaging, building (`colcon`) and launch files. |
| **`set_pose`** | A Gazebo service that places a model at a position. Drones are moved with it (many small steps a second), so they glide rather than fly with real physics. |

## 3. Current state of the project

**Everything is merged into `main` and tagged `demo-final`.** The build and all unit tests pass (117 tests), and the demo has been run end to end on the team laptop.

| Part | What it does | Where (`src/`) | Status |
|---|---|---|---|
| World | 110 × 60 m crater, rover with camera, 4 drones, 1 base, red target flag | `mars_sim` | ✅ |
| Shared definitions | Unit names, layout, radio range, Zenoh keys, packet format + CRC32 | `mesh_common` | ✅ |
| Link model + radio channel | Line of sight over the real terrain + 20 m range decides which links are up; delivers a packet only to units with a working link | `mesh_link_model` | ✅ |
| Relay planner | Works out how many drones are needed and where they go | `mesh_link_model` | ✅ |
| Mesh protocol (DTN) | One node per unit: custody transfer, store and forward, lifetimes, priorities, routes, CRC check | `mesh_protocol` | ✅ |
| Leader drone | Election with a lease; the leader plans drone positions and routes and follows the rover | `mesh_protocol` | ✅ |
| Mission autopilot | Drives the rover to the target, asks for a photo on arrival, re-arms after a world reset | `mesh_protocol` | ✅ |
| Camera, photos, message from base | Live feed (2 fps), rover photo on arrival or button (rover → base), and a **message from the base to the rover**: a picture sent from a visitor's phone that the rover shows on its screen and that struggles to form when the link is down; sha256 check, packet log | `mesh_protocol`, `mesh_dashboard` | ✅ |
| Mission-control dashboard | Crater map with live routes, held bundles, delivery rates, late odometry, event log, camera, photo, QR, failure controls, passcode | `mesh_dashboard` | ✅ |
| Demo runner | One command starts everything, with a QR window and an optional public tunnel | `scripts/` | ✅ |
| Failure injection | **Stand-in only** (`passthrough_injector`: drop / corrupt). The full module is not built | `mesh_link_model` | 🟡 |
| Real drone flight physics | Drones glide with `set_pose`; no multicopter physics | `mesh_protocol/mover.py` | ❌ later |

**How it got here (what was merged):**

| Work | Status |
|---|---|
| v3: DTN mesh, leader election, photo transfer, web dashboard, smooth drone flight | ✅ in `main` |
| v4 Step 1 (one base, science target, 20 m radio) and Step A (kill/revive fix, code-review fixes) | ✅ in `main` |
| Passcode-locked dashboard, `mars_demo.sh` runner, QR window, Cloudflare tunnel, requirements file, mission re-arm (PR #1) | ✅ in `main` |
| Real rover camera, live feed, phone QR upload, packet log | ✅ in `main` (the phone picture is now a base → rover message, see section 8) |
| Open-day docs (`docs/OPEN_DAY.md`) and merge of the above (PR #2) | ✅ in `main` |
| Old PyQt6 GUI (`Ronak_GUI` branch) | ❌ **not merged and not used**: replaced by the web dashboard |
| Old v4 plan, steps 2 to 6 (self-driving rover, orbiter / Earth, data limit, scenario runner, Earth panel) | ❌ **dropped**: that plan is outdated |

Gazebo is reached with the **Gazebo Python bindings** (`gz.transport13`): positions come from `/world/mars_world/pose/info`, the rover is driven on `/rover/cmd_vel`, the camera is read from `/rover/camera`, and drones move with the `set_pose` service. No ROS 2 bridge is needed for the demo.

## 4. Quick start (demo day)

**You need** the setup from [section 6](#6-setup-on-a-new-laptop-once) done once on the laptop.

```bash
cd ~/mars_ws
git fetch --tags --force && git checkout demo-final        # the final version
colcon build --symlink-install && source install/setup.bash
./scripts/mars_demo.sh deps       # once: installs the extra Python packages (prints a sudo command if one is needed)
./scripts/mars_demo.sh stop       # clear any old run (ports 7447, 8080 and 8081 must be free)
./scripts/mars_demo.sh start --tunnel   # Gazebo + mesh + dashboard + mission + QR window + public links
```
(`start` without `--tunnel` works only for phones on the same Wi-Fi as the laptop, see [the venue network](#the-venue-network-who-uses-which-link) below.)

Then open **http://localhost:8080**. Stop with `Ctrl-C`, or `./scripts/mars_demo.sh stop` from another terminal. Check what is running with `./scripts/mars_demo.sh status`.

What `start` does, in order: builds if needed, checks the packages, starts the dashboard, starts Gazebo + the 6 mesh nodes and helpers (**mesh OFF at the start**), prints the dashboard links (LAN and, with `--tunnel`, public), opens the QR window, then starts the mission so the QR window is up before the rover moves.

| Option for `start` | Meaning |
|---|---|
| `--tunnel` | open **two** public Cloudflare https links: one for the dashboard (viewers) and one for the phone upload page (the QR on the dashboard then points at it, unless you set `PHONE_URL` yourself). Needs internet on the laptop |
| `--mesh on` | start with the mesh already ON (default: off) |
| `--passcode TEXT` | controls passcode (default: `$MESH_PASSCODE` or the built-in one; `""` unlocks everything) |
| `--host ADDR` / `--port N` | dashboard address (default `0.0.0.0` = whole LAN) and port (default 8080) |
| `--fake` | no Gazebo, fake rover (for protocol or dashboard work) |
| `--no-mission` | start without driving the rover |
| `--build` | force a `colcon build` first |
| `--no-qr` | do not open the QR window |
| `--detach` | leave it running in the background |

**Passcode:** watching the dashboard is public. The mesh toggle, the failure controls and TAKE PHOTO need the passcode, which a masked box asks for once per browser tab. Default: `kratos123`. Change it with `MESH_PASSCODE=... ./scripts/mars_demo.sh start`. It travels in clear text over plain http, so it keeps visitors out but is not real security.

### The venue network: who uses which link

Campus Wi-Fi often stops devices from talking to each other, so a link to the laptop's LAN address may not open on a visitor's phone. That is why the demo uses public Cloudflare links.

| Who | Uses | Link |
|---|---|---|
| **Demonstrator** (the laptop owner) | the dashboard on the laptop, with the controls | `http://localhost:8080` and the passcode |
| **Viewers who just watch** | the QR window the script opens, or the public dashboard link it prints | `https://….trycloudflare.com`: view only, the controls need the passcode |
| **Visitors who send a picture to the rover** | the QR in the dashboard's "Message from base → rover" panel | a second public link `https://….trycloudflare.com/phone` |

- `./scripts/mars_demo.sh start --tunnel` creates both public links by itself and prints them. The phone link is created **first**, because the dashboard reads it at start-up.
- Put the laptop on a **phone hotspot or other internet connection** you trust. Cloudflare tunnels need outbound internet, and a restrictive campus network may block them. Try it on the venue network before the audience arrives.
- **Both public addresses change on every run.** Start the demo before people arrive, and show them the fresh QR.
- `cloudflared` is stored in `/tmp/mars_demo/cloudflared/`, which a reboot wipes. After a reboot, run `./scripts/mars_demo.sh deps --tunnel` once while you have internet.
- If a tunnel cannot start, the script says so and carries on: the dashboard QR window then shows a LAN link, and the phone QR falls back to the LAN address (it only works for phones on the same Wi-Fi as the laptop, for example the same hotspot).
- `./scripts/mars_demo.sh stop` closes both tunnels.
- Anyone with the public links can open them. The dashboard controls are protected by the passcode, the upload page is not (see [known limits](#17-known-limits)).

**On a VM** (virtual GPU, Gazebo's default `ogre2` renderer fails): use `./scripts/phone_tunnel.sh`. It starts Gazebo with the classic `ogre` renderer plus a Cloudflare tunnel for the phone page, then run the mission yourself with `ros2 run mesh_protocol mission`. Details: [`docs/OPEN_DAY.md`](docs/OPEN_DAY.md).

## 5. The demo script

Do the steps in this order. The failure demo goes **last**.

1. **Show the problem (mesh OFF).** The rover drives from the base toward the flag. The base is CONNECTED at first. Once the hills (or the 20 m range) cut the line, the dashboard shows NO CONNECTION and the rover stops after 2 s. Say: "its radio is blocked by the crater, so it waits."
2. **Switch the mesh ON.** Click the **MESH** toggle and enter the passcode. The leader drone flies the other drones out. The chain forms, the base is CONNECTED again, and the rover drives on. Held odometry arrives late (amber on the odometry graph), then live (green). The rover needs 0 drones near the base, 1 at about x = -12, 2 at about x = 0 and 3 at the target; the spare drone shortens the longest hop.
3. **Arrival and photo.** The rover stops 2 m before the flag and sends a photo of what its camera sees. Show the **live camera** panel (2 fps) and press **TAKE PHOTO** for another. It arrives in 1 KB chunks and is checked with sha256. The event log and packet log show the chunks and the route they took.
4. **Message from the base.** A visitor scans the QR code and sends a selfie as "mission control". The base cuts it into chunks and sends them to the **rover** over the mesh; the **Rover screen** panel shows it forming. **Best moment to do it:** send it while the mesh is **OFF** and the rover is out of reach. The chunks pile up at the base (see the held bundles), the screen stays static, and the picture only starts to form when you switch the mesh ON and the drones link up.
5. **Failure demo (last).** Drop 10 to 20 % of packets: resends go up, nothing is lost. Corrupt packets: the CRC catches them. Select a drone and press **KILL**: the routes change, and if it was the leader another drone takes over in about 2 s. **REVIVE** brings it back.
6. **Repeat.** Click "Restart simulation" in Gazebo. Everything starts again from scratch (see walkthrough G): the rover goes back to the start line and the mission re-arms, the drones fly back to their parking spots, the mesh goes **OFF**, and the dashboard clears all its history and reloads itself. There is no need to restart the script.

Tips: in Gazebo scroll out until the base and the flag are both visible, or right-click `primary_rover` in the Entity Tree and choose **Follow**. For odometry over the **whole** trip, turn the mesh ON before the rover leaves (`--mesh on`).

## 6. Setup on a new laptop (once)

**You need:** Ubuntu 24.04 (native or dual boot is best; a VM works with `phone_tunnel.sh`) and about 15 GB free.

> 📶 **On campus Wi-Fi?** If a download fails with `SSL certificate problem: self-signed certificate`, use a phone hotspot for the setup.

Run each block in a terminal (`Ctrl + Alt + T`), in order.

**Step 1: Language settings** (ROS 2 needs UTF-8)
```bash
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

**Step 2: Add the ROS 2 package source**
```bash
sudo apt install -y software-properties-common curl
sudo add-apt-repository -y universe
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
```

**Step 3: ROS 2 Jazzy + build tools**
```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y ros-jazzy-desktop ros-dev-tools git
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**Step 4: Gazebo Harmonic + the ROS–Gazebo packages**
```bash
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt update && sudo apt install -y gz-harmonic ros-jazzy-ros-gz
```
> ⚠️ Use **Gazebo Harmonic** (`gz sim`) only. Do **not** install Gazebo Classic (`gazebo`, `gazebo11`, `gazebo_ros`).

**Step 5: System packages for the Python parts**
```bash
sudo apt install -y python3-gz-transport13 python3-gz-msgs10 python3-numpy python3-pip
```

**Step 6: Get access + log in to GitHub.** The repo is **private**. Ask the owner to add you (repo → Settings → Collaborators), accept the invite, then:
```bash
sudo apt install -y gh
git config --global user.name "Your Name"
git config --global user.email "you@example.com"        # the email on your GitHub account
gh auth login --hostname github.com --git-protocol https --web
gh auth setup-git
```

**Step 7: Clone, install the Python packages, build.** This repository **is** the ROS 2 workspace. Always clone it into `~/mars_ws`:
```bash
git clone https://github.com/Project-Kratos-28/auto_openday_mesh_networking_project.git ~/mars_ws
cd ~/mars_ws
python3 -m pip install --user --break-system-packages -r scripts/requirements.txt
colcon build --symlink-install
source install/setup.bash
echo "source ~/mars_ws/install/setup.bash" >> ~/.bashrc    # optional: every new terminal knows our packages
```
`scripts/requirements.txt` installs **`eclipse-zenoh` 1.10.x** (the old `zenoh` package on PyPI has a different API and does not work), **Pillow** and **segno** (camera, photos, QR) and **pytest**. `./scripts/mars_demo.sh deps` does the same and also fetches `cloudflared` for the tunnel.

**Step 8: Check everything works.** Open a **new** terminal:
```bash
echo $ROS_DISTRO                                              # → jazzy
gz sim --version                                              # → Gazebo Sim, version 8.x
python3 -c "import zenoh, numpy, PIL, segno, gz.transport13; print('python OK')"
for p in mesh_common mesh_protocol mesh_dashboard mesh_link_model; do python3 -m pytest -q src/$p/test; done   # 5 + 83 + 24 + 5 tests pass
ros2 run mesh_protocol selftest                               # optional, ~8 min, whole demo without Gazebo
```
The self-test prints `N/M checks passed` on its last line. Lines marked `[KNOWN ISSUE]` are expected and do not fail the run (see [section 17](#17-known-limits)).

## 7. The world

**World name:** `mars_world` · gravity **3.71 m/s²** (Mars) · metres · crater centre = origin `(0, 0)` · the route runs along the x axis · render engine `ogre` (so it also runs on VMs).

| Entity | Gazebo name | Mesh name | Position (x, y) | Notes |
|---|---|---|---|---|
| Terrain | `mars_crater` | none | none | 110 × 60 m. Crater 36 m wide: floor z = 0, rim top z = 6 at r = 18 m, plain z ≈ 1.5 |
| Base | `base_station` | `base` | (-40, 0) | Sends commands; receives odometry, camera feed and photos. 10 × 10 m pad, antenna ≈ 10.5 m above the pad |
| Rover | `primary_rover` | `rover` | (-32, 0) | 1 m, differential drive, faces +x. Antenna 1.3 m up. Camera 320 × 240, 2 Hz, topic `/rover/camera` |
| Target | red flag marker | none | (30, 0) | The rover stops 2 m before it (`TARGET_XY`, `TARGET_REACH`) |
| Drones | `drone_1` … `drone_4` | same | parked at (-48, ±3) and (-51, ±3), 3 m up | Hover where placed; moved with `set_pose` |

- **Radio:** a link is up when the distance is ≤ **20 m** **and** the straight line between the two antennas stays **0.5 m above the ground** everywhere.
- The base and the target (70 m apart) can never talk directly.

| Gazebo name | Type | Used for |
|---|---|---|
| `/rover/cmd_vel` | `gz.msgs.Twist` | drive the rover (sent by the rover's mesh node) |
| `/rover/odom` | `gz.msgs.Odometry` | rover wheel odometry (sent to the base) |
| `/rover/camera` | `gz.msgs.Image` | live feed and photos |
| `/world/mars_world/pose/info` | `gz.msgs.Pose_V` | true positions of all units (link model) |
| `/world/mars_world/set_pose` | service `gz.msgs.Pose` | move drones: each drone glides itself in small steps (20 a second) |

## 8. How the network works

### Packet path

A packet never goes straight from one unit to another:
```
 [unit node] ─ mesh/<u>/tx ─► radio channel ─ mesh/<v>/air ─► failure injector ─ mesh/<v>/rx ─► [unit v node]
                              (only to units with a                 (corrupt / drop)
                               working link: mesh/links)
 Gazebo poses ─► link model ─► mesh/links, mesh/positions       (5 times a second)
```

### Units and roles

| Mesh name | Role |
|---|---|
| `base` | **Commander and receiver.** Turns `mesh/base/cmd` into COMMAND packets, pings the rover every 1 s when idle, and publishes ACKs, odometry, the live feed and photos for the dashboard |
| `rover` | Applies COMMANDs (speed limited to 2 m/s and 3 rad/s, NaN = stop), answers each with an ACK, sends odometry every 0.5 s, the camera feed and photo chunks. **Watchdog:** stops if no drive command for 2 s |
| `drone_1` … `drone_4` | **Relays** (when the leader's route table names them). One of them is the **leader**: it plans where the drones go and computes the route table |

### Protocol rules (`mesh_protocol/mesh_node.py`, `store.py`, `routing.py`, `leader.py`)
- **HELLO** every 0.5 s (TTL 1, never forwarded). A neighbour is gone after 1.5 s without one.
- **Routed bundles** (`COMMAND`, `ACK`, `TELEMETRY`, `FEED`, `PHOTO_REQ`, `PHOTO_CHUNK`) go hop by hop. The next hop comes from the leader's route table (or the destination itself if it is in range). A node never hands a bundle to a next hop it cannot hear.
- **Custody transfer:** the sender keeps the bundle in its **store** until the next hop answers with `CUSTODY_ACK`, and resends after 0.3 s, doubling up to 2 s. The receiver always acks (even duplicates) and then holds the bundle itself.
- **Store and forward:** no usable next hop = the bundle waits in the store (max 500; when full the oldest bulk bundle goes first).
- **Lifetime:** every bundle expires. `COMMAND` 1 s (below the rover's 2 s watchdog, so it never acts on old orders), `FEED` 1 s (old camera frames are dropped, not queued), `TELEMETRY` 120 s (odometry arrives late instead of being lost), `PHOTO_CHUNK` 300 s.
- **Priority:** expedited (commands, control) → normal (odometry) → bulk (photo chunks, feed). Each node sends at most 40 bundles/s.
- **Control floods** (`STATUS`, `LEADER`, `MESH_CMD`): duplicate filter + TTL, passed on by every unit, no custody (they repeat every 0.5 to 1 s).
- **Duplicate filter** on `(src, type, seq)`, **TTL** 8 as a hop limit, **CRC32 + parse check:** broken packets are counted (`bad_crc`) and dropped. They never crash a node.

### Packet format (JSON, defined in `mesh_common`)
```json
{"src": "rover", "dst": "base", "next_hop": "drone_4", "seq": 42, "ttl": 7,
 "path": ["rover", "drone_3"], "type": "TELEMETRY", "sent_time": 1729071234.5,
 "lifetime": 120, "priority": 1,
 "payload": {"odom": {"x": 1.2, "y": 0.0, "yaw": 0.01, "v": 1.0}, "truth": {"pos": [1.2, 0.0, 0.1]},
             "moving": true, "battery": 99.5},
 "crc32": 3735928559}
```
`type` is one of: `HELLO`, `CUSTODY_ACK` (one hop), `COMMAND` (base → rover, e.g. `{"linear": 1.0, "angular": 0.0}` or `{"ping": true}`), `ACK` (rover → base), `TELEMETRY` (odometry), `FEED` (live camera JPEG), `PHOTO_REQ` / `PHOTO_CHUNK`, and the control floods `STATUS` (position + neighbours), `LEADER` (heartbeat: term, mode, route table, drone targets) and `MESH_CMD` (on / off).

### The leader drone (`mesh_protocol/leader.py`)
**Election:** the leader floods a `LEADER` heartbeat every 0.5 s (lease 2 s). A drone that hears none for 2 s + a random 0 to 0.5 s, and is the lowest-id drone it has heard from in the last 2 s, becomes leader with `term + 1`. A leader steps down on a higher term, or on the same term from a lower id. A new leader inherits the mode and drone targets from the last heartbeat. A killed or revived drone forgets its old role and mode first.

| Mode | Behaviour |
|---|---|
| `off` (default) | Drones parked behind the base, not used as relays. Direct links only. Switching OFF sends every drone home at once |
| `on` | Drones deploy **at once**, wherever the rover is, and follow it |

The dashboard (or `mesh_ctl`) asks the base, which sends `MESH_CMD` to the leader every second until a heartbeat shows the new mode. While deployed, the leader (using positions **reported** in `STATUS`, never true positions):
- **Checks** every 0.5 s whether the chain holds, for where the rover is now and where it will be in 5 s. If the rover goes quiet it guesses its position from its last speed.
- **Re-plans** only when the chain would break, because every move can open a short gap.
- **Plans** with `mesh_link_model/planner.py`: it tries 0, 1, 2… drones spaced along the line rover → base, at 3 / 6 / 10 / 15 m above the ground, uses the fewest that give every hop line of sight and range, and puts spare drones where they shorten the longest hop.
- **Assigns** new relay spots to the nearest drones, so a re-plan never sends drones across the crater to swap places.
- **Moves** each drone at up to 15 m/s with gentle speed-up and slow-down (`DRONE_SPEED`, `DRONE_ACCEL`), at least 2 m above the ground (`mesh_protocol/mover.py`, in a background thread). While nothing is connected all drones fly at once; while the chain works, one at a time.
- **Routes** by shortest hop count over the links it works out from positions, range and terrain line of sight. Drones with no `STATUS` for 2 s count as dead.

### Camera and photos
- **Live feed:** the rover turns the newest frame into a JPEG (q55) every 0.5 s and sends it as a `FEED` bundle (lifetime 1 s, bulk). The base publishes `mesh/base/feed`; the dashboard marks frames older than 2 s as frozen.
- **Photo (button or arrival):** the base sends `PHOTO_REQ`; the rover grabs a real frame (PNG) and sends it as `PHOTO_CHUNK`s (1 KB, base64, sha256 of the whole file). The base reassembles it, and chunks of an older photo are ignored.
- **Message from the base (the phone picture):** the QR opens `/phone` on a separate small server (port 8081; only `/phone` and `/upload`, images only, 15 MB max). The dashboard shrinks the picture (max 480 px, **baseline** JPEG q70, about 30 chunks) and publishes `mesh/base/uplink_photo`. The **base** sends it to the **rover** as `PHOTO_CHUNK` bundles (destination `rover`, bulk priority, custody transfer, its own photo ids). With no link the chunks wait in the base's store; the rover's `Reassembler` collects them, publishes progress on `mesh/rover/uplink` (including the gap-free start of the file, which a browser can draw because baseline JPEGs show their top part) and the dashboard's **Rover screen** shows the picture forming from the top down, then complete with a sha256 check.
- The QR on the dashboard encodes `PHONE_URL`, else the laptop's LAN address (only works for phones on the same Wi-Fi). `./scripts/mars_demo.sh start --tunnel` sets `PHONE_URL` itself to a public Cloudflare address for the upload page, so the QR works from any network. `./scripts/phone_tunnel.sh` (the VM version) does the same.

### Zenoh keys
| Key | From → To | Contents |
|---|---|---|
| `mesh/<unit>/tx` · `/air` · `/rx` | see packet path | raw packets |
| `mesh/links` | link model → all | `{"links": [{"a","b","up","distance","in_range","los","quality"}]}` |
| `mesh/positions` | link model → all | true `x, y, z` and `yaw` of every unit |
| `mesh/stats/<unit>` | each node → dashboard | counters + `held` (by priority), `custody_sent/acked`, `retransmits`, `expired`, `dropped_full`, `by_type`, `routes`, `leader`, `term`, `killed` |
| `mesh/events` | each node → dashboard | one event: retransmit, expired, election, route change, photo, killed / revived |
| `mesh/base/cmd` | mission, dashboard → base | velocity command `{"linear", "angular"}` |
| `mesh/base/acks` | base → dashboard | every ACK: round trip, command route, ACK route |
| `mesh/base/telemetry` | base → dashboard | odometry + route + delay + `late` |
| `mesh/base/photo_cmd` · `mesh/base/photo` | dashboard → base · base → dashboard | take a photo · progress, then the image + sha256 `ok` |
| `mesh/base/feed` | base → dashboard | newest live camera frame (JPEG) |
| `mesh/base/uplink_photo` | dashboard → base | a picture (from a visitor's phone) to send to the rover |
| `mesh/rover/uplink` | rover → dashboard | what the rover has received of that picture: progress, the gap-free start, then the whole image + sha256 `ok` |
| `mesh/coordinator/cmd` | dashboard, `mesh_ctl` → base | `on` / `off` |
| `mesh/coordinator/state` | base → dashboard | the newest `LEADER` heartbeat: leader, term, mode, state, chain, drone targets, routes |
| `mesh/mission/status` | mission → dashboard | rover x, progress %, assists |
| `mesh/failure/control` | dashboard, terminal → injector, nodes | e.g. `{"corrupt_probability": 0.2, "drop_probability": 0.05}`, `{"kill": "drone_2"}`, `{"revive": "drone_2"}` |
| `mesh/failure/events` | injector → dashboard | each corrupt / drop event |
| `mesh/reset` | link model → everyone | the simulation was restarted: `{"n": epoch}`. Every node, the injector and the dashboard start again from scratch |

All mesh programs connect only on **`127.0.0.1`** (one hub at `tcp/127.0.0.1:7447`, multicast off), so the mesh never leaks onto the venue Wi-Fi. Everything (Gazebo, mesh, dashboard) runs on **one laptop**. Only the dashboard (port 8080) and the phone page (port 8081) are served to other devices.

## 9. What happens when… (end-to-end walkthroughs)

Use these to explain the system step by step. File names tell you where to read the code.

### A. One drive command (base → rover, and the ACK back)
1. `mission.py` aims at a point 4 m ahead on the line y = 0 and, twice a second, publishes `{"linear", "angular"}` on `mesh/base/cmd`. (It steers using the simulator's true rover position, like a localisation system, so the demo does not drift. The rover's own odometry goes to the base over the mesh.)
2. The `base` node (`mesh_node.on_cmd_request`) wraps it in a `COMMAND` bundle: destination `rover`, lifetime 1 s, priority 0, a CRC32. It takes custody (puts it in its store) and picks the next hop from the leader's route table.
3. It publishes the bytes on `mesh/base/tx`. The **radio channel** copies them to `mesh/<v>/air` for **every unit that has an up link to the base** (like a real radio, all neighbours hear it). No link = the packet is simply lost.
4. The **failure injector** copies `air` to `rx`, optionally dropping or flipping bits (the sliders on the dashboard).
5. The receiving node decodes the bytes. A bad CRC or broken JSON is counted (`bad_crc`) and dropped. A good packet is answered at once with `CUSTODY_ACK`. If the packet is not for this unit, it is stored and forwarded the same way (next hop from the route table). If it is for this unit, `deliver` handles it.
6. The rover applies the command (`/rover/cmd_vel`, speed clamped to 2 m/s and 3 rad/s) and sends an `ACK` bundle back by the same mechanism. The watchdog timer restarts.
7. If the next hop does not confirm within 0.3 s (then 0.6, 1.2 … up to 2 s), the sender resends. If the bundle's 1 s lifetime runs out first, it expires: the base would rather send a fresh command than a stale one.

### B. The link is lost (mesh OFF), then the mesh is switched ON
1. The rover drives behind the crater rim. `link_model` sees the line of sight blocked, so `mesh/links` marks the base ↔ rover link down. Commands stop arriving. After 2 s the rover's **watchdog** stops it.
2. The rover keeps producing odometry. With no usable next hop, the `TELEMETRY` bundles sit in its **store** (lifetime 120 s). Old commands expire.
3. You click **MESH ON**. The browser sends `POST /api/mesh` with the passcode; `bridge.set_mesh` publishes `on` on `mesh/coordinator/cmd`.
4. The base turns that into a `MESH_CMD` flood, repeated every second until a `LEADER` heartbeat shows the new mode.
5. The **leader** (an elected drone) switches to mode `on`. Every 0.5 s it looks at the positions reported in `STATUS` bundles and asks the **planner** how many drones are needed between rover and base (0, 1, 2 or 3 in a chain, spare drones shorten hops). It **assigns** spots to the nearest drones and hands out targets in its heartbeat.
6. Each drone's `Mover` glides to its target (up to 15 m/s, speeding up and slowing down, at least 2 m above the ground). The leader also publishes a **route table** in the heartbeat, recomputed from the new positions.
7. When the chain is complete the links come up. The rover's stored `TELEMETRY` is released (a link coming back "kicks" retries), and the base receives it late: the dashboard draws these samples **amber**. Commands flow again and the rover drives on.
8. As the rover keeps going, the leader checks every 0.5 s whether the chain would break within the next 5 s, and moves drones ahead of the rover *before* it happens.

### C. A photo (button or arrival)
1. **TAKE PHOTO** (passcode needed) or the mission arriving publishes `mesh/base/photo_cmd`. The base sends a `PHOTO_REQ` bundle to the rover.
2. The rover takes the newest real frame from `/rover/camera` (PNG; the stock picture if no frame is available), cuts it into **1 KB chunks** (base64) and sends them as `PHOTO_CHUNK` bundles: bulk priority, custody transfer, lifetime 300 s, plus the file's **sha256**.
3. The base's `Reassembler` collects chunks (ignoring chunks of older photos), and when all are in it checks the sha256 and publishes the image on `mesh/base/photo`. The dashboard shows a progress bar, then the picture with "sha256 OK".
4. The **live feed** uses the same path but is deliberately throw-away: a JPEG every 0.5 s, lifetime 1 s, so under a bad link old frames are dropped instead of queued, and commands are never delayed by video.
5. The **message from the base** (phone picture) goes the other way, see walkthrough F.

### D. A drone is killed (including the leader)
1. The dashboard sends `{"kill": "drone_2"}` on `mesh/failure/control`. That node stops taking part in the mesh, and forgets any leader role, election timer, orders and mesh mode.
2. Its neighbours miss its `HELLO` for 1.5 s and drop it. The leader sees no `STATUS` for 2 s and counts it dead. It recomputes the plan and route table from the drones that are left, and bundles are re-routed. Bundles that were waiting for that next hop stay in the store and are retried on the new route.
3. If the dead drone was the **leader**, the other drones hear no heartbeat for the lease (2 s) plus a random 0 to 0.5 s. The lowest-id drone that is still heard becomes leader with `term + 1` and carries on from the last heartbeat (mode and targets).
4. **REVIVE** brings the drone back as a normal drone with a fresh role. It will only lead again if the rest stop hearing a leader.

### E. A corrupted or lost packet
1. With "corrupt 20 %", the injector flips random bits in the raw bytes of some packets on their way to a receiver. It never decodes packets, so the nodes see exactly what noise would do.
2. The receiver's CRC32 check fails, so it counts `bad_crc` and drops the packet **without** sending `CUSTODY_ACK`.
3. The sender, still holding custody, resends after its timer. The bundle arrives on a later try, or expires if its lifetime runs out first. Nothing crashes, and the delivery table on the dashboard shows sent, delivered and expired counts.

### F. A message from mission control to the rover (the phone picture)
1. A visitor opens the QR link and takes a selfie. The small upload server (port 8081) gives it to the dashboard, which shrinks it to a baseline JPEG (about 30 KB) and publishes `mesh/base/uplink_photo`.
2. The **base** cuts it into 1 KB `PHOTO_CHUNK` bundles addressed to the **rover** (bulk priority, custody, lifetime 300 s) and takes custody of all of them. The dashboard's **Rover screen** now says "sent from the base, the rover has received nothing yet".
3. **Mesh OFF and the rover out of reach:** the base has no usable next hop, so the chunks just sit in its store (you can see them as held bundles for the base). The rover's screen stays as static. If the rover is still in range, or when the link comes back for a while, only some chunks get through: the picture **struggles to form** from the top down and stalls with "link lost, the rest is waiting at the base".
4. **Mesh ON:** the drones deploy, the link comes up, the stored chunks are released (a link coming back "kicks" the retries), custody moves hop by hop, and the rover's `Reassembler` fills in the missing chunks. When the last one arrives it checks the sha256 and the picture is shown complete.
5. This is the same store-and-forward behaviour as the odometry that arrives late, but visible to a visitor. The message and the rover's own camera photo (rover → base) use separate ids, panels and Zenoh keys, so one can never overwrite the other.

### G. The simulation is restarted ("Restart simulation" in Gazebo)
1. Gazebo puts every model back at its start and sets the simulation time back to 0. The **link model** reads the simulation time from every pose message; when it goes **backwards** it publishes `mesh/reset` with a new epoch number.
2. **Every node** (base, rover, drones) forgets the run before: its store of waiting bundles, duplicate filter, neighbours, counters, the leader and election, pictures in progress and the mesh mode, which goes back to **OFF**. A killed drone comes back. Each drone also asks its mover to fly to its parking spot, in case it was still flying.
3. **Late packets of the old run are ignored:** `LEADER` and `MESH_CMD` carry the epoch, and a node drops any with an older epoch. Otherwise a heartbeat still in flight could switch the mesh back ON.
4. The **injector** puts packet loss and corruption back to 0. The **dashboard** bridge clears its graphs, log, pictures, counters and sliders and bumps its own epoch, and the open page sees the new epoch and **reloads itself**, so nothing from before is left on the screen.
5. The **mission** notices the rover jumping back to the start line and re-arms, so the next run drives again from the beginning.

## 10. The dashboard

`ros2 run mesh_dashboard dashboard` (or started by `mars_demo.sh`), then open **http://localhost:8080**. Plain HTML, CSS and JS with **no external URLs**, so it works on venue Wi-Fi with no internet.

| Panel | Shows |
|---|---|
| Header | mission clock, leader, term, lease, LIVE / OFFLINE, the **MESH ON/OFF** toggle |
| Crater profile · links · live routes | side view of the crater with units, working links, the command and odometry routes and bundles held per drone |
| Base links + Delivery | base → rover and rover → base state with route and round-trip time; sent / delivered / expired counts per type |
| Bundles held per node | how much each node is holding for custody |
| Odometry at the base | rover position over time; late samples (held during an outage) in amber, live in green |
| Event log | resends, expiries, elections, route changes, photos, kills and revives, plus the packet log for feed and photo chunks |
| Rover live camera | the 2 fps feed from the rover's Gazebo camera |
| Rover photo → base | **TAKE PHOTO** button, progress bar, the received picture and its sha256 result |
| Message from base → rover | QR code for the upload page, then the **Rover screen**: the picture the visitor sent, forming as chunks reach the rover, with a progress bar |
| Failure injection | drop % and corrupt % sliders, and a unit picker with **KILL** / **REVIVE** |

Server details (`mesh_dashboard/server.py`): `GET /` page, `GET /events` server-sent events (a snapshot 4 times a second plus log lines), `GET /qr.svg`, and `POST /api/mesh`, `/api/failure`, `/api/photo`. The POST routes check their input (unknown keys get 400) and need the `X-Passcode` header; a wrong guess costs one second. All Zenoh in and out is in `bridge.py` only.

## 11. Code map: which file does what

| File | What it does |
|---|---|
| **`mesh_common/__init__.py`** | The one shared definitions file: unit names, start positions, `TARGET_XY`, radio range (20 m), antenna heights, speeds, Zenoh key names, packet types, lifetimes, priorities, resend timers, the packet builder, `encode` / `decode` and CRC32, and `flight_time`. Every other package imports from here. |
| **`mesh_link_model/terrain.py`** | Loads the crater mesh (`crater.obj`) into a height grid and answers "is the straight line from A to B above the ground?" (`line_of_sight`). |
| **`mesh_link_model/link_model.py`** | Reads every unit's position from Gazebo (or a fake rover with `--fake`), decides which links are up (range + line of sight) and publishes `mesh/links` and `mesh/positions` 5 times a second. It is also the Zenoh **hub** the other programs connect to. |
| **`mesh_link_model/radio_channel.py`** | The "air": copies a unit's transmission to every unit whose link is up. It never reads or changes the bytes. |
| **`mesh_link_model/passthrough_injector.py`** | Stand-in failure injector: copies `air` to `rx`, optionally drops packets or flips bits (live settings on `mesh/failure/control`). |
| **`mesh_link_model/planner.py`** | The relay planner: tries 0, 1, 2… evenly spaced drones on the line rover → base at several heights until every hop has range and line of sight, then adds spare drones to shorten the longest hop. |
| **`mesh_link_model/sim_io.py`** | Moves a unit: Gazebo `set_pose` through the Python bindings (milliseconds), with a slower `gz service` fallback, or a Zenoh message in fake mode. |
| **`mesh_protocol/mesh_node.py`** | **One node per unit**, the heart of the protocol: duplicate filter, custody transfer, store / forward, expiry, CRC check, HELLO neighbours, flooding control packets, the rover's drive + watchdog + camera, the base's command/ACK/photo handling, kill / revive, and the leader loop. Large, but it only wires together the pure-logic modules below. |
| **`mesh_protocol/store.py`** | The custody store: `add`, `due` (what to (re)send now, best priority first, within the send budget), `ack`, `expire`, `kick`. Pure logic. |
| **`mesh_protocol/routing.py`** | `compute_routes`: shortest path by hops (ties: better link quality) where only relay drones may be in the middle. Pure logic. |
| **`mesh_protocol/leader.py`** | `Election` (lease, term, jitter) and `Coordinator` (the leader's planning: reachability checks, assignment of spots, which drone moves when, guessing a silent rover's position, keeping drones apart with `clear_of` and the remembered positions of silent drones). Pure logic. |
| **`mesh_protocol/mover.py`** | `Mover`: flies one drone smoothly in a background thread (speed, acceleration, ground clearance, facing the way it flies), so slow Gazebo calls never block the radio handling. |
| **`mesh_protocol/photo.py`** | `Camera` (newest Gazebo frame → image bytes), `chunks` and `Reassembler` (1 KB pieces + sha256). |
| **`mesh_protocol/mission.py`** | The autopilot: steers the rover to the target, asks for a photo on arrival, assist when stuck, detects a world reset and re-arms. |
| **`mesh_protocol/tools.py`** | Small commands: `mesh_ctl`, `mesh_monitor`, `send_cmd`, `move_unit`. |
| **`mesh_protocol/selftest.py`** | Runs the whole demo without Gazebo (fake rover) and checks the behaviour: connections, mesh on/off, resends, leader failover, kill / revive, photo integrity. |
| **`mesh_dashboard/bridge.py`** | The only code that talks Zenoh for the dashboard: builds the snapshot the page draws, collects events, sends the mesh / failure / photo commands, shrinks phone pictures and hands them to the base. |
| **`mesh_dashboard/server.py`** | A small standard-library web server: the page, live updates (server-sent events), the QR image, the passcode check and the POST routes; a second port serves only the phone upload page. |
| **`mesh_dashboard/static/`** | `index.html`, `style.css`, `app.js` (the drawing: map, graphs, panels) and `phone.html` (upload page). No external URLs. |
| **`mars_sim/scripts/generate_crater.py`** | Makes the crater terrain mesh (`crater.obj`) from a formula: floor, rim, plain, bumps. |
| **`mars_sim/scripts/build_relay_world.py`** | Writes the world file `worlds/mars_crater.sdf` (terrain, base, rover, drones, target flag, lighting, render engine). |
| **`mars_sim/models/`** | SDF models: `primary_rover` (with camera), `relay_drone`, `base_station`, `mars_crater`. |
| **`mars_sim/launch/sim.launch.py`** | Starts Gazebo with the crater world and registers the model path. |
| **`mesh_protocol/launch/mesh.launch.py`** | Starts the link model, radio channel, injector and the six mesh nodes (and optionally the dashboard, the mission, Gazebo). |
| **`scripts/mars_demo.sh`** | The one-command demo runner: preflight checks, starts and records every process, LAN / Cloudflare links (dashboard and phone page), QR window, `stop` and `status`. |

## 12. Design decisions and why

Every choice below was made on purpose, and several were forced by problems we hit (the full list is in [`ERRORS.md`](ERRORS.md)).

- **Simulated radio instead of real Wi-Fi.** The "air" is our own code: a link is up if the distance is ≤ 20 m **and** the terrain does not block the line. This makes the demo repeatable and lets us cut links on purpose. Real radios are more complicated (fading, interference), which we do not model.
- **20 m range and a 60 m drive.** At 20 m the rover needs 0 drones near the base, 1 at x ≈ -12, 2 at x ≈ 0 and 3 at the target, so all four drones have a job and the visitor sees the chain grow.
- **A small crater (36 m wide, 6 m deep).** In the first, much larger world the rover and drones were invisible on screen. Shrinking everything equally did not work either, because the 10 m base mast could then see over a shallow rim, so we tuned the depth and the range against the real mast height.
- **Elected leader instead of a fixed coordinator.** A single fixed coordinator is a single point of failure. With an election (lease + term), killing the leader only costs about 2 s. The leader plans using positions **reported over the mesh**, never the true positions, so it behaves like a real node that only knows what it has been told.
- **DTN (custody + store and forward) instead of plain flooding.** Earlier versions flooded every packet. DTN is what real space networks do: links are intermittent, so data must survive an outage. It is also why odometry arrives *late* instead of being lost.
- **Lifetimes chosen against the watchdog.** A command lives 1 s, shorter than the rover's 2 s watchdog, so the rover never acts on an old order after a reconnection. Odometry lives 120 s because old samples are still worth having. Camera frames live 1 s because an old frame is worthless.
- **Priorities and a send budget (40 bundles/s).** Without them a photo or video could delay a drive command. With them, commands always go first.
- **Drones move themselves.** Each drone node flies itself in its own process. A single central program moving all four got only about 1 `set_pose` per second through; separate processes get about 18 per second each.
- **Gazebo Python bindings instead of the ROS–Gazebo bridge.** `set_pose` through the bindings takes milliseconds; calling the `gz service` command takes about 0.3 s per call. The demo needs fast teleport steps for smooth flight, so no ROS 2 bridge is used.
- **Look-ahead of 5 s and a position guess.** With the 20 m radio the rover can drive out of reach and stop reporting. The leader now guesses where it is from its last speed (for up to 4 s), and starts moving drones 5 s ahead so the link is not cut.
- **Zenoh bound to 127.0.0.1 only.** One hub at `tcp/127.0.0.1:7447`, multicast off, so our programs never find (or disturb) teammates' laptops on venue Wi-Fi, and it avoids IPv6 errors.
- **Web dashboard instead of the PyQt GUI.** It runs in any browser (including a phone), needs no extra install, has no external URLs, and could be given a passcode and a QR upload page. The PyQt prototype was never merged.
- **Passcode only on the controls.** Visitors can watch, but only the team can switch the mesh, kill drones or take photos. It is clear-text over http, so it is a polite lock, not security.
- **`ogre` render engine.** The default `ogre2` fails on virtual GPUs (VMs). The classic engine works on both.
- **Pure-logic modules.** Store, routing, election/coordinator, photo and mover contain no Zenoh or Gazebo calls, so each has fast unit tests (117 in total), and the self-test runs the whole system without Gazebo.

## 13. Commands and options

| Command | What it does |
|---|---|
| `./scripts/mars_demo.sh start\|stop\|status\|deps` | The one-command demo runner ([section 4](#4-quick-start-demo-day)) |
| `./scripts/phone_tunnel.sh` · `./scripts/run_demo.sh start [nogui]\|stop` | VM version: classic `ogre` renderer, plus a Cloudflare tunnel for the phone page |
| `ros2 launch mars_sim sim.launch.py` | Gazebo with the crater world (`world:=empty_mars.sdf` for the old flat world) |
| `ros2 launch mesh_protocol mesh.launch.py` | Link model, radio, injector, 6 mesh nodes. Options: `mesh:=on`, `dashboard:=true`, `mission:=true`, `fake:=true` (no Gazebo), `injector:=none` (use a real injector), `sim:=true` (also start Gazebo) |
| `ros2 run mesh_dashboard dashboard [--port 8080] [--host ADDR] [--passcode TEXT]` | The dashboard (default host `127.0.0.1`) |
| `ros2 run mesh_protocol mission [--speed 0.5] [--no-assist]` | Drive the rover from the base to the target. **Assist:** if the rover gets commands but is stuck for 6 s, it is lifted 1.5 m forward. It stays alive after arriving and re-arms after a world reset |
| `ros2 run mesh_protocol mesh_ctl on\|off` | The mesh switch from a terminal (the dashboard follows it) |
| `ros2 run mesh_protocol mesh_monitor` | Live text view of the base, links and counters |
| `ros2 run mesh_protocol send_cmd 1.0 [0.2]` | One drive command from the base (0 = stop) |
| `ros2 run mesh_protocol move_unit drone_1 0 0 12` / `move_unit home` | Teleport a unit / park all drones (`--fake` without Gazebo) |
| `ros2 run mesh_protocol selftest` | The whole demo without Gazebo (~8 min) |
| `python3 -m pytest src/<package>/test` | Unit tests of one package |
| `mesh_node --unit drone_1 [--mesh on] [--fake]` · `link_model` · `radio_channel` · `passthrough_injector` | Single pieces (normally started by the launch file) |

**Without Gazebo** (fake rover, good for working on the protocol or dashboard):
```bash
ros2 launch mesh_protocol mesh.launch.py fake:=true mission:=true dashboard:=true
```

Logs of `mars_demo.sh` are in `/tmp/mars_demo/logs/` (one file per program).

## 14. Developing and changing things

Always after changing files: `colcon build --symlink-install`, then `source install/setup.bash` in each terminal.

| I want to change… | Edit | Then |
|---|---|---|
| Crater size / depth / bumps, terrain size | numbers at the top of `src/mars_sim/scripts/generate_crater.py` | regenerate (below) |
| Where the base, rover, target or parked drones start | `src/mars_sim/scripts/build_relay_world.py` **and** `START_XY` / `TARGET_XY` in `src/mesh_common/mesh_common/__init__.py` | regenerate (below) |
| Radio range, antenna heights, rover speed, relay heights, drone speed | `src/mesh_common/mesh_common/__init__.py` (`RADIO_RANGE`, `ANTENNA_HEIGHT`, `ROVER_SPEED`, `RELAY_HEIGHTS`, `DRONE_SPEED`, `DRONE_ACCEL`, `TARGET_REACH`) | relaunch |
| Rover / drone / base shape, camera | `src/mars_sim/models/<model>/model.sdf` | relaunch |
| Gravity, sun, sky, render engine | `build_relay_world.py` (the world file is generated) | regenerate |

Regenerate the terrain and world:
```bash
cd ~/mars_ws/src/mars_sim && python3 scripts/generate_crater.py && python3 scripts/build_relay_world.py
cd ~/mars_ws && colcon build --symlink-install
ros2 run mesh_protocol selftest              # check the demo still works
```
- **Never hand-edit** `worlds/mars_crater.sdf` or `models/mars_crater/meshes/crater.obj`: both are generated. Commit them **together with** the scripts.
- Changed the crater, radio range or positions? Check the story still works: the rover must lose the base inside the crater, and the base must not see the target. Run the self-test and update section 7.
- Keep the names stable (`mars_world`, `primary_rover`, `drone_1…4`, `base_station`, topics, Zenoh keys). Other packages depend on them.
- Keep the packet format and keys in `mesh_common`. Never decode packets in a failure injector: flip bits in the raw bytes, the nodes must survive it.

**Failure-injection teammates:** the full module would be a new package `failure_injection` that subscribes to `mesh/*/air`, changes, drops or delays the raw bytes, and publishes to `mesh/<unit>/rx`. Copy the interface of `mesh_link_model/passthrough_injector.py` (same keys, live settings on `mesh/failure/control`, events on `mesh/failure/events`) and run the mesh with `injector:=none`.

**Tests:** `mesh_common` 5, `mesh_protocol` 78, `mesh_dashboard` 22, `mesh_link_model` 4 (109 in total, all passing on `main`). The camera feed, phone upload and packet events are only checked by hand in the live run.

## 15. Repository layout

```
mars_ws/                              ← repo root = ROS 2 workspace
├── README.md                         ← this file
├── ERRORS.md                         ← problems we hit + how we fixed them
├── scripts/
│   ├── mars_demo.sh                  ← one-command demo runner (start / stop / status / deps)
│   ├── run_demo.sh, phone_tunnel.sh  ← VM versions (classic ogre renderer, phone tunnel)
│   └── requirements.txt, apt-packages.txt   ← extra packages
├── docs/
│   ├── OPEN_DAY.md                   ← camera, photos, phone QR, passcode
│   ├── V3_CHANGES.md, RELAY_DEMO.md, OPTIMIZATIONS.md   ← v3 notes and ideas (RELAY_DEMO is the old two-base guide)
│   ├── v4/EXPLAINER.md               ← v4 Step 1 and Step A in simple words
│   └── superpowers/                  ← design specs and plans (the v4 plan is outdated)
└── src/
    ├── mars_sim/                     ← Gazebo world, models, launch (ament_cmake)
    │   ├── scripts/generate_crater.py    ← makes models/mars_crater/meshes/crater.obj
    │   ├── scripts/build_relay_world.py  ← makes worlds/mars_crater.sdf
    │   └── worlds/  models/  launch/sim.launch.py
    ├── mesh_common/                  ← names, layout, Zenoh keys, packet format + CRC
    ├── mesh_link_model/              ← terrain + line of sight, link model, radio channel,
    │                                    relay planner, stand-in injector
    ├── mesh_protocol/                ← mesh node (DTN), store, routing, leader election, photo,
    │                                    mover, mission, tools, self-test, launch
    └── mesh_dashboard/               ← web dashboard (server, offline page, phone upload page)
```
`build/`, `install/` and `log/` come from `colcon build` and are **not** in git.

## 16. Team workflow (git)

**Branches today:** `main` is the integrated final build (tag `demo-final`). `v3-dtn`, `integration` and `v4-mission` are old points on the same history; `Ronak` and `advaith` are already merged. `Ronak_GUI` (PyQt6) is **not** merged and not used. Until the demo, only fix real bugs, and do it by Pull Request.

```bash
cd ~/mars_ws && git checkout main && git pull       # ALWAYS first
git checkout -b feature/<short-name>                # e.g. feature/failure-injection
# ... edit, then build and test:
colcon build --symlink-install && source install/setup.bash && python3 -m pytest -q src/mesh_protocol/test
git add -A && git commit -m "Add bit-flip injector" && git push -u origin feature/<short-name>
```
Then open a **Pull Request** on GitHub so someone checks it before it is merged.

- Pull before you start, push when you stop.
- One person per file at a time. Say in the group chat which file you are editing.
- Don't commit `build/`, `install/` or `log/`.
- After switching branches, run `colcon build --symlink-install` and open a new terminal, because old build leftovers can break Gazebo's startup.

**If git says "conflict":** open the file, keep the right lines, delete the `<<<<<<<`, `=======` and `>>>>>>>` markers, then `git add -A && git commit -m "Merge latest main" && git push`. Not sure which version is right? Ask whoever made the other change.

## 17. Known limits

Be honest with visitors about these. The first two are why the self-test prints `[KNOWN ISSUE]` lines.

1. **A short link drop when the drones first deploy.** Right after MESH ON, the link can drop for a few seconds, once, because a second drone may briefly declare itself leader while they all start moving. It heals by itself. Some self-test limits are loosened for this and marked `TEMP` in `selftest.py`.
2. **Two leaders after a kill + revive ("split brain").** With one drone dead the chain is cut, so the far drone leads its own group, and the group around the real leader may not be redeployed within 15 s. Keep the failure demo short; if it goes wrong, switch the mesh OFF and ON, or restart the sim. (Reviving a killed leader itself is fixed.)
3. **Drones can still overlap in extreme cases.** There is no collision physics, so the leader keeps drones apart itself: a killed drone hangs frozen where it died and is remembered as a "ghost", new relay spots are kept at least 2.5 m away from ghosts and from each other (1.5 m if the chain is tight), and a revived drone that wakes up on top of another is sent away. In a random test of kills and revives this removed the overlaps when none or one drone was dead; it can still happen when two or three drones are dead and the only possible relay spot is where a dead drone hangs.
4. **A photo right after a drone kill may not finish** until the relay chain has re-formed. Do the failure demo last and give it a few seconds.
5. **The failure-injection module is a stand-in.** Drop and corrupt work; scripted failures and delays are not built.
6. **Drones are not real flying robots.** They glide with `set_pose`: no flight physics, wind or battery.
7. **The phone upload page has no passcode.** Anyone with the QR link can send a picture to the rover's screen (images only, 15 MB max). Don't post the link publicly: with `--tunnel` it is reachable from the internet while the demo runs. The tunnel addresses change on every run.
8. **Weak passcode protection.** It is sent in clear text over http; use a tunnel (TLS) on an untrusted network.
9. **Run it once on the demo machine before open day.** The Gazebo window, camera, QR and tunnel are not covered by automated tests.

## 18. Questions visitors ask (and short answers)

**Why does the rover need drones at all?** Hills block radio waves. The base can't see the rover inside the crater, but a drone hovering higher can see both, so data hops over the obstacle.

**Is this a real radio?** No. It is a model: two units hear each other if they are within 20 m and the straight line is above the ground. Real radios are messier, but the logic of the network is the same.

**What is DTN and why not just use the internet's TCP?** TCP assumes a connection that stays up. In space, links drop for minutes and signals are slow. DTN stores data at each hop and forwards it when a link appears. We built a simplified version of that idea: custody transfer, store and forward, lifetimes, priorities.

**What happens if a drone breaks?** The neighbours notice within about 1.5 s, the leader re-plans and re-routes, and waiting data goes the new way. If it was the leader, another drone takes over in about 2 s.

**What happens if packets are corrupted or lost?** Corrupted ones fail the CRC32 check and are discarded; lost or discarded ones are resent until they arrive or expire. Try the sliders on the dashboard.

**Why did the rover stop?** Its safety watchdog stops it if it receives no command for 2 s. That is the right behaviour when the link is gone, and why we need the mesh.

**How many drones are needed?** As few as possible: 0 near the base, 1, 2, then 3 as the rover goes deeper. The spare drone is used to shorten the longest hop, so all four help.

**Who decides where the drones go?** The elected leader drone, using positions the other drones report to it over the mesh.

**Why is some data amber / "late"?** The rover recorded those samples while it had no link. They were held in its store and delivered when the mesh reconnected. Nothing was lost.

**How do we know the photo is not damaged?** It travels in 1 KB chunks with a sha256 fingerprint of the whole file; the base only shows "OK" if the fingerprint of the reassembled picture matches.

**What is the phone selfie for?** It is a message from mission control to the rover. The visitor plays mission control: the picture goes from the base through the drones to the rover, whose screen shows it forming. If you send it while the mesh is OFF and the rover is out of reach, it cannot get through: the pieces wait at the base, and the picture only appears after the mesh is switched ON.

**Is the data encrypted or secure?** No. The mesh has no encryption or authentication, and the passcode is only a lock for the demo controls. Real missions would need both.

**Do you simulate the delay to Earth?** No. The first plan had an orbiter and Earth with real light-time delay, but that part was dropped for this demo. The delay-tolerant design is there, and the outage in the crater plays the role of the "disruption".

**Are those real flying drones?** They are simplified: we place them with small steps so they glide smoothly, but there is no rotor physics, wind or battery. Real Mars helicopters exist (Ingenuity flew on Mars), so the idea is realistic.

**What is the weakest part?** Be honest: see [Known limits](#17-known-limits). The leader handover can cause a brief pause when the drones first deploy, and killing a drone right after a revive can briefly leave two groups.

## 19. Troubleshooting

| Problem | Fix |
|---|---|
| `ModuleNotFoundError: No module named 'zenoh'` | `python3 -m pip install --user --break-system-packages -r scripts/requirements.txt` |
| Zenoh errors like `AttributeError … payload` | The wrong `zenoh` package is installed. `pip uninstall zenoh`, then install `eclipse-zenoh` 1.10.x |
| `No module named 'gz.transport13'` | `sudo apt install -y python3-gz-transport13 python3-gz-msgs10` |
| `error: externally-managed-environment` (pip) | add `--break-system-packages` |
| `No module named mesh_common` / `Package 'mesh_protocol' not found` | `cd ~/mars_ws && source install/setup.bash` in that terminal |
| `Unable to find uri[model://...]` or `package 'base_station_gui' not found` | `colcon build --symlink-install`, then open a new terminal. Old build leftovers from another branch can cause the second one |
| `[link_model] WARNING: no position yet` | Gazebo isn't running or is paused (press ▶). Check `gz topic -l \| grep pose` |
| Dashboard shows OFFLINE / no leader | Start the mesh first; the drones elect a leader about 2 s after they start |
| Drones don't move | Check `gz service -l \| grep set_pose`; the world must be `mars_world` |
| Rover doesn't move | Mesh OFF and rover in the crater = no link (that's the demo). Otherwise press ▶ or check `gz topic -l \| grep rover` |
| `ASSIST #n` lines in the mission | The rover slipped on the crater wall and was lifted forward. Occasional is fine |
| `port 7447 busy` / `Address already in use` | An old run is still going: `./scripts/mars_demo.sh stop`, or `pkill -f "mesh_protocol\|mesh_link_model"` |
| Dashboard on another laptop sees nothing | `mars_demo.sh start` listens on the whole LAN; the manual `ros2 run mesh_dashboard dashboard` uses `127.0.0.1` unless you pass `--host 0.0.0.0` |
| Gazebo window black / crashes (VM) | Use `./scripts/phone_tunnel.sh` (classic `ogre` renderer). On a real laptop: `sudo ubuntu-drivers install`, reboot |
| QR page says "pip install segno", or no camera / photo upload | `./scripts/mars_demo.sh deps` (installs `segno` and `Pillow`) |
| Phone can't open a QR link | Start with `./scripts/mars_demo.sh start --tunnel` and check the script printed `phone page: https://…`. Without `--tunnel` the phone must be on the same Wi-Fi (campus Wi-Fi often blocks that). On a NAT'd VM only the tunnel works |
| `--tunnel` prints "tunnel failed" or no URL | The laptop has no internet, or the network blocks Cloudflare. Use a phone hotspot. After a reboot run `./scripts/mars_demo.sh deps --tunnel` (cloudflared lives in `/tmp`). Check `/tmp/mars_demo/logs/tunnel.log` and `phone_tunnel.log` |
| `port 8081 (phone upload page) busy` | An old run is still going: `./scripts/mars_demo.sh stop` |
| Controls ask for a passcode | See [section 4](#4-quick-start-demo-day). Cancel leaves them locked; a wrong one asks again |
| Self-test fails | Read the `FAIL` lines and `/tmp/mesh_selftest.log`. `[KNOWN ISSUE]` lines do not count as failures |
| `SSL certificate problem` on campus Wi-Fi | Use a phone hotspot |

More in [`ERRORS.md`](ERRORS.md).

## 20. Roadmap

**Done**
- [x] World: crater terrain, rover with camera, 4 drones, base, science target
- [x] Shared definitions (`mesh_common`): packet format + CRC32, unit names, Zenoh keys
- [x] Link model + radio channel: line of sight + range → `mesh/links`
- [x] DTN mesh protocol: custody transfer, store and forward, lifetimes, priorities, routed bundles, re-routing when a drone fails
- [x] Leader drone: election with a lease, planning, route table (kill it: another takes over)
- [x] Smooth drone flight (stepped `set_pose` with speed-up and slow-down)
- [x] Mission: the base drives the rover to the target over the mesh; re-arms after a world reset
- [x] Web dashboard: map, routes, held bundles, delivery, late odometry, events, failure controls
- [x] Rover camera: live feed, photo on arrival and by button, sha256 check, phone QR upload
- [x] Passcode-protected controls, one-command demo runner, Cloudflare tunnels for the dashboard and the phone upload page
- [x] v4 Step 1 (one base, science target, 20 m radio) and Step A (kill / revive fix, review fixes)
- [x] Everything merged into `main` and tagged `demo-final`

**Open**
- [ ] Final run of the full demo on the demo laptop, in the order of [section 5](#5-the-demo-script)

**Later (after open day)**
- [ ] Fix the leader gap and the split brain after kill + revive ([section 17](#17-known-limits)); then restore the old self-test limits and turn the `[KNOWN ISSUE]` check back into a real check
- [ ] Real failure-injection package: bit flips, drops, delay, scripted failures
- [ ] Drones with real flight physics (multicopter plugin, velocity control). More ideas: [`docs/OPTIMIZATIONS.md`](docs/OPTIMIZATIONS.md)
- [ ] New screenshots of the dashboard and the world for this README (the old ones showed the two-base v3 build and were removed)

## 21. More docs

| File | What is in it |
|---|---|
| [`docs/OPEN_DAY.md`](docs/OPEN_DAY.md) | camera, photos, phone QR, passcode: how they work and how to run them |
| [`docs/v4/EXPLAINER.md`](docs/v4/EXPLAINER.md) | v4 Step 1 and Step A in simple words, with visitor talking points |
| [`docs/V3_CHANGES.md`](docs/V3_CHANGES.md) | what changed from v2 to v3 (DTN) and why |
| [`docs/OPTIMIZATIONS.md`](docs/OPTIMIZATIONS.md) | ideas for making the mesh and the flight better |
| [`docs/RELAY_DEMO.md`](docs/RELAY_DEMO.md) | the old two-base demo guide (kept for history; this README is current) |
| [`docs/superpowers/specs/`](docs/superpowers/specs/) | the v3 design and the v4 plan (outdated, kept for history) |
| [`ERRORS.md`](ERRORS.md) | every problem we hit and the fix |
