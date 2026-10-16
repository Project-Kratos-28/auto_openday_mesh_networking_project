<!-- mars-mesh-readme -->
# Mars Rover Relay Mesh — Simulation

A ROS 2 + Gazebo simulation of a rover that loses contact with its base station inside a crater, and a team of relay drones that form a **mesh network** to carry its data back.

![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-blue) ![Gazebo Harmonic](https://img.shields.io/badge/Gazebo-Harmonic-orange) ![Ubuntu 24.04](https://img.shields.io/badge/Ubuntu-24.04-E95420)

<!--PREVIEW-->

---

## 1. The idea

```
        drone (relay)  ●───────────●  base station
                      /     clear line
           clear line/
                    /      ▲ crater rim blocks the direct line
   rover ● ────────X───────┘
   (crater floor)
```

1. The **primary rover** drives into a crater. The rim blocks the direct radio line to the **base station**.
2. **Relay drones** move to positions where they can "see" both sides (for example, above the rim).
3. Data hops **rover → drone(s) → base station** using our own **mesh network protocol**, built on **[Zenoh](https://zenoh.io)**.
4. A **failure injection node** corrupts, drops and delays packets, and can switch units off, so we can show that the protocol survives errors.
5. The **base station GUI** shows the rover's data, the live network links, and the route each packet took.

A radio link only exists when two units have **line of sight** over the terrain **and** are within **radio range**. The simulation decides this from the real 3D terrain.

## 2. System overview

| Part | What it does | Package (in `src/`) | Status |
|---|---|---|---|
| Simulation | Mars terrain, rover, drones, base station | `mars_sim` | ✅ World + models done |
| ROS bridge | Moves positions / commands between Gazebo and ROS 2 | `mars_sim` | ⏳ Next |
| Link model | Decides which units can reach each other (line of sight + range) | `mesh_link_model` | 📝 Planned |
| Radio channel | Passes packets on **only** over working links | `mesh_link_model` | 📝 Planned |
| Failure injection | Corrupts / drops / delays packets, kills units or links on demand | `failure_injection` | 📝 Planned |
| Mesh protocol (Zenoh) | One node per unit: neighbour discovery, routing, forwarding, ACKs, CRC checks | `mesh_protocol` | 📝 Planned |
| Shared definitions | Packet format, unit names, Zenoh key names (one Python module everyone imports) | `mesh_common` | 📝 Planned |
| Base station GUI | Map, links, packet routes, rover telemetry, failure stats, send commands | `base_station_gui` | 📝 Planned |

## 3. Communication architecture (Zenoh)

**Zenoh** is a fast publish/subscribe protocol. Programs `put` data on a **key** (like `mesh/rover/tx`), and anyone who subscribed to that key gets it. We use it in two ways:

- **Mesh packets** between units: each unit's protocol node opens a Zenoh session and sends/receives packets on Zenoh keys (Python: `eclipse-zenoh`).
- **(Optional) ROS 2 over Zenoh:** ROS 2 itself can run on Zenoh instead of DDS (`rmw_zenoh_cpp`). The team decides whether to switch, and the code works either way.

**Packet path:** a packet never goes straight from one unit to another. It always passes through the radio channel and the failure injector:

```
 Gazebo ──ROS 2──► link model ──► mesh/links  (who can reach whom)
                                      │
 [rover node] ── mesh/rover/tx ──► radio channel ── mesh/drone_1/air ──► failure injector ── mesh/drone_1/rx ──► [drone_1 node]
   protocol         (send)        (link up? else drop)                  (corrupt / drop / delay)    (deliver)      forwards again...
                                                                                                                       │
 [base node] ◄── mesh/base/rx ◄── failure injector ◄── radio channel ◄── mesh/drone_1/tx ◄────────────────────────────┘
      │
      └──► mesh/base/telemetry ──► Base station GUI          GUI commands ──► mesh/base/cmd ──► back through the mesh to the rover
```

**Units:** `rover`, `drone_1` … `drone_4`, `base`

**Zenoh keys (draft, agree as a team):**

| Key | Published by | Read by | Contents |
|---|---|---|---|
| `mesh/<unit>/tx` | unit's protocol node | radio channel | packets the unit sends |
| `mesh/<unit>/air` | radio channel | failure injector | packets that made it over a working link |
| `mesh/<unit>/rx` | failure injector | unit's protocol node | packets delivered to the unit (maybe corrupted) |
| `mesh/links` | link model | protocol, GUI | link table: pair, up/down, distance, quality |
| `mesh/positions` | link model | GUI | x, y, z of every unit |
| `mesh/base/telemetry` | base protocol node | GUI | rover data that reached the base (+ route, delay) |
| `mesh/base/cmd` | GUI | base protocol node | commands to send to the rover |
| `mesh/failure/control` | GUI / terminal | failure injector | change failure settings while running |
| `mesh/failure/events` | failure injector | GUI | log of what was corrupted, dropped or killed |
| `mesh/stats/<unit>` | protocol nodes | GUI | sent / received / forwarded / bad-CRC / retransmit counts |

**Packet format** (JSON, defined once in `mesh_common`):
```json
{
  "src": "rover",          "dst": "base",           "next_hop": "drone_1",
  "seq": 42,               "ttl": 6,                "path": ["rover", "drone_1"],
  "type": "TELEMETRY",     "sent_time": 1729071234.512,
  "payload": {"x": 1.2, "y": -0.4, "battery": 87},
  "crc32": 3735928559
}
```
- `type` is one of `TELEMETRY`, `COMMAND`, `ACK` or `HELLO`.
- `crc32` is a checksum of the packet without the `crc32` field. The receiver recalculates it. If it doesn't match, the packet was corrupted: it's dropped, and the sender retransmits when no `ACK` arrives.
- The failure injector flips bits in the **raw bytes**, so most corrupted packets aren't even valid JSON any more. Receivers must treat "can't parse" exactly like "bad CRC": drop the packet, no crash. In a test of 10,000 packets with 1–8 flipped bits, every one was caught by these two checks together.

**Failure injection** settings (`src/failure_injection/config/failures.yaml`, draft):
```yaml
corrupt_probability: 0.05    # chance a packet gets bit errors
bit_flips: [1, 8]            # how many bits to flip in a corrupted packet
drop_probability: 0.02       # chance a packet disappears
delay_ms: [20, 150]          # random extra delay per hop
scheduled:                   # scripted failures for the demo
  - {time: 60, action: kill_unit, unit: drone_2}     # drone_2 stops relaying
  - {time: 90, action: revive_unit, unit: drone_2}
  - {time: 120, action: link_down, pair: [drone_1, base]}
```
You can change the settings live by publishing to `mesh/failure/control`, for example `{"corrupt_probability": 0.3}`.

## 4. Setup on a new laptop (step by step)

Do this **once** per laptop. It takes about 30–60 minutes, mostly downloading.

**You need:** **Ubuntu 24.04** (native install or dual boot is best. On Windows, WSL2 with Ubuntu 24.04 also works) and about **15 GB** of free disk space.

> 📶 **On campus Wi-Fi?** If a download fails with `SSL certificate problem: self-signed certificate`, switch to a phone hotspot for the setup. The campus firewall blocks some downloads.

Open a terminal (`Ctrl + Alt + T`) and run each block in order. Lines starting with `#` are explanations.

**Step 1: Language settings** (ROS 2 needs UTF-8)
```bash
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

**Step 2: Add the ROS 2 package source** (tells Ubuntu where to download ROS from)
```bash
sudo apt install -y software-properties-common curl
sudo add-apt-repository -y universe
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
```

**Step 3: Install ROS 2 Jazzy + build tools**
```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y ros-jazzy-desktop ros-dev-tools git
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc     # load ROS 2 in every new terminal
source ~/.bashrc
```

**Step 4: Install Gazebo Harmonic + the ROS–Gazebo bridge**
```bash
# add the Gazebo package source
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
# install Gazebo itself (gives the `gz` command) and the ROS 2 connection
sudo apt update && sudo apt install -y gz-harmonic ros-jazzy-ros-gz
```
> ⚠️ Use **Gazebo Harmonic** (`gz sim`) only. Do **not** install Gazebo Classic (`gazebo`, `gazebo11`, `gazebo_ros`). It conflicts with Harmonic, and many online tutorials still use it.

**Step 5: Install Zenoh** (networking library for the mesh protocol)
```bash
pip install --break-system-packages eclipse-zenoh
# optional: run ROS 2 itself over Zenoh instead of DDS
sudo apt install -y ros-jazzy-rmw-zenoh-cpp
```

**Step 6: Check everything works.** Open a **new** terminal, then run:
```bash
echo $ROS_DISTRO                                   # → jazzy
gz sim --version                                   # → Gazebo Sim, version 8.x
ros2 pkg list | grep ros_gz_sim                    # → ros_gz_sim
python3 -c "import zenoh; print('zenoh OK')"       # → zenoh OK
gz sim shapes.sdf                                  # a window with simple shapes opens (close it after)
```

Zenoh test: save this as `zenoh_hello.py` and run `python3 zenoh_hello.py`. It should print `received: ...`.
```python
import json, time
import zenoh

session = zenoh.open(zenoh.Config())                        # join the Zenoh network
session.declare_subscriber(                                 # listen on a key
    "mesh/test", lambda sample: print("received:", sample.payload.to_string()))
time.sleep(0.5)                                             # give the subscriber a moment
session.put("mesh/test", json.dumps({"src": "rover", "msg": "hello from Mars"}))  # publish
time.sleep(1)
session.close()
```

Optional, ROS 2 over Zenoh: run the router `ros2 run rmw_zenoh_cpp rmw_zenohd` in its own terminal, and `export RMW_IMPLEMENTATION=rmw_zenoh_cpp` in every terminal that runs ROS 2 nodes.

## 5. Get the project and run it

**Step 1: Get access.** This repo is **private**. Ask the repo owner to add your GitHub account (repo → Settings → Collaborators), then **accept the invite** from the email or from github.com/notifications.

**Step 2: Log in to GitHub from the terminal** (once per laptop):
```bash
sudo apt install -y gh
git config --global user.name "Your Name"
git config --global user.email "you@example.com"        # the email on your GitHub account
gh auth login --hostname github.com --git-protocol https --web
gh auth setup-git                                       # lets git use this login
```
`gh auth login` shows a one-time code. Press Enter, and a browser opens at github.com/login/device. Type the code there and click **Authorize**. Keep the terminal open until it says `Logged in as ...`.

**Step 3: Download (clone), build, run.** This repository **is** a ROS 2 workspace, with packages in `src/`. Always clone it into `~/mars_ws`:
```bash
git clone https://github.com/Project-Kratos-28/auto_openday_mesh_networking_project.git ~/mars_ws
cd ~/mars_ws
colcon build --symlink-install               # build all packages
source install/setup.bash                    # load our packages into this terminal
ros2 launch mars_sim sim.launch.py           # start the simulation
```
> Tip: add `source ~/mars_ws/install/setup.bash` to `~/.bashrc` so every new terminal knows our packages.

Gazebo opens with the crater world. Press ▶ (bottom-left) if the simulation is paused.

**Try it** (in a second terminal):
```bash
# drive the rover forward at 1 m/s   (send x: 0 to stop)
gz topic -t /rover/cmd_vel -m gz.msgs.Twist -p "linear: {x: 1.0}"

# move drone_1 to a relay spot above the crater rim
gz service -s /world/mars_world/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean \
  --timeout 2000 --req 'name: "drone_1", position: {x: 100, y: 0, z: 43}'

# put the rover back at the start
gz service -s /world/mars_world/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean \
  --timeout 2000 --req 'name: "primary_rover", position: {x: 0, y: 0, z: 0.05}, orientation: {w: 1}'
```

Other world: `ros2 launch mars_sim sim.launch.py world:=empty_mars.sdf` (flat test ground).

## 6. Repository structure

```
mars_ws/                          ← repository root = ROS 2 workspace
├── README.md                     ← this file
├── ERRORS.md                     ← problems we hit + how we fixed them
├── docs/                         ← guides and images
└── src/
    ├── mars_sim/                 ← simulation package ✅
    │   ├── worlds/               ← world files (.sdf): mars_crater.sdf, empty_mars.sdf
    │   ├── models/               ← mars_crater, primary_rover, relay_drone, base_station
    │   ├── launch/sim.launch.py  ← starts Gazebo with a world
    │   └── scripts/generate_crater.py  ← regenerates the crater terrain mesh
    ├── mesh_common/              ← packet format, unit names, Zenoh keys (planned)
    ├── mesh_link_model/          ← line of sight + range, radio channel (planned)
    ├── failure_injection/        ← corrupt / drop / delay / kill (planned)
    ├── mesh_protocol/            ← Zenoh mesh node, one per unit (planned)
    └── base_station_gui/         ← base station GUI (planned)
```

`build/`, `install/` and `log/` are created by `colcon build` and are **not** stored in git.

## 7. Simulation reference

**World:** `mars_world` · gravity **3.71 m/s²** (Mars) · units in metres · crater centre = origin `(0, 0)`

| Entity | Name in Gazebo | Position (x, y, z) | Notes |
|---|---|---|---|
| Terrain | `mars_crater` | — | 480 × 480 m. Crater 200 m wide. **Floor z = 0**, rim top z = 33, plain z = 25 |
| Primary rover | `primary_rover` | (0, 0, 0) | 4-wheel differential drive, comms mast |
| Relay drones | `drone_1` … `drone_4` | (±6, 0, 3), (0, ±6, 3) | Hover in place (static) for now; move with `set_pose` |
| Base station | `base_station` | (200, 0, 25) | Outside the rim; antenna top ≈ z = 36 |

From the crater floor, the rim **blocks** the direct line to the base station. A drone about 10 m above the rim, for example at (100, 0, 43), can see both.

**Change the crater:** edit the numbers at the top of `src/mars_sim/scripts/generate_crater.py`, then:
```bash
cd ~/mars_ws/src/mars_sim && python3 scripts/generate_crater.py
cd ~/mars_ws && colcon build --symlink-install
```

## 8. Working on the Gazebo simulation (team guide)

Anyone in the team can work on the simulation. Follow this so we don't overwrite each other's work.

### Which file controls what

| I want to change... | Edit this file | Rebuild needed? |
|---|---|---|
| Where the rover / drones / base start | `src/mars_sim/worlds/mars_crater.sdf` (the `<pose>` lines) | No, just relaunch |
| Add or remove a drone | `worlds/mars_crater.sdf`: copy or delete one `<include>` block and give it a new `<name>` | No |
| Gravity, sky colour, sun, lighting | `worlds/mars_crater.sdf` | No |
| Rover shape, wheels, speed topic | `models/primary_rover/model.sdf` | No |
| Drone shape | `models/relay_drone/model.sdf` (all 4 drones use it) | No |
| Base station shape | `models/base_station/model.sdf` | No |
| Crater size, depth, bumps | `scripts/generate_crater.py`, then run it (see below) | Run the script |
| Add a **new** model or world file | new folder in `models/` (needs `model.config` + `model.sdf`) or a new `.sdf` in `worlds/` | **Yes** |
| Launch options | `launch/sim.launch.py` | No |

"No rebuild" works because we build with `--symlink-install`: the installed files point back to your edited files. Just close Gazebo (`Ctrl + C`) and launch again. **New** files always need `colcon build --symlink-install`.

### The edit → test → share loop

```bash
# 1. Start from the latest version (ALWAYS do this first)
cd ~/mars_ws && git checkout main && git pull

# 2. Make a branch for your change
git checkout -b sim/<short-name>             # e.g. sim/add-mountains

# 3. Edit files, then test
colcon build --symlink-install && source install/setup.bash
ros2 launch mars_sim sim.launch.py           # check it opens with no red errors in the terminal

# 4. Save and upload your branch
git add -A
git commit -m "Add two mountains east of the crater"
git push -u origin sim/<short-name>
```
**Step 5:** On GitHub, click **Compare & pull request**. Write what you changed, add a screenshot of Gazebo if it changes how things look, then click **Create pull request**. After a quick check by someone else, click **Merge**.

### Rules to avoid clashes

1. **Pull before you start, push when you stop.** Don't sit on changes for days.
2. **One person per file at a time.** Tell the team in the group chat: "I'm editing `mars_crater.sdf` today."
3. **Want to experiment?** Copy the world instead of changing the shared one:
   ```bash
   cp src/mars_sim/worlds/mars_crater.sdf src/mars_sim/worlds/mars_crater_<yourname>.sdf
   colcon build --symlink-install && source install/setup.bash
   ros2 launch mars_sim sim.launch.py world:=mars_crater_<yourname>.sdf
   ```
4. **Never hand-edit `crater.obj`.** It is generated. Change `generate_crater.py`, run it, and commit **both** files together:
   ```bash
   cd ~/mars_ws/src/mars_sim && python3 scripts/generate_crater.py && cd ~/mars_ws
   ```
5. **Keep names stable.** Other packages depend on the entity names (`primary_rover`, `drone_1`…`drone_4`, `base_station`), the world name `mars_world`, and the topics. If you change one, update section 7 of this README in the same pull request.
6. **Changed positions or the crater shape?** Update the table in section 7 so everyone knows the new numbers.

### If git says "conflict"

This means you and a teammate changed the same lines.
```bash
git pull                       # shows which files conflict
```
1. Open each conflicting file. You'll see blocks like this:
   ```
   <<<<<<< HEAD
   <pose>8 0 3 0 0 0</pose>        ← your version
   =======
   <pose>9 0 3 0 0 0</pose>        ← their version
   >>>>>>> origin/main
   ```
2. Keep the right lines, and delete the `<<<<<<<`, `=======` and `>>>>>>>` marker lines.
3. Rebuild and test, then finish with:
   ```bash
   git add -A && git commit -m "Merge latest main" && git push
   ```
4. Not sure which version is right? Ask the person who made the other change before choosing.

## 9. Interfaces

**Available now (Gazebo topics):**

| Topic | Type | Direction | Meaning |
|---|---|---|---|
| `/rover/cmd_vel` | `gz.msgs.Twist` | in | Rover speed command |
| `/rover/odom` | `gz.msgs.Odometry` | out | Rover position estimate |
| `/world/mars_world/set_pose` | service `gz.msgs.Pose` | in | Teleport a model (e.g. move a drone) |

**Planned ROS 2 topics (simulation side, draft):**

| Topic | Type | From → To | Meaning |
|---|---|---|---|
| `/rover/cmd_vel` | `geometry_msgs/Twist` | rover mesh node → sim | Drive the rover (commands that arrived over the mesh) |
| `/rover/odom` | `nav_msgs/Odometry` | sim → link model, rover mesh node | Rover position |
| `/drone_N/cmd_vel` | `geometry_msgs/Twist` | → sim | Move drone N |
| `/drone_N/odom` | `nav_msgs/Odometry` | sim → link model | Drone N position |

**Network side:** all mesh traffic uses the **Zenoh keys** and **packet format** in [section 3](#3-communication-architecture-zenoh). The GUI only needs Zenoh, not ROS 2.

## 10. Roadmap

- [x] **1. World:** crater terrain, rover, 4 drones, base station
- [ ] **2. ROS bridge:** rover and drone topics in ROS 2; drones can move
- [ ] **3. Shared definitions (`mesh_common`):** packet format + CRC32, unit names, Zenoh keys
- [ ] **4. Link model + radio channel:** line of sight + range → `mesh/links`; pass packets only over working links
- [ ] **5. Mesh protocol v1 (Zenoh):** flooding with TTL + duplicate filter (first rover → base demo)
- [ ] **6. Failure injection:** bit corruption, drops, delay, kill / revive units, scripted failures
- [ ] **7. Mesh protocol v2:** HELLO neighbour discovery, routing table, CRC check, ACK + retransmit, re-routing
- [ ] **8. Base station GUI:** map, live links, packet route animation, telemetry, failure stats, command box
- [ ] **9. Demo:** rover enters crater → link lost → drones deploy → route forms → data reaches base → corruption switched on (CRC + retransmit keep data correct) → a drone fails → re-route

## 11. How to contribute

> Working on the Gazebo world or models? Follow **section 8**. The steps below are for any package.

1. **Get the latest code** before you start:
   ```bash
   cd ~/mars_ws && git pull
   ```
2. **Make a branch** for your task:
   ```bash
   git checkout -b feature/<short-name>       # e.g. feature/protocol-flooding
   ```
3. **Put your code in your own package** under `src/` (see the table in section 2). Create one with:
   ```bash
   cd ~/mars_ws/src && ros2 pkg create --build-type ament_python <package_name>
   ```
4. **Build and test** before committing: `colcon build --symlink-install`.
5. **Commit small, clear changes:**
   ```bash
   git add -A
   git commit -m "Add HELLO beacons to mesh_protocol"
   git push -u origin feature/<short-name>
   ```
6. **Open a Pull Request** on GitHub. Another member reviews it, then it is merged into `main`.

**Rules:**
- Use a branch + Pull Request for code changes (small doc fixes can go straight to `main`).
- Don't commit `build/`, `install/`, `log/` (already in `.gitignore`).
- Keep topic names, Zenoh keys and the packet format exactly as in sections 3 and 9. Change them only after the team agrees, then update this README.
- Import shared names and the packet format from `mesh_common`. Don't copy them into your own package.

## 12. Troubleshooting

| Problem | Fix |
|---|---|
| `curl: (60) SSL certificate problem: self-signed certificate` (on campus Wi-Fi) | The campus firewall intercepts HTTPS. Use a phone hotspot, or install the campus CA certificate (`sudo update-ca-certificates`). |
| `gz: command not found` | Install Gazebo Harmonic (section 4, step 4), then open a new terminal. |
| `error: externally-managed-environment` when installing Zenoh | Add `--break-system-packages` to the `pip install` command (section 4, step 5). |
| `ModuleNotFoundError: No module named 'zenoh'` | Install it: `pip install --break-system-packages eclipse-zenoh`. |
| `zenoh.ZError: ... tcp/[::]:0 ... Address family not supported` | IPv6 is off on your machine. Make Zenoh use IPv4: `cfg = zenoh.Config(); cfg.insert_json5("listen/endpoints", '["tcp/0.0.0.0:0"]'); session = zenoh.open(cfg)`. |
| Zenoh nodes don't see each other | They must run on the same network. Check a firewall isn't blocking UDP multicast, or connect them explicitly to one Zenoh router. |
| ROS 2 nodes stop seeing each other after switching to `rmw_zenoh_cpp` | Start the router `ros2 run rmw_zenoh_cpp rmw_zenohd`, and set `RMW_IMPLEMENTATION=rmw_zenoh_cpp` in **every** terminal. |
| `Unable to find uri[model://...]` / models missing | Rebuild and source: `colcon build --symlink-install && source install/setup.bash`. |
| `ros2: command not found` | `source /opt/ros/jazzy/setup.bash` (add it to `~/.bashrc`). |
| Rover doesn't move | Simulation is paused (press ▶), or check the topic: `gz topic -l \| grep rover`. |
| Rover keeps moving | It keeps the last command: send `linear: {x: 0}`. |
| Simulation slow | Lower `GRID` in `generate_crater.py` (e.g. 101), regenerate, rebuild. |
| Black window / crash | Update graphics drivers: `sudo ubuntu-drivers install`, reboot. |

More fixes are logged in [`ERRORS.md`](ERRORS.md).
