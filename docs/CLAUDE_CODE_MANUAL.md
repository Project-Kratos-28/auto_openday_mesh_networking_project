# Claude Code: Simple Manual
For the Mars Rover Relay Simulation project (Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic).

---

## 1. What is Claude Code?
A helper program that runs **in your terminal, inside your project folder**. You type what you want in normal English. It can:
- read your files,
- create and edit files,
- run terminal commands (build, test, git),
and it **asks your permission** before doing risky things.

It is NOT the chat website. It works directly on the files on your laptop, so you don't copy-paste code around.

---

## 2. Daily start (3 steps)
```bash
cd ~/mars_ws        # 1. go to the project folder (Claude Code reads files from where you start it)
claude              # 2. start it
```
3. **First time only:** log in (a browser window opens) and choose **"Yes, I trust this folder"**.
   (If you skip trust, the safe settings in `.claude/settings.json` won't apply.)

To leave: type `/exit` (or press `Ctrl+D` twice).

---

## 3. How to talk to it
Talk like you would to a helpful teammate. Be **specific** and ask for **small steps**.

| Weak | Strong |
|---|---|
| "make the simulation" | "Add a crater to `worlds/empty_mars.sdf` using a heightmap. Keep it simple and explain each line." |
| "it's broken" | "I ran `colcon build` and got this error: (paste full error). Find the cause and fix it." |
| "add drones" | "First read CLAUDE.md. Then make a simple box-shaped drone model in `models/relay_drone/`, nothing else." |

Useful tricks:
- **Paste the full error text**, don't summarise it.
- Type `@` and a file name to point at a file (example: `@src/mars_sim/launch/sim.launch.py`).
- Say "explain like I'm a beginner" any time you don't understand something.
- Say "ask me questions first if anything is unclear" for big tasks.
- Always **test** what it builds. It can be wrong.

---

## 4. Permission prompts (the most important habit)
Before running a command or editing a file, Claude Code may show a prompt:
- **Yes**: do it once.
- **Yes, and don't ask again**: remember this kind of command for the future.
- **No**: don't do it (you can type a reason with `Tab`).

**Always read the command before pressing Yes.** Be extra careful with anything containing `sudo`, `rm`, `curl ... | bash`, or `git push`.

What our settings file (`.claude/settings.json`) does:
- **Auto-accepts file edits** inside the workspace (mode `acceptEdits`).
- **Pre-approves safe commands**: `colcon build`, `ros2 launch/topic/node/pkg`, `gz topic`, `git status/diff/log/add/commit`.
- **Blocks**: `sudo`, `rm -rf`, `git push`.

Note: the blocking is a safety net, not a perfect wall (the same program can be written in other ways). So still read prompts.
Change rules any time with `/permissions`.

---

## 5. Modes (press `Shift+Tab` to switch)
- **Normal / Manual**: asks before edits and commands.
- **Accept edits**: edits files without asking, still asks for other commands (our default).
- **Plan**: it only reads and thinks, and does NOT change your files. Use it before big steps: "plan how to add a crater and drones" then review, then switch back and say "go ahead".

Do **not** use "bypass permissions" on your own laptop. It is meant only for isolated environments like containers or VMs.

---

## 6. Cheat sheet

**Typed in the normal terminal**
| Command | What it does |
|---|---|
| `claude` | start Claude Code in this folder |
| `claude "task"` | start with a first instruction |
| `claude -p "question"` | answer once and exit (no chat) |
| `claude -c` | continue the most recent conversation in this folder |
| `claude -r` | pick an older conversation to resume |
| `claude --version` | check it's installed |

**Typed inside Claude Code**
| Command | What it does |
|---|---|
| `/help` | list commands |
| `/clear` | wipe the conversation (start fresh; the project files remain) |
| `/resume` | continue an earlier conversation |
| `/permissions` | see / edit allow and deny rules |
| `/login` | log in again or switch account |
| `/exit` | quit |

**Keys**: `Shift+Tab` change mode, `Esc` stop / cancel, `Tab` autocomplete, `Up arrow` previous message.

---

## 7. Files in this workspace that matter
| File | Why |
|---|---|
| `CLAUDE.md` | Claude Code reads it at the start of every session: our setup, rules, progress. **Keep it updated.** |
| `.claude/settings.json` | permissions (what is allowed / blocked) |
| `ERRORS.md` | log of problems and fixes, to upload to git |
| `src/mars_sim/` | our ROS package: `worlds/`, `models/`, `launch/`, `config/` |
| `docs/CLAUDE_CODE_MANUAL.md` | this file |

Tip: when you and your team agree on topic names or message types, tell Claude Code: "Add this to CLAUDE.md under Interfaces."

---

## 8. Good working loop (repeat for every feature)
1. **Plan** (Plan mode or "explain your plan first").
2. **Small change**: one file or one feature at a time.
3. **Build**: "build and run the headless Gazebo check" (it can run `colcon build`).
4. **Look**: you run `ros2 launch mars_sim sim.launch.py` in your own terminal to see the window.
5. **Fix**: paste any error back.
6. **Log**: "Add this error and fix to ERRORS.md" (CLAUDE.md already tells it to).
7. **Commit**: "commit my changes with a clear message".
8. When the conversation gets long or confused: `/clear`, then "Read CLAUDE.md and continue with ...".

---

## 9. Letting it work while you do other things
- Give a **clear, finished task**: "Create the crater world, build, run the headless check, fix errors, and tell me when done."
- Keep it inside the safe settings above. It will only stop and wait if it needs a permission you haven't pre-approved.
- One-off job without chat: `claude -p "run colcon build and summarise any warnings"`.
- It **cannot** type your `sudo` password. Installs needing sudo are done by you (like `install_gazebo.sh`).
- It **cannot** judge how the Gazebo window "looks" by itself. You check visually.

---

## 10. Problems and fixes
| Problem | Fix |
|---|---|
| `claude: command not found` | Open a **new** terminal. If still missing: `export PATH="$HOME/.local/bin:$PATH"` (add that line to `~/.bashrc`). |
| `ros2: command not found` inside Claude Code | Each command runs in a fresh shell. Start commands with `source /opt/ros/jazzy/setup.bash && source install/setup.bash && ...` (written in CLAUDE.md). |
| Claude Code hangs on `gz sim` / `ros2 launch` | They never end. Use `timeout 20 gz sim -s -r <world.sdf>` for checks, or run them yourself. Press `Esc` to stop. |
| Too many permission prompts | Choose "Yes, and don't ask again", or add rules with `/permissions`. |
| Safe settings not applied | You didn't trust the folder. Quit, run `claude` again in the workspace, accept the trust question. |
| It uses old Gazebo commands (`gazebo`, `gazebo_ros`) | Remind it: "We use Gazebo Harmonic and `gz sim`. Re-read CLAUDE.md." |
| It forgets things | Put important facts in `CLAUDE.md`. |
| Login problems | `/login`. You need a Claude subscription or a Console account. |

---

## 11. Copy-paste starter prompts for this project
1. *Check my setup:* "Read CLAUDE.md. Check that ROS Jazzy and Gazebo Harmonic work: run `gz sim --version`, build the workspace, and run the headless check. Report problems simply."
2. *Terrain:* "Plan, then add a crater and two mountains to the Mars world so they can block line-of-sight. Keep it simple, use basic shapes first. Explain each new line."
3. *Rover:* "Create a simple 4-wheel rover model in `models/primary_rover/` that I can drive with `ros2 topic pub`. Add the ROS-Gazebo bridge for `cmd_vel` and odometry."
4. *Drones:* "Add 3 simple relay drone models and a launch argument to spawn them at chosen positions."
5. *Wrap-up:* "Summarise every error we fixed today into ERRORS.md, then commit."
