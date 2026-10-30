#!/usr/bin/env bash
# One-command Mars relay-mesh demo for Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic.
#
#   ./scripts/mars_demo.sh start                  # Gazebo + mesh + mission + dashboard on the LAN
#   ./scripts/mars_demo.sh start --tunnel         # also expose the dashboard AND the phone-upload page publicly (Cloudflare)
#   ./scripts/mars_demo.sh start --detach         # leave it running in the background
#   ./scripts/mars_demo.sh stop                   # stop everything this script started
#   ./scripts/mars_demo.sh status                 # what is running, plus the live mesh state
#   ./scripts/mars_demo.sh deps                   # install the extra packages this branch needs
#   ./scripts/mars_demo.sh deps --tunnel          # ...and fetch cloudflared up front
#
# Options for start:
#   --mesh on|off        mesh mode at launch            (default: off = demo starts with direct links)
#   --passcode TEXT      dashboard controls passcode    (default: $MESH_PASSCODE or built in, "" disables)
#   --host ADDR          dashboard bind address         (default: 0.0.0.0 = LAN, 127.0.0.1 = local only)
#   --port N             dashboard port                 (default: 8080)
#   --fake               no Gazebo, fake rover
#   --no-mission         start the mesh without driving
#   --build              force a colcon build first
#   --tunnel             open Cloudflare quick tunnels (dashboard + phone upload page) and print their https URLs.
#                        The phone QR on the dashboard then points at the public upload link (a PHONE_URL you set wins)
#   --no-tunnel          never use a tunnel (default)
#   --qr                 open a separate window with a scannable QR code (default on)
#   --no-qr              do not open the QR window
#   --detach             do not stay in the foreground / do not stop on Ctrl-C
#
# Everything it starts is recorded in $MARS_DEMO_DIR (default /tmp/mars_demo), with one log per
# program, so `stop` never guesses with pkill and never kills your editor.
set -uo pipefail

ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
RUN_DIR=${MARS_DEMO_DIR:-/tmp/mars_demo}
LOG_DIR=$RUN_DIR/logs
CF_DIR=$RUN_DIR/cloudflared
ROS_SETUP=${ROS_SETUP:-/opt/ros/jazzy/setup.bash}

MESH=off
PASSCODE=${MESH_PASSCODE:-kratos123}
HOST=0.0.0.0
PORT=8080
PHONE_PORT=8081     # the phone upload page (same number as PHONE_PORT in mesh_common)
FAKE=
MISSION=1
BUILD=0
TUNNEL=0
QR=1
DETACH=0

c_ok()   { printf '\033[32m%s\033[0m\n' "$*" >&2; }
c_warn() { printf '\033[33m%s\033[0m\n' "$*" >&2; }
c_bad()  { printf '\033[31m%s\033[0m\n' "$*" >&2; }
say()    { printf '\033[36m==>\033[0m %s\n' "$*" >&2; }

usage() { sed -n '2,27p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; }

# ---------------------------------------------------------------- helpers
have() { command -v "$1" >/dev/null 2>&1; }

port_busy() { ss -tln 2>/dev/null | grep -q ":$1 "; }

pids() { cat "$RUN_DIR/$1.pid" 2>/dev/null; }

alive() { local p; p=$(pids "$1"); [ -n "$p" ] && kill -0 "$p" 2>/dev/null; }

# Start one command in its own process group so the whole tree can be stopped at once.
spawn() {                       # spawn NAME LOGFILE CMD...
    local name=$1 log=$2; shift 2
    setsid "$@" >"$log" 2>&1 &
    echo $! >"$RUN_DIR/$name.pid"
}

stop_one() {                    # stop_one NAME
    local name=$1 p
    p=$(pids "$name") || return 0
    if [ -n "$p" ] && kill -0 "$p" 2>/dev/null; then
        if role_ok "$name" "$p"; then
            kill -TERM -- "-$p" 2>/dev/null || kill -TERM "$p" 2>/dev/null
            local i
            for i in $(seq 1 30); do kill -0 "$p" 2>/dev/null || break; sleep 0.1; done
            kill -0 "$p" 2>/dev/null && kill -KILL -- "-$p" 2>/dev/null
        else
            c_warn "ignoring stale $name.pid ($p is not ours)"
        fi
    fi
    rm -f "$RUN_DIR/$name.pid"
}

# A pidfile can go stale and its PID get reused by something else (including your shell).
# Only signal a process group whose command line still looks like the thing we started.
role_ok() {                     # role_ok NAME PID
    local name=$1 p=$2 cmd
    cmd=$(tr '\0' ' ' <"/proc/$p/cmdline" 2>/dev/null) || return 1
    case $name in
        launch)    [[ $cmd == *mesh_protocol* || $cmd == *"ros2 launch"* ]] ;;
        dashboard) [[ $cmd == *dashboard* ]] ;;
        tunnel|phone_tunnel) [[ $cmd == *cloudflared* ]] ;;
        qr)        [[ $cmd == *show_qr.sh* ]] ;;
        mission)   [[ $cmd == *mesh_protocol*mission* ]] ;;
        *)         return 1 ;;
    esac
}

wait_http() {                   # wait_http URL SECONDS
    local url=$1 limit=$2 i=0
    while [ "$i" -lt "$limit" ]; do
        if curl -fsS -m 2 -o /dev/null "$url" 2>/dev/null; then return 0; fi
        sleep 1; i=$((i + 1))
    done
    return 1
}

lan_ips() {
    # IPv4 only: an IPv6 URL would need brackets and is rarely what people type.
    { ip -4 -o addr show scope global 2>/dev/null | awk '{print $4}' | cut -d/ -f1
      hostname -I 2>/dev/null | tr ' ' '\n' | grep -E '^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$'
    } | grep -v '^10\.0\.2\.' | grep -v '^127\.' | grep -v '^$' | sort -u
}

ros_source() {
    set +u                        # ROS setup files reference unbound vars; they must not abort us
    if [ -f "$ROS_SETUP" ]; then . "$ROS_SETUP"; else set -u; c_bad "ROS 2 not found at $ROS_SETUP"; return 1; fi
    if [ -f "$ROOT/install/setup.bash" ]; then . "$ROOT/install/setup.bash"
    else set -u; c_bad "workspace not built: $ROOT/install/setup.bash missing (run with --build)"; return 1; fi
    set -u
}

# ---------------------------------------------------------------- cloudflared
ensure_cloudflared() {
    local bin=$CF_DIR/bin/cloudflared arch deb_arch idx fn
    if [ -x "$bin" ]; then echo "$bin"; return 0; fi
    mkdir -p "$CF_DIR"
    case $(uname -m) in
        aarch64|arm64) deb_arch=arm64 ;;
        x86_64|amd64)  deb_arch=amd64 ;;
        *) c_bad "unsupported architecture $(uname -m)"; return 1 ;;
    esac
    say "fetching cloudflared ($deb_arch, one time, ~18 MB)"

    # Cloudflare's apt package is much smaller than the GitHub release asset and resumes cleanly.
    idx=$(curl -fsSL -m 30 "https://pkg.cloudflare.com/cloudflared/dists/any/main/binary-$deb_arch/Packages" 2>/dev/null)
    fn=$(printf '%s\n' "$idx" | awk '/^Filename:/{print $2; exit}')
    if [ -n "$fn" ] && curl -fsSL -C - --retry 5 --retry-delay 2 --connect-timeout 10 \
            -m 600 -o "$CF_DIR/cloudflared.deb" "https://pkg.cloudflare.com/cloudflared/$fn" 2>/dev/null; then
        if have dpkg-deb && dpkg-deb -x "$CF_DIR/cloudflared.deb" "$CF_DIR/deb" 2>/dev/null; then
            mkdir -p "$CF_DIR/bin"
            mv "$CF_DIR/deb/usr/bin/cloudflared" "$bin" && chmod +x "$bin" && echo "$bin" && return 0
        fi
    fi

    # Fallback: static release binary, resumed a few times (the CDN sometimes drops mid-file).
    say "package download failed, trying the release binary"
    for _ in 1 2 3 4 5 6 7 8; do
        curl -sfL -C - --connect-timeout 10 -m 120 -o "$CF_DIR/bin/cloudflared" \
            "https://github.com/cloudflare/cloudflared/releases/latest/download/cloudflared-linux-$deb_arch" 2>/dev/null
        chmod +x "$CF_DIR/bin/cloudflared" 2>/dev/null
        "$CF_DIR/bin/cloudflared" --version >/dev/null 2>&1 && { echo "$CF_DIR/bin/cloudflared"; return 0; }
    done
    c_bad "could not download cloudflared"
    return 1
}

start_tunnel() {                # start_tunnel NAME PORT -> prints the public https URL
    local name=$1 port=$2 bin url
    bin=$(ensure_cloudflared) || return 1
    spawn "$name" "$LOG_DIR/$name.log" "$bin" tunnel --url "http://127.0.0.1:$port" --no-autoupdate
    for _ in $(seq 1 30); do
        url=$(grep -oE 'https://[a-z0-9-]+\.trycloudflare\.com' "$LOG_DIR/$name.log" 2>/dev/null | head -1)
        [ -n "$url" ] && { echo "$url"; return 0; }
        sleep 1
    done
    c_bad "tunnel started but no URL appeared: see $LOG_DIR/$name.log"
    return 1
}

# ---------------------------------------------------------------- QR code
# QRENCODE is a command array: either the system binary or a copy unpacked from Ubuntu's .deb,
# which needs no sudo (apt-get download + dpkg-deb -x).
QRENCODE=()

ensure_qrencode() {
    [ ${#QRENCODE[@]} -gt 0 ] && return 0
    if have qrencode; then QRENCODE=(qrencode); return 0; fi

    local dir=$RUN_DIR/qrencode bin lib
    bin=$dir/root/usr/bin/qrencode
    if [ ! -x "$bin" ]; then
        say "fetching qrencode (~50 kB, no sudo)"
        mkdir -p "$dir"
        ( cd "$dir" && apt-get download qrencode libqrencode4 >/dev/null 2>&1 ) || true
        local deb
        for deb in "$dir"/*.deb; do [ -f "$deb" ] && dpkg-deb -x "$deb" "$dir/root" 2>/dev/null; done
    fi
    [ -x "$bin" ] || return 1
    lib=$(find "$dir/root/usr/lib" -name 'libqrencode.so.4*' -printf '%h\n' 2>/dev/null | head -1)
    if [ -n "$lib" ]; then QRENCODE=(env "LD_LIBRARY_PATH=$lib" "$bin"); else QRENCODE=("$bin"); fi
}

show_qr() {                     # show_qr URL [TITLE]
    local url=$1 title=${2:-Mars relay mesh}
    ensure_qrencode || {
        c_warn "qrencode unavailable (sudo apt install qrencode) — link not shown as a QR"
        c_ok "$url"
        return 0
    }
    mkdir -p "$RUN_DIR"
    "${QRENCODE[@]}" -o "$RUN_DIR/qr.png" -s 8 -m 2 "$url" 2>/dev/null \
        && say "QR image: $RUN_DIR/qr.png"
    { printf '%s\n' "$url"; [ -n "$PASSCODE" ] && printf 'passcode: %s\n' "$PASSCODE"; } > "$RUN_DIR/link.txt"

    # A helper script keeps the quoting sane and holds the window open.
    cat > "$RUN_DIR/show_qr.sh" <<EOF
#!/usr/bin/env bash
echo \$\$ > "$RUN_DIR/qr.pid"
clear
${QRENCODE[*]} -t ANSIUTF8 -m 2 "$url"
echo
echo "  $url"
$([ -n "$PASSCODE" ] && echo "echo \"  passcode: $PASSCODE\"")
echo
echo "  Scan with a phone camera. Close this window when you are done."
while :; do sleep 3600; done
EOF
    chmod +x "$RUN_DIR/show_qr.sh"

    if [ -n "${WAYLAND_DISPLAY:-}${DISPLAY:-}" ]; then
        if have gnome-terminal; then
            gnome-terminal --title="$title — scan to watch" -- "$RUN_DIR/show_qr.sh" >/dev/null 2>&1 && return 0
        fi
        have x-terminal-emulator && x-terminal-emulator -T "$title" -e "$RUN_DIR/show_qr.sh" >/dev/null 2>&1 && return 0
        have xterm && xterm -T "$title" -hold -e "$RUN_DIR/show_qr.sh" >/dev/null 2>&1 && return 0
        have eog && eog "$RUN_DIR/qr.png" >/dev/null 2>&1 && return 0
        have xdg-open && xdg-open "$RUN_DIR/qr.png" >/dev/null 2>&1 && return 0
    fi
    # No display: draw it in this terminal and keep going.
    "${QRENCODE[@]}" -t ANSIUTF8 -m 2 "$url"
    c_ok "$url"
}

# ---------------------------------------------------------------- dependencies
# The extra packages this branch needs live next to this script, so `deps` on a fresh machine is
# one command and nothing here has to guess at versions.
REQ_FILE=$ROOT/scripts/requirements.txt
APT_FILE=$ROOT/scripts/apt-packages.txt

pip_install_reqs() {
    # Ubuntu 24.04 marks the system python as externally managed; --user alone is refused.
    local flags=(--user)
    python3 -m pip install --help 2>/dev/null | grep -q -- '--break-system-packages' \
        && flags+=(--break-system-packages)
    python3 -m pip install "${flags[@]}" -r "$REQ_FILE"
}

zenoh_version() {
    python3 -c "import importlib.metadata as m; print(m.version('eclipse-zenoh'))" 2>/dev/null
}

# The abandoned `zenoh` package on PyPI installs a module called `zenoh` too, with the old 0.x API
# (sample.payload is bytes, there is no .to_bytes()). That fails on the first packet, so say it here.
warn_legacy_zenoh() {
    local legacy
    legacy=$(python3 -c "import importlib.metadata as m; print(m.version('zenoh'))" 2>/dev/null)
    [ -n "$legacy" ] || return 0
    c_warn "the abandoned 'zenoh' $legacy package is installed as well and shadows eclipse-zenoh"
    c_warn "fix: python3 -m pip uninstall -y zenoh"
}

dep_row() {                     # dep_row LABEL CMD...
    local label=$1 v; shift
    if v=$("$@" 2>/dev/null | head -1) && [ -n "$v" ]; then
        printf '  %-14s %s\n' "$label" "$v" >&2
    else
        printf '  %-14s %s\n' "$label" "MISSING" >&2
    fi
}

cmd_deps() {
    local pkgs=()
    say "workspace $ROOT"
    [ -f "$REQ_FILE" ] || { c_bad "missing $REQ_FILE"; return 1; }

    if ! python3 -m pip --version >/dev/null 2>&1; then
        c_bad "pip is missing — sudo apt install python3-pip"; return 1
    fi
    say "python packages from scripts/requirements.txt"
    pip_install_reqs || { c_bad "pip install failed (see above)"; return 1; }
    warn_legacy_zenoh

    if [ -f "$APT_FILE" ]; then
        mapfile -t pkgs < <(grep -vE '^[[:space:]]*(#|$)' "$APT_FILE")
    fi
    if [ ${#pkgs[@]} -gt 0 ]; then
        if have apt-get && sudo -n true 2>/dev/null; then
            say "system packages: ${pkgs[*]}"
            sudo -n apt-get install -y "${pkgs[@]}" >/dev/null 2>&1 \
                && c_ok "system packages installed" \
                || c_warn "apt install failed — not fatal, the script fetches qrencode itself"
        else
            c_warn "no passwordless sudo — skipping ${pkgs[*]}"
            c_warn "  optional: the QR window downloads qrencode without root"
        fi
    fi

    if [ "$TUNNEL" = 1 ]; then
        say "cloudflared (needed for --tunnel)"
        ensure_cloudflared >/dev/null && c_ok "cloudflared ready" || c_warn "cloudflared unavailable"
    fi
    ensure_qrencode >/dev/null || c_warn "qrencode unavailable — the QR window will print the link"

    echo
    echo "installed:" >&2
    dep_row zenoh python3 -c 'import importlib.metadata as m; print(m.version("eclipse-zenoh"))'
    dep_row numpy python3 -c 'import numpy; print(numpy.__version__)'
    dep_row "gz bindings" python3 -c 'import gz.transport13; print("ok")'
    dep_row gazebo gz sim --versions
    [ ${#QRENCODE[@]} -gt 0 ] && dep_row qrencode "${QRENCODE[@]}" --version
    [ -x "$CF_DIR/bin/cloudflared" ] && dep_row cloudflared "$CF_DIR/bin/cloudflared" --version
    dep_row pytest python3 -m pytest --version
    dep_row colcon bash -c 'command -v colcon'
    echo

    # A 0.x zenoh imports fine but breaks on the first message, so check the version, not just the import.
    local zver major
    zver=$(zenoh_version)
    if [ -z "$zver" ]; then
        c_bad "python3 cannot import eclipse-zenoh — pip install --user --break-system-packages eclipse-zenoh"
        return 1
    fi
    major=${zver%%.*}
    [ "$major" -lt 1 ] 2>/dev/null && { c_bad "zenoh $zver is too old (need 1.10.x)"; return 1; }
    c_ok "zenoh $zver ready — ./scripts/mars_demo.sh start --tunnel"
}

# ---------------------------------------------------------------- commands
cmd_start() {
    mkdir -p "$LOG_DIR" "$RUN_DIR"
    local p
    if { p=$(pids launch); [ -n "$p" ] && role_ok launch "$p"; } \
       || { p=$(pids dashboard); [ -n "$p" ] && role_ok dashboard "$p"; }; then
        c_bad "already running — use '$0 stop' first (or '$0 status')"; return 1
    fi
    rm -f "$RUN_DIR"/*.pid 2>/dev/null   # stale leftovers must never steer a later stop

    say "workspace $ROOT"
    ros_source || return 1

    if [ "$BUILD" = 1 ] || [ ! -f "$ROOT/install/setup.bash" ] || [ -n "$(find "$ROOT/src" -newer "$ROOT/install/setup.bash" -name '*.py' -print -quit 2>/dev/null)" ]; then
        say "colcon build --symlink-install"
        ( cd "$ROOT" && colcon build --symlink-install ) || { c_bad "build failed"; return 1; }
        set +u                # install/setup.bash trips over unbound vars under `set -u`
        . "$ROOT/install/setup.bash"
        set -u
    fi

    # Preflight
    have curl || { c_bad "curl is missing"; return 1; }
    have colcon || { c_bad "colcon is missing — sudo apt install ros-dev-tools"; return 1; }
    python3 -c 'import zenoh' 2>/dev/null || { c_bad "python zenoh missing: pip install --user --break-system-packages eclipse-zenoh"; return 1; }
    if [ -z "$FAKE" ]; then
        have gz || { c_bad "gz not found — install Gazebo Harmonic (see README step 4)"; return 1; }
        python3 -c 'import gz.transport13' 2>/dev/null || { c_bad "python gz bindings missing: sudo apt install python3-gz-transport13 python3-gz-msgs10"; return 1; }
    fi
    port_busy 7447 && { c_bad "port 7447 busy — an old mesh is still running ('$0 stop')"; return 1; }
    port_busy "$PORT" && { c_bad "port $PORT busy ('$0 stop', or --port)"; return 1; }
    port_busy "$PHONE_PORT" && { c_bad "port $PHONE_PORT (phone upload page) busy ('$0 stop')"; return 1; }

    # With --tunnel the phone upload page gets its own public link too, so the phone QR on the dashboard
    # works for phones on any network. It has to exist BEFORE the dashboard starts, because the dashboard
    # reads PHONE_URL at start-up. A PHONE_URL you set yourself wins.
    if [ "$TUNNEL" = 1 ] && [ -z "${PHONE_URL:-}" ]; then
        say "opening Cloudflare quick tunnel for the phone upload page"
        local phone_pub
        if phone_pub=$(start_tunnel phone_tunnel "$PHONE_PORT"); then
            export PHONE_URL="$phone_pub/phone"
            c_ok "phone page: $PHONE_URL"
        else
            c_warn "phone tunnel failed; the phone QR will use the LAN address"
        fi
    fi

    say "starting dashboard on $HOST:$PORT"
    local dash_args=(ros2 run mesh_dashboard dashboard --host "$HOST" --port "$PORT")
    [ -n "$PASSCODE" ] && dash_args+=(--passcode "$PASSCODE")
    spawn dashboard "$LOG_DIR/dashboard.log" "${dash_args[@]}"

    say "starting sim + mesh (mesh=$MESH)"
    local launch_args=(ros2 launch mesh_protocol mesh.launch.py "mesh:=$MESH")
    [ -n "$FAKE" ] && launch_args+=(fake:=true) || launch_args+=(sim:=true)
    spawn launch "$LOG_DIR/launch.log" "${launch_args[@]}"

    if ! wait_http "http://127.0.0.1:$PORT/" 25; then
        c_bad "dashboard did not answer — see $LOG_DIR/dashboard.log"
    fi

    echo
    c_ok "dashboard: http://127.0.0.1:$PORT/"
    local ip lan_first=
    while read -r ip; do
        [ -n "$ip" ] || continue
        [ -z "$lan_first" ] && lan_first=$ip
        c_ok "         : http://$ip:$PORT/"
    done < <(lan_ips)
    if [ "$HOST" = 0.0.0.0 ] && [ -z "$lan_first" ]; then
        c_warn "only loopback addresses found — this VM looks NAT'd (10.0.2.x / 192.168.64.x); use --tunnel"
    fi

    local public=
    if [ "$TUNNEL" = 1 ]; then
        say "opening Cloudflare quick tunnel"
        if public=$(start_tunnel tunnel "$PORT"); then c_ok "public    : $public"; else c_warn "tunnel failed; LAN only"; fi
    fi

    if [ "$QR" = 1 ]; then
        if [ -n "$public" ]; then
            say "QR window (tunnel link)"
            show_qr "$public" "Mars relay mesh — public"
        elif [ -n "$lan_first" ]; then
            say "QR window (LAN link)"
            show_qr "http://$lan_first:$PORT/" "Mars relay mesh — LAN"
        else
            c_warn "no reachable URL to encode (NAT'd VM) — run with --tunnel"
        fi
        # The helper writes qr.pid as its first line: wait for it so the window is really up
        # before the rover moves (10 s cap, then carry on).
        for _ in $(seq 1 20); do [ -s "$RUN_DIR/qr.pid" ] && break; sleep 0.5; done
    fi

    # The mission starts only now: the QR window is up, so whoever scans it can watch from the
    # first metre. It is our own process group, not the launch file's, so stop/status see it.
    if [ "$MISSION" = 1 ]; then
        say "starting mission (base -> target)"
        local margs=(ros2 run mesh_protocol mission)
        [ -n "$FAKE" ] && margs+=(--fake)
        spawn mission "$LOG_DIR/mission.log" "${margs[@]}"
    fi

    [ -n "$PASSCODE" ] && c_warn "controls passcode: $PASSCODE" || c_warn "controls are UNLOCKED (empty passcode)"
    echo
    echo "logs : $LOG_DIR"
    echo "stop : $0 stop"
    echo

    if [ "$DETACH" = 1 ]; then
        c_ok "running in the background"
        return 0
    fi

    trap 'echo; say "stopping everything"; cmd_stop; exit 0' INT TERM
    say "mission starts ~8 s after launch; Ctrl-C stops everything"
    tail -n +1 -F "$LOG_DIR/launch.log" 2>/dev/null | sed -u 's/^/[sim] /' &
    local tail_pid=$!
    while alive launch; do sleep 1; done
    kill "$tail_pid" 2>/dev/null
    say "the sim process ended — stopping the rest"
    cmd_stop
}

cmd_stop() {
    local name
    for name in launch dashboard tunnel phone_tunnel qr mission; do stop_one "$name"; done
    rm -f "$RUN_DIR"/*.pid
    if port_busy 7447 || port_busy "$PORT"; then
        c_warn "some ports are still busy: $(ss -tln 2>/dev/null | grep -E ":(7447|$PORT) " | awk '{print $4}' | tr '\n' ' ')"
    else
        c_ok "everything stopped (7447 and $PORT free)"
    fi
}

cmd_status() {
    local name p
    for name in launch dashboard tunnel phone_tunnel qr mission; do
        p=$(pids "$name")
        if [ -n "$p" ] && kill -0 "$p" 2>/dev/null; then printf '%-10s running (pid %s)\n' "$name" "$p"
        else printf '%-10s not running\n' "$name"; fi
    done
    echo
    if [ -z "$(pids launch)" ]; then
        c_warn "nothing to query — start it with '$0 start'"
        return 0
    fi
    # SSE never ends, so take the first data line and stop; `-m` + `pipefail` would look like an error.
    local snap
    snap=$(curl -sS -N -m 5 "http://127.0.0.1:$PORT/events" 2>/dev/null \
           | awk '/^data: /{sub(/^data: /, ""); print; exit}')
    [ -n "$snap" ] || { c_warn "no snapshot yet (is the mesh still starting?)"; return 0; }
    printf '%s' "$snap" | python3 -c '
import json, sys
d = json.loads(sys.stdin.read())
co, m, o = d.get("coord") or {}, d.get("mission") or {}, d.get("last_odom") or {}
print("mesh      :", co.get("mode"), co.get("state"), "| leader", co.get("leader"),
      "| chain", co.get("chains", {}).get("base"))
print("mission   :", m.get("state"), "x =", m.get("x"), "m,", m.get("progress"), "%")
print("odometry  :", ("%.2f s old" % d["odom_age"]) if d.get("odom_age") is not None else "none",
      "| route", o.get("route"), "| delay", o.get("delay_ms"), "ms")
print("delivery  :", d.get("delivery"))
'
}

# ---------------------------------------------------------------- main
[ $# -eq 0 ] && { usage; exit 1; }
cmd=$1; shift
while [ $# -gt 0 ]; do
    case $1 in
        --mesh)      MESH=${2:-on}; shift 2 ;;
        --passcode)  PASSCODE=${2:-}; shift 2 ;;
        --host)      HOST=${2:-0.0.0.0}; shift 2 ;;
        --port)      PORT=${2:-8080}; shift 2 ;;
        --fake)      FAKE=1; shift ;;
        --no-mission) MISSION=0; shift ;;
        --build)     BUILD=1; shift ;;
        --tunnel)    TUNNEL=1; shift ;;
        --no-tunnel) TUNNEL=0; shift ;;
        --qr)        QR=1; shift ;;
        --no-qr)     QR=0; shift ;;
        --detach)    DETACH=1; shift ;;
        -h|--help)   usage; exit 0 ;;
        *) c_bad "unknown option: $1"; usage; exit 1 ;;
    esac
done

case $cmd in
    start)  cmd_start ;;
    stop)   cmd_stop ;;
    status) cmd_status ;;
    deps)   cmd_deps ;;
    restart) cmd_stop; cmd_start ;;
    -h|--help|help) usage; exit 0 ;;
    *) usage; exit 1 ;;
esac
