#!/usr/bin/env bash
# One-command Mars relay-mesh demo for Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic.
#
#   ./scripts/mars_demo.sh start                  # Gazebo + mesh + mission + dashboard on the LAN
#   ./scripts/mars_demo.sh start --tunnel         # also expose it publicly through Cloudflare
#   ./scripts/mars_demo.sh start --detach         # leave it running in the background
#   ./scripts/mars_demo.sh stop                   # stop everything this script started
#   ./scripts/mars_demo.sh status                 # what is running, plus the live mesh state
#
# Options for start:
#   --mesh on|off        mesh mode at launch            (default: on)
#   --passcode TEXT      dashboard controls passcode    (default: kratos123, "" disables)
#   --host ADDR          dashboard bind address         (default: 0.0.0.0 = LAN, 127.0.0.1 = local only)
#   --port N             dashboard port                 (default: 8080)
#   --fake               no Gazebo, fake rover
#   --no-mission         start the mesh without driving
#   --build              force a colcon build first
#   --tunnel             open a Cloudflare quick tunnel and print its https URL
#   --no-tunnel          never use a tunnel (default)
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

MESH=on
PASSCODE=kratos123
HOST=0.0.0.0
PORT=8080
FAKE=
MISSION=1
BUILD=0
TUNNEL=0
DETACH=0

c_ok()   { printf '\033[32m%s\033[0m\n' "$*" >&2; }
c_warn() { printf '\033[33m%s\033[0m\n' "$*" >&2; }
c_bad()  { printf '\033[31m%s\033[0m\n' "$*" >&2; }
say()    { printf '\033[36m==>\033[0m %s\n' "$*" >&2; }

usage() { sed -n '2,30p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; }

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
        kill -TERM -- "-$p" 2>/dev/null || kill -TERM "$p" 2>/dev/null
    fi
    rm -f "$RUN_DIR/$name.pid"
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

start_tunnel() {
    local bin url
    bin=$(ensure_cloudflared) || return 1
    spawn tunnel "$LOG_DIR/tunnel.log" "$bin" tunnel --url "http://127.0.0.1:$PORT" --no-autoupdate
    for _ in $(seq 1 30); do
        url=$(grep -oE 'https://[a-z0-9-]+\.trycloudflare\.com' "$LOG_DIR/tunnel.log" 2>/dev/null | head -1)
        [ -n "$url" ] && { echo "$url"; return 0; }
        sleep 1
    done
    c_bad "tunnel started but no URL appeared: see $LOG_DIR/tunnel.log"
    return 1
}

# ---------------------------------------------------------------- commands
cmd_start() {
    mkdir -p "$LOG_DIR" "$RUN_DIR"
    if alive launch || alive dashboard; then
        c_bad "already running — use '$0 stop' first (or '$0 status')"; return 1
    fi

    say "workspace $ROOT"
    ros_source || return 1

    if [ "$BUILD" = 1 ] || [ ! -f "$ROOT/install/setup.bash" ] || [ -n "$(find "$ROOT/src" -newer "$ROOT/install/setup.bash" -name '*.py' -print -quit 2>/dev/null)" ]; then
        say "colcon build --symlink-install"
        ( cd "$ROOT" && colcon build --symlink-install ) || { c_bad "build failed"; return 1; }
        . "$ROOT/install/setup.bash"
    fi

    # Preflight
    have curl || { c_bad "curl is missing"; return 1; }
    python3 -c 'import zenoh' 2>/dev/null || { c_bad "python zenoh missing: pip install --user --break-system-packages eclipse-zenoh"; return 1; }
    if [ -z "$FAKE" ]; then
        have gz || { c_bad "gz not found — install Gazebo Harmonic (see README step 4)"; return 1; }
        python3 -c 'import gz.transport13' 2>/dev/null || { c_bad "python gz bindings missing: sudo apt install python3-gz-transport13 python3-gz-msgs10"; return 1; }
    fi
    port_busy 7447 && { c_bad "port 7447 busy — an old mesh is still running ('$0 stop')"; return 1; }
    port_busy "$PORT" && { c_bad "port $PORT busy ('$0 stop', or --port)"; return 1; }

    say "starting dashboard on $HOST:$PORT"
    local dash_args=(ros2 run mesh_dashboard dashboard --host "$HOST" --port "$PORT")
    [ -n "$PASSCODE" ] && dash_args+=(--passcode "$PASSCODE")
    spawn dashboard "$LOG_DIR/dashboard.log" "${dash_args[@]}"

    say "starting sim + mesh (mesh=$MESH)"
    local launch_args=(ros2 launch mesh_protocol mesh.launch.py "mesh:=$MESH")
    [ -n "$FAKE" ] && launch_args+=(fake:=true) || launch_args+=(sim:=true)
    [ "$MISSION" = 1 ] && launch_args+=(mission:=true)
    spawn launch "$LOG_DIR/launch.log" "${launch_args[@]}"

    if ! wait_http "http://127.0.0.1:$PORT/" 25; then
        c_bad "dashboard did not answer — see $LOG_DIR/dashboard.log"
    fi

    echo
    c_ok "dashboard: http://127.0.0.1:$PORT/"
    local ip
    while read -r ip; do
        [ -n "$ip" ] && c_ok "         : http://$ip:$PORT/"
    done < <(lan_ips)
    if [ "$HOST" = 0.0.0.0 ] && ! lan_ips | grep -q .; then
        c_warn "only loopback addresses found — this VM looks NAT'd (10.0.2.x / 192.168.64.x); use --tunnel"
    fi

    if [ "$TUNNEL" = 1 ]; then
        say "opening Cloudflare quick tunnel"
        local url
        if url=$(start_tunnel); then c_ok "public    : $url"; else c_warn "tunnel failed; LAN only"; fi
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
    for name in launch dashboard tunnel; do stop_one "$name"; done
    sleep 3
    for name in launch dashboard tunnel; do
        local p; p=$(pids "$name")
        [ -n "$p" ] && kill -KILL -- "-$p" 2>/dev/null
    done
    rm -f "$RUN_DIR"/*.pid
    if port_busy 7447 || port_busy "$PORT"; then
        c_warn "some ports are still busy: $(ss -tln 2>/dev/null | grep -E ":(7447|$PORT) " | awk '{print $4}' | tr '\n' ' ')"
    else
        c_ok "everything stopped (7447 and $PORT free)"
    fi
}

cmd_status() {
    local name p
    for name in launch dashboard tunnel; do
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
        --detach)    DETACH=1; shift ;;
        -h|--help)   usage; exit 0 ;;
        *) c_bad "unknown option: $1"; usage; exit 1 ;;
    esac
done

case $cmd in
    start)  cmd_start ;;
    stop)   cmd_stop ;;
    status) cmd_status ;;
    restart) cmd_stop; cmd_start ;;
    *) usage; exit 1 ;;
esac
