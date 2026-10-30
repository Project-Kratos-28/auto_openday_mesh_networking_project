#!/bin/bash
# Public https tunnel to the phone-upload page, then (re)start the demo with the QR pointing at it.
CF=/tmp/mars_demo/cloudflared/bin/cloudflared
"$(dirname "$0")/run_demo.sh" stop >/dev/null 2>&1
pkill -f "[c]loudflared tunnel --url http://127.0.0.1:8081"
sleep 2
setsid nohup $CF tunnel --url http://127.0.0.1:8081 --no-autoupdate > ~/demo_logs/phone_tunnel.log 2>&1 < /dev/null &
for i in $(seq 30); do
  U=$(grep -oE 'https://[a-z0-9-]+\.trycloudflare\.com' ~/demo_logs/phone_tunnel.log | head -1)
  [ -n "$U" ] && break; sleep 1
done
[ -z "$U" ] && { echo "no tunnel url"; exit 1; }
echo "$U/phone" > ~/demo_logs/phone_url.txt
PHONE_URL="$U/phone" "$(dirname "$0")/run_demo.sh" start
echo "phone page: $U/phone"
