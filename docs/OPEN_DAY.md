# Open day: camera, photos, phone QR and the passcode

What was added on top of the v4 mission, how it works, and how to run it. Rehearsed end to end on the VM (Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic).

## Run it

```bash
./scripts/phone_tunnel.sh        # stops any old run, opens the phone tunnel, starts Gazebo + mesh + dashboard
```

1. Open the dashboard: `http://localhost:8080` (in the VM; from a Mac use `ssh -N -L 18080:127.0.0.1:8080 <vm>` then `http://localhost:18080`).
2. Click **MESH OFF** to turn it ON. A masked box asks for the passcode (once per browser tab).
3. Start the mission in a terminal: `ros2 run mesh_protocol mission`. The rover drives to the target and asks for a photo on arrival.
4. Show the live camera, press **TAKE PHOTO**, scan the QR with a phone and send a picture.
5. Do the failure demo last (kill a drone, packet loss). Reset by clicking "Restart simulation" in Gazebo: the mission re-arms by itself.

`scripts/mars_demo.sh start` (the teammate's one-command runner) does the same on a laptop with Gazebo and can also expose the **dashboard** with `--tunnel`. On the VM use `scripts/run_demo.sh`/`scripts/phone_tunnel.sh`, which start Gazebo with the classic `ogre` renderer (the VM's virtual GPU cannot run the default `ogre2`).

## Passcode

Watching the dashboard is public. The three controls (`/api/mesh`, `/api/failure`, `/api/photo`) need the header `X-Passcode`.

- The browser asks in a **masked** dialog (`<input type="password">`, dots instead of letters). A wrong passcode shows an error and asks again; Cancel leaves the controls locked. The right one is kept for the tab only (`sessionStorage`), never in the source.
- Default: the built-in `kratos123`. Change it without editing code: `MESH_PASSCODE=... ./scripts/phone_tunnel.sh`, or `--passcode TEXT` on the dashboard. `--passcode ""` turns the check off (loopback demos).
- A wrong guess costs the caller one second, so guessing is slow.
- The passcode travels in clear text over plain http. It keeps visitors out; it is not real authentication. Put TLS in front (the Cloudflare tunnel does) on an untrusted network.

## Camera and photos

| Piece | How it works |
|---|---|
| Rover camera | A Gazebo `camera` sensor on the rover chassis (320×240, 2 Hz, topic `/rover/camera`). The world uses the `ogre` render engine so it also renders on the VM. |
| Live feed | The rover turns the newest frame into a JPEG (q55) every 0.5 s and sends it as a `FEED` bundle: custody transfer, lifetime 1 s, priority 2, so old frames are dropped rather than queued. The base publishes `mesh/base/feed`; the dashboard shows it and marks frames older than 2 s as frozen. |
| Photo by button / on arrival | `TAKE PHOTO` (or the mission arriving) publishes `mesh/base/photo_cmd`. The base sends `PHOTO_REQ`; the rover grabs a real frame (PNG; the stock picture if there is no camera) and sends it as `PHOTO_CHUNK`s (1 KB, base64, sha256 for the whole file). The base reassembles and the dashboard shows it. The button walks through "requested, receiving n/N, photo taken". |
| Message from the base (phone picture) | The QR opens `/phone` on a small separate server (port 8081, only `/phone` and `/upload`, 15 MB max, images only). The dashboard shrinks the picture (EXIF rotation applied, max 480 px, baseline JPEG q70) and publishes `mesh/base/uplink_photo`. The **base** sends it to the **rover** over the mesh (`PHOTO_CHUNK` bundles, destination `rover`). The rover publishes what it has received on `mesh/rover/uplink`, and the dashboard's **Rover screen** shows the picture forming from the top down. With the mesh OFF and the rover out of reach the chunks wait at the base and the screen stays static until the mesh is switched ON. |
| Packet log | The base logs each `FEED` and `PHOTO_CHUNK` it receives (type, seq, route, delay), merged into one line with a `×N` count. |

Photo ids are clock based; the base ignores chunks of older photos, so a late chunk never replaces a newer picture.

## The phone QR works on any device

The QR encodes `PHONE_URL`, else the machine's LAN address. A LAN address only works for phones on the same network (and not at all when the phone page runs inside a VM). `./scripts/phone_tunnel.sh` instead starts a Cloudflare quick tunnel to port 8081 (`cloudflared` comes from `scripts/mars_demo.sh deps --tunnel`) and sets `PHONE_URL` to its `https://<random>.trycloudflare.com/phone` address, so any phone on any network can scan it. The address is new on every run.

The upload page has no passcode and is reachable by anyone who has the link. Do not post the link publicly.

## Dependencies

`segno` (QR image) and Pillow (image resize/encode) are Python packages: `pip3 install --user --break-system-packages segno pillow` (also in `scripts/requirements.txt`). Without `segno`, `/qr.svg` answers 404 "pip install segno" and the page still works; without Pillow the camera and phone upload fail.

## Not covered

- No automated tests for the feed, the `Camera` class, the phone upload or the packet events (checked in the live run).
- After killing and reviving a drone, a photo taken straight away may not complete until the relay chain has re-formed; rehearse the failure demo last.
