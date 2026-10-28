import json
import os
import threading
import urllib.error
import urllib.request

import pytest

from mesh_dashboard.server import make_server

STATIC = os.path.join(os.path.dirname(__file__), "..", "mesh_dashboard", "static")


class FakeBridge:
    """Stands in for the Zenoh bridge: records what the server asked it to do."""
    def __init__(self):
        self.calls = []

    def snapshot(self):
        return {"coord": {"mode": "off"}, "stats": {"rover": {"held": {"0": 0, "1": 2, "2": 0}}}}

    def events(self, after):
        return [{"event": "expired", "unit": "rover"}][after:], 1

    def set_mesh(self, on):
        self.calls.append(("mesh", on))

    def set_failure(self, cfg):
        self.calls.append(("failure", cfg))

    def take_photo(self):
        self.calls.append(("photo",))


@pytest.fixture
def server():
    b = FakeBridge()
    srv = make_server(b, port=0)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    yield f"http://127.0.0.1:{srv.server_address[1]}", b
    srv.shutdown()


def post(url, body):
    req = urllib.request.Request(url, data=body if isinstance(body, bytes) else json.dumps(body).encode(),
                                 headers={"Content-Type": "application/json"}, method="POST")
    try:
        return urllib.request.urlopen(req, timeout=3).status
    except urllib.error.HTTPError as e:
        return e.code


def test_index_served(server):
    url, _ = server
    r = urllib.request.urlopen(url + "/", timeout=3)
    assert r.status == 200 and b"MISSION CONTROL" in r.read()


def test_static_js_served(server):
    url, _ = server
    assert urllib.request.urlopen(url + "/static/app.js", timeout=3).status == 200


def test_path_traversal_refused(server):
    url, _ = server
    with pytest.raises(urllib.error.HTTPError) as e:
        urllib.request.urlopen(url + "/static/..%2f..%2fsetup.py", timeout=3)
    assert e.value.code == 404


def test_events_streams_snapshot(server):
    url, _ = server
    r = urllib.request.urlopen(url + "/events", timeout=3)
    assert r.headers["Content-Type"].startswith("text/event-stream")
    lines = []
    while True:
        line = r.readline().decode().strip()
        if not line and lines:
            break
        if line:
            lines.append(line)
    assert lines[0] == "event: snapshot"
    assert "stats" in json.loads(lines[1][len("data: "):])
    r.close()


def test_post_mesh_calls_bridge(server):
    url, b = server
    assert post(url + "/api/mesh", {"on": True}) == 200
    assert b.calls == [("mesh", True)]


def test_post_failure_passes_valid_settings(server):
    url, b = server
    assert post(url + "/api/failure", {"drop_probability": 0.1, "kill": "drone_2"}) == 200
    assert b.calls == [("failure", {"drop_probability": 0.1, "kill": "drone_2"})]


def test_post_failure_rejects_unknown_key(server):
    url, b = server
    assert post(url + "/api/failure", {"explode": True}) == 400
    assert b.calls == []


def test_post_failure_rejects_bad_values(server):
    url, b = server
    assert post(url + "/api/failure", {"drop_probability": 3}) == 400
    assert post(url + "/api/failure", {"kill": "nobody"}) == 400
    assert b.calls == []


def test_bad_json_400(server):
    url, _ = server
    assert post(url + "/api/mesh", b"{not json") == 400


def test_post_photo(server):
    url, b = server
    assert post(url + "/api/photo", {}) == 200
    assert b.calls == [("photo",)]


def test_no_external_urls():
    for name in os.listdir(STATIC):
        text = open(os.path.join(STATIC, name), encoding="utf-8").read()
        text = text.replace("http://www.w3.org/2000/svg", "")     # SVG namespace name, never fetched
        assert "http://" not in text and "https://" not in text, name
