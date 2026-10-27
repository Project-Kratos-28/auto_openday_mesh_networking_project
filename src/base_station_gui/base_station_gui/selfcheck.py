"""GUI self-check (used by the mesh self-test). Opens the window off-screen, clicks the
mesh toggle ON then OFF, checks the coordinator followed, saves a screenshot.
    python3 -m base_station_gui.selfcheck [screenshot.png]
"""
import os
import sys
import time

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")      # no window pops up


def main():
    from PyQt6.QtWidgets import QApplication
    from base_station_gui.app import BaseStationWindow
    from base_station_gui.mesh_bridge import MeshBridge
    shot = sys.argv[1] if len(sys.argv) > 1 else "/tmp/gui_selfcheck.png"
    app = QApplication([])
    bridge = MeshBridge()
    win = BaseStationWindow(bridge)
    win.show()

    def wait(cond, timeout):
        end = time.time() + timeout
        while time.time() < end:
            app.processEvents()
            win.refresh()
            if cond():
                return True
            time.sleep(0.1)
        return False

    ok = True
    if not wait(lambda: bridge.snapshot()["coordinator_running"], 10):
        print("GUI: FAIL coordinator not seen"); sys.exit(1)
    order = ("off", "on") if bridge.snapshot()["mesh_mode"] == "on" else ("on", "off")
    for want in order:                                           # flip it both ways
        win.toggle.click()                                       # what a user's click does
        good = wait(lambda: bridge.snapshot()["mesh_mode"] == want and win.pending is None
                    and win.toggle.isChecked() == (want == "on"), 8)
        print(f"GUI: toggle -> {want.upper()}: {'OK' if good else 'FAIL'}")
        ok &= good
        if want == "on" and good:
            wait(lambda: bridge.snapshot()["base_b_receiving"], 8)
            win.refresh()
            win.grab().save(shot)                                # picture with the mesh ON
            print(f"GUI: screenshot saved to {shot}")
    bridge.close()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
