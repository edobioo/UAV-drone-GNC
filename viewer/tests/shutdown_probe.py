"""Exercise the real CLI event loop and interpreter exit in a fresh process."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from PySide6 import QtCore, QtWidgets
from shiboken6 import isValid, ownedByPython
from uav_viewer import app as viewer_app
import view


def main():
    path, mode = sys.argv[1:]
    application = QtWidgets.QApplication([])
    windows = []  # Deliberately retain Python wrappers through interpreter exit.
    original_window = viewer_app.ViewerWindow

    class ProbeWindow(original_window):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            windows.append(self)
            for control in (self.axes_box, self.trail_box, self.play_button,
                            self.pause_button, self.speed_box):
                assert not ownedByPython(control)
            if mode != 'close-paused':
                self.play()
            action = application.quit if mode == 'quit-playing' else self.close
            QtCore.QTimer.singleShot(50, action)

    viewer_app.ViewerWindow = ProbeWindow
    sys.argv = ['view.py', path]
    for _ in range(2):
        assert view.main() == 0
        assert not isValid(windows[-1])
    print('shutdown OK', mode, flush=True)


if __name__ == '__main__':
    main()
