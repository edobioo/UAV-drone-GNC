"""Minimal Qt playback controls and live telemetry."""
import os
os.environ['QT_API'] = 'pyside6'

from bisect import bisect_right
from time import monotonic
import warnings
from PySide6 import QtCore, QtGui, QtWidgets
from pyvistaqt import QtInteractor
from .data import load_csv
from .playback import Playback
from .scene import FlightScene


class ViewerWindow(QtWidgets.QMainWindow):
    def __init__(self, flight, config, path, notices=()):
        super().__init__()
        self._closing = False
        self.setAttribute(QtCore.Qt.WidgetAttribute.WA_DeleteOnClose)
        self.config = config
        self.setWindowTitle('UAV')
        self.resize(1280, 850)
        self.setStyleSheet('''
            QMainWindow, QWidget { background: #152131; color: #e0e8f2; }
            QPushButton, QComboBox { background: #27394d; padding: 6px; border-radius: 4px; }
            QPushButton:hover { background: #355269; }
            QPushButton:disabled { color: #61778e; }
            QCheckBox { padding: 4px; }
        ''')
        central = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(central)
        layout.setContentsMargins(6, 6, 6, 6)
        self.plotter = QtInteractor(central, auto_update=False, multi_samples=0)
        layout.addWidget(self.plotter.interactor, 1)

        controls = QtWidgets.QHBoxLayout()
        # Give every control its Qt parent at construction. Adding an initially
        # parentless widget through a nested layout can leave ownedByPython()
        # true in PySide even after Qt reparents it: both then try to delete it.
        self.play_button = self._button(controls, '▶', 'Play', self.play, central)
        self.pause_button = self._button(controls, '⏸', 'Pausa', self.pause, central)
        self.speed_box = QtWidgets.QComboBox(central)
        self.speed_box.setToolTip('Velocità di riproduzione')
        self.speed_box.setAccessibleName('Velocità di riproduzione')
        for speed in (.25, .5, 1., 2., 4.):
            self.speed_box.addItem(f'{speed:g}×', speed)
        self.speed_box.setCurrentIndex(2)
        self.speed_box.currentIndexChanged.connect(self.change_speed)
        controls.addWidget(self.speed_box)
        controls.addStretch()
        self.axes_box = self._check(controls, 'Terna body', lambda b: self.set_visibility(axes=b), central)
        self.trail_box = self._check(controls, 'Traiettoria', lambda b: self.set_visibility(trail=b), central)
        layout.addLayout(controls)

        self.telemetry = QtWidgets.QLabel(central)
        self.telemetry.setTextFormat(QtCore.Qt.TextFormat.PlainText)
        self.telemetry.setStyleSheet('font-family: monospace; padding: 4px;')
        layout.addWidget(self.telemetry)
        self.notice = QtWidgets.QLabel(central)
        self.notice.setWordWrap(True)
        self.notice.setTextFormat(QtCore.Qt.TextFormat.PlainText)
        self.notice.setStyleSheet('color: #ffd166; padding: 3px;')
        layout.addWidget(self.notice)
        self.setCentralWidget(central)

        self.shortcuts = []
        for key, callback in [('Space', self.toggle_play),
                              (QtGui.QKeySequence.StandardKey.Open, self.open_file)]:
            shortcut = QtGui.QShortcut(QtGui.QKeySequence(key), self)
            shortcut.activated.connect(callback)
            self.shortcuts.append(shortcut)
        self.install_flight(flight, path, notices)
        self.timer = QtCore.QTimer(self)
        self.timer.setTimerType(QtCore.Qt.TimerType.PreciseTimer)
        self.timer.timeout.connect(self.tick)
        self.timer.start(max(1, round(1000 / config['display']['fps'])))

    @staticmethod
    def _button(layout, symbol, name, callback, parent):
        button = QtWidgets.QPushButton(symbol, parent)
        button.setFixedSize(36, 32)
        button.setToolTip(name)
        button.setAccessibleName(name)
        button.clicked.connect(callback)
        layout.addWidget(button)
        return button

    @staticmethod
    def _check(layout, text, callback, parent):
        box = QtWidgets.QCheckBox(text, parent)
        box.setChecked(True)
        box.toggled.connect(callback)
        layout.addWidget(box)
        return box

    def install_flight(self, flight, path, notices=()):
        if self._closing:
            return
        self.flight = flight
        self.player = Playback(flight)
        self.player.speed = self.speed_box.currentData()
        if hasattr(self, 'scene'):
            self.scene.close()
        self.plotter.clear()
        self.scene = FlightScene(self.plotter, flight, self.config)
        self.scene.visibility(axes=self.axes_box.isChecked(), trail=self.trail_box.isChecked())
        self.base_notice = ' | '.join(str(item) for item in notices)
        self.update_frame()

    def open_file(self):
        if self._closing:
            return
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, 'Apri una simulazione', '', 'CSV (*.csv)')
        if not path or self._closing:
            return
        self.pause()
        try:
            with warnings.catch_warnings(record=True) as caught:
                warnings.simplefilter('always')
                flight = load_csv(path, self.config)
            self.install_flight(flight, path, [item.message for item in caught])
        except (OSError, ValueError) as exc:
            QtWidgets.QMessageBox.critical(self, 'Errore CSV', str(exc))

    def play(self):
        if self._closing:
            return
        if self.player.blocked:
            # A second explicit Play resumes at the next recorded sample;
            # never synthesize motion across a missing interval.
            index = bisect_right(self.flight.times, self.player.time)
            self.player.seek(self.flight.times[min(index, len(self.flight.times) - 1)])
        self.player.play()
        self.update_frame()

    def pause(self):
        if self._closing:
            return
        self.player.pause()
        self.update_frame()

    def toggle_play(self):
        self.pause() if self.player.playing else self.play()

    def change_speed(self):
        if self._closing:
            return
        self.player.set_speed(self.speed_box.currentData())
        self.update_frame()

    def set_visibility(self, **kwargs):
        if self._closing:
            return
        self.scene.visibility(**kwargs)
        self.plotter.render()

    def tick(self):
        if not self._closing and self.player.playing:
            self.player.tick()
            self.update_frame(refresh_ui=(not self.player.playing or
                                          monotonic() - self._last_ui_update >= .05))

    def update_frame(self, refresh_ui=True):
        if self._closing:
            return
        pose = self.flight.sample(self.player.time)
        self.scene.update(pose)
        if refresh_ui:
            self._update_telemetry(pose)
            self._last_ui_update = monotonic()
        self.plotter.render()

    def _update_telemetry(self, pose):
        self.play_button.setEnabled(not self.player.playing and bool(self.flight.duration))
        self.pause_button.setEnabled(self.player.playing)
        p = ', '.join(f'{v:+.3f}' for v in pose.position)
        q = ', '.join(f'{v:+.4f}' for v in pose.quaternion_wxyz)
        text = f't = {pose.time:.3f} s / {self.flight.end:.3f} s    CoM ENU [m] = ({p})\nq ENU←B [w,x,y,z] = ({q})'
        if pose.velocity is not None:
            text += f'    |v| = {sum(pose.velocity ** 2) ** .5:.3f} m/s'
        if pose.angular_velocity is not None:
            text += f'    |ω| = {sum(pose.angular_velocity ** 2) ** .5:.3f} rad/s'
        self.telemetry.setText(text)
        blocked = 'Lacuna temporale: riproduzione in pausa. Premi ▶ per continuare dal prossimo campione.'
        message = blocked if self.player.blocked else self.base_notice
        self.notice.setText(message)
        self.notice.setVisible(bool(message))

    def shutdown(self):
        """Release graphics while Qt is alive; queued UI calls become no-ops."""
        if self._closing:
            return
        self._closing = True
        self.player.playing = False
        self.timer.stop()
        self.timer.timeout.disconnect(self.tick)
        for shortcut in self.shortcuts:
            shortcut.setEnabled(False)
        self.plotter.setUpdatesEnabled(False)
        self.plotter.blockSignals(True)
        self.scene.close()
        self.plotter.close()
        self.scene = None

    def closeEvent(self, event):
        self.shutdown()
        super().closeEvent(event)
