"""Opt-in desktop/offscreen integration tests: UAV_VIEWER_RENDER_TESTS=1."""
import os
from pathlib import Path
import statistics
import subprocess
import sys
import time
import numpy as np
import pytest

pytestmark = pytest.mark.skipif(os.environ.get('UAV_VIEWER_RENDER_TESTS') != '1', reason='Test grafici opt-in')


@pytest.mark.parametrize('position,parallel', [
    ([0, 0, 0], False), ([0, 0, 1000], False),
    ([20000, -30000, 500], False), ([0, 0, -100], False),
    ([0, 0, 1000], True),
])
def test_grid_is_always_on_ground_and_fades_at_distance(position, parallel):
    pv = pytest.importorskip('pyvista')
    from uav_viewer.grid import grid_layout
    camera = pv.Camera()
    camera.focal_point = position
    camera.position = np.asarray(position) + [3, -4, 3]
    camera.parallel_projection = parallel
    layout = grid_layout(camera, .55, 1.6)
    assert np.isfinite(layout.center).all()
    assert layout.center[2] == 0
    assert layout.spacing > 0
    assert 0 <= layout.blend <= 1
    assert layout.opacity == (1 if position[2] == 0 else 0)


def test_grid_scale_changes_crossfade_without_a_pop():
    pv = pytest.importorskip('pyvista')
    from uav_viewer.grid import grid_layout
    camera = pv.Camera()
    camera.parallel_projection = True
    camera.focal_point = (0, 0, 0)
    camera.position = (0, -10, 10)
    # Target spacing passes through exactly one metre at this view width.
    layouts = []
    for factor in (1 - 1e-6, 1 + 1e-6):
        camera.parallel_scale = 6 * factor
        layouts.append(grid_layout(camera, .55, 1))
    weights = [{layout.spacing: 1 - layout.blend, layout.spacing * 2: layout.blend}
               for layout in layouts]
    assert weights[0][1.] == pytest.approx(weights[1][1.], abs=1e-6)
    assert weights[0][.5] < 1e-6
    assert weights[1][2.] < 1e-6


def test_grid_altitude_fade_is_continuous_and_reversible():
    pv = pytest.importorskip('pyvista')
    from uav_viewer.grid import grid_layout
    camera = pv.Camera()
    camera.parallel_projection = True
    camera.parallel_scale = 2
    heights = np.linspace(0, 30, 301)
    opacities = []
    for altitude in heights:
        camera.focal_point = (0, 0, altitude)
        camera.position = (3, -4, altitude + 3)
        layout = grid_layout(camera, .55, 1.6)
        assert layout.center[2] == 0
        opacities.append(layout.opacity)
    assert opacities[0] == 1
    assert opacities[-1] == 0
    assert np.max(np.diff(opacities)) <= 0
    assert np.max(np.abs(np.diff(opacities))) < .01
    camera.focal_point = (0, 0, 0)
    camera.position = (3, -4, 3)
    assert grid_layout(camera, .55, 1.6).opacity == 1


def test_scene_and_absolute_transform(tmp_path, demo_dir):
    pv = pytest.importorskip('pyvista')
    pytest.importorskip('scipy')
    from uav_viewer.config import load_config
    from uav_viewer.data import load_csv
    from uav_viewer.scene import FlightScene
    flight = load_csv(demo_dir / 'demo_yaw90.csv', load_config())
    plotter = pv.Plotter(off_screen=True, window_size=(640, 480))
    try:
        scene = FlightScene(plotter, flight, load_config())
        original = [actor.mapper.dataset.points.copy() for actor in scene.actors]
        pose = flight.sample(4)
        scene.update(pose)
        scene.update(pose)
        for actor, points in zip(scene.actors, original):
            np.testing.assert_allclose(actor.user_matrix, pose.matrix)
            np.testing.assert_allclose(actor.mapper.dataset.points, points)
        image = plotter.screenshot(str(tmp_path / 'scene.png'))
        assert image.shape == (480, 640, 3)
        assert np.std(image) > 5
    finally:
        plotter.close()


def test_world_axes_and_ground_scale_for_high_altitude_flight():
    pv = pytest.importorskip('pyvista')
    from scipy.spatial.transform import Rotation
    from uav_viewer.config import load_config
    from uav_viewer.data import FlightData
    from uav_viewer.scene import FlightScene
    config = load_config()
    flight = FlightData([0, 1], [[0, 0, 0], [0, 0, 1000]], Rotation.identity(2))
    plotter = pv.Plotter(off_screen=True, window_size=(640, 480))
    try:
        scene = FlightScene(plotter, flight, config)
        length = config['model']['world_axes_length_m']
        for axis, actor in enumerate(scene.enu_axes):
            bounds = np.asarray(actor.bounds).reshape(3, 2)
            assert bounds[axis, 1] == pytest.approx(length)
            assert np.max(bounds[:, 1] - bounds[:, 0]) == pytest.approx(length)
        grid_mesh = scene.grid.mesh
        assert all(actor.bounds[4] == 0 for actor in scene.grid.actors)
        scene.update(flight.sample(1))
        # No moving reference planes and no grid cost at large distance.
        assert scene.enu_axes[2].bounds[5] == pytest.approx(length)
        assert not any(actor.GetVisibility() for actor in scene.grid.actors)
        scene.set_mode('free')
        assert all(actor.bounds[4] == 0 for actor in scene.grid.actors)
        assert any(actor.GetVisibility() for actor in scene.grid.actors)
        assert all(actor.mapper.dataset is grid_mesh for actor in scene.grid.actors)
        scene.set_mode('attitude')
        plotter.camera.zoom(.1)
        plotter.screenshot()
        assert all(actor.mapper.dataset is grid_mesh for actor in scene.grid.actors)
    finally:
        plotter.close()


def test_changing_grid_altitude_does_not_stall_rendering():
    pv = pytest.importorskip('pyvista')
    from uav_viewer.config import load_config
    from uav_viewer.grid import AdaptiveGrid
    plotter = pv.Plotter(off_screen=True, window_size=(640, 480))
    try:
        grid = AdaptiveGrid(plotter, load_config()['model'])
        plotter.camera_position = [(3, -4, 1003), (0, 0, 1000), (0, 0, 1)]
        grid.update()
        plotter.show(auto_close=False, interactive=False)

        def render_frames(moving):
            durations = []
            for frame in range(12):
                altitude = 1 + frame * .01 if moving else 1
                start = time.perf_counter()
                plotter.camera_position = [(3, -4, altitude + 3), (0, 0, altitude), (0, 0, 1)]
                grid.update()
                plotter.render()
                if frame >= 3:  # Exclude rendering warm-up.
                    durations.append(time.perf_counter() - start)
            return statistics.median(durations)

        stationary = render_frames(False)
        moving = render_frames(True)
        # Relative guard against the previously observed 5-6x slowdown;
        # do not impose a particular GPU speed or a 60 fps hardware requirement.
        assert moving < stationary * 4 + .01, (stationary, moving)
        assert 'grid_scale' not in plotter.renderer.actors
    finally:
        plotter.close()


def test_grid_geometry_stays_bounded_and_clipping_includes_visible_lines():
    pv = pytest.importorskip('pyvista')
    from uav_viewer.config import load_config
    from uav_viewer.grid import AdaptiveGrid
    plotter = pv.Plotter(off_screen=True, window_size=(640, 480))
    try:
        grid = AdaptiveGrid(plotter, load_config()['model'])
        mesh_time = grid.mesh.GetMTime()
        assert grid.mesh.n_cells < 1000
        for altitude in (0, 1, 2, 10, 1000, 100000, -1000, 0):
            plotter.camera_position = [(3, -4, altitude + 3), (0, 0, altitude), (0, 0, 1)]
            plotter.camera.clipping_range = (1, 8)
            grid.update()
            assert all(actor.bounds[4] == 0 for actor in grid.actors)
            if abs(altitude) > 100:
                assert not any(actor.GetVisibility() for actor in grid.actors)
                assert plotter.camera.clipping_range[1] == 8
            else:
                assert plotter.camera.clipping_range[1] > 8
            for actor in grid.actors:
                spacing = actor.GetScale()[0]
                for coordinate in actor.GetPosition()[:2]:
                    assert coordinate / spacing == pytest.approx(round(coordinate / spacing))
            assert grid.mesh.GetMTime() == mesh_time
    finally:
        plotter.close()


def test_decimated_trajectory_keeps_gaps_and_seeks_correct():
    pv = pytest.importorskip('pyvista')
    from scipy.spatial.transform import Rotation
    from uav_viewer.config import load_config
    from uav_viewer.data import FlightData
    from uav_viewer.scene import FlightScene
    times = np.r_[np.linspace(0, 2, 1000), np.linspace(5, 7, 1000)]
    positions = np.column_stack((times, times * 0, times * 0))
    flight = FlightData(times, positions, Rotation.identity(len(times)))
    config = load_config()
    config['display']['max_trail_points'] = 100
    plotter = pv.Plotter(off_screen=True)
    try:
        scene = FlightScene(plotter, flight, config)
        assert len(scene.trail_positions) <= 100
        assert scene.tail.n_verts == 0
        assert len(flight.positions) == 2000
        lines = scene.full_trail.mapper.dataset.lines.reshape(-1, 3)[:, 1:]
        endpoints = scene.trail_times[lines]
        assert not np.any((endpoints[:, 0] < 5) & (endpoints[:, 1] > 2))
        for instant in (1.5, 6, 1.5, 7, 0):
            pose = flight.sample(instant)
            scene.update(pose)
            assert scene.tail.points[:, 0].max() <= instant
            assert scene.tail.points[:, 0].min() >= (5 if instant >= 5 else 0)
            np.testing.assert_allclose(pose.position, [instant, 0, 0])
    finally:
        plotter.close()


def test_body_axes_are_short_and_thin():
    pv = pytest.importorskip('pyvista')
    from uav_viewer.config import load_config
    from uav_viewer.model import build_model
    config = load_config()['model']
    plotter = pv.Plotter(off_screen=True)
    try:
        _, axes = build_model(plotter, config)
        length = config['axes_length_m']
        assert length < config['world_axes_length_m']
        for axis, actor in enumerate(axes):
            points = actor.mapper.dataset.points
            assert points[:, axis].max() == pytest.approx(length)
            transverse = np.delete(points, axis, axis=1)
            assert np.max(np.linalg.norm(transverse, axis=1)) < .006
            shaft = points[:, axis] < length * .85
            assert np.max(np.linalg.norm(transverse[shaft], axis=1)) < .003
    finally:
        plotter.close()


def test_vectors_are_not_rendered(demo_dir):
    pv = pytest.importorskip('pyvista')
    from uav_viewer.config import load_config
    from uav_viewer.data import FlightData, load_csv
    from uav_viewer.scene import FlightScene
    flight = load_csv(demo_dir / 'demo_flight.csv', load_config())
    pose_only = FlightData(flight.times, flight.positions, flight.rotations)
    plotter = pv.Plotter(off_screen=True)
    try:
        counts, labels = [], []
        for data in (flight, pose_only):
            scene = FlightScene(plotter, data, load_config())
            scene.update(data.sample(10))
            counts.append(plotter.renderer.GetActors().GetNumberOfItems())
            labels.append([actor.GetInput() for actor in plotter.renderer.actors.values()
                           if hasattr(actor, 'GetInput')])
            scene.close()
            plotter.clear()
        assert counts[0] == counts[1]
        assert labels[0] == labels[1]
    finally:
        plotter.close()


def test_qt_minimal_controls_and_live_telemetry(demo_dir):
    os.environ['QT_API'] = 'pyside6'
    pytest.importorskip('pyvistaqt')
    from PySide6 import QtWidgets
    from uav_viewer.app import ViewerWindow
    from uav_viewer.config import load_config
    from uav_viewer.data import load_csv
    config = load_config()
    path = demo_dir / 'demo_flight.csv'
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    window = ViewerWindow(load_csv(path, config), config, path)
    window.timer.stop()
    window.show()
    now = [0.]
    window.player.clock = lambda: now[0]
    window.player._anchor()
    try:
        app.processEvents()
        assert [b.text() for b in window.findChildren(QtWidgets.QPushButton)] == ['▶', '⏸']
        assert len(window.findChildren(QtWidgets.QComboBox)) == 1
        assert [b.text() for b in window.findChildren(QtWidgets.QCheckBox)] == ['Terna body', 'Traiettoria']
        assert not window.findChildren(QtWidgets.QSlider)
        assert not window.findChildren(QtWidgets.QDoubleSpinBox)
        assert 'demo_flight.csv' not in window.telemetry.text()
        assert window.play_button.isEnabled()
        assert not window.pause_button.isEnabled()
        assert window.notice.isHidden()

        window.play_button.click()
        assert window.player.playing
        assert not window.play_button.isEnabled()
        assert window.pause_button.isEnabled()
        now[0] = 2.
        window._last_ui_update = -1.
        window.tick()
        assert 't = 2.000 s' in window.telemetry.text()
        assert '|v|' in window.telemetry.text()
        assert '|ω|' in window.telemetry.text()
        window.speed_box.setCurrentIndex(3)
        assert window.player.speed == 2
        now[0] = 3.
        window.pause_button.click()
        assert not window.player.playing
        assert window.player.time == 4
        assert 't = 4.000 s' in window.telemetry.text()

        window.axes_box.setChecked(False)
        window.trail_box.setChecked(False)
        assert not any(actor.GetVisibility() for actor in window.scene.body_axes)
        assert not window.scene.tail_actor.GetVisibility()
        assert not window.scene.full_trail.GetVisibility()
        window.axes_box.setChecked(True)
        assert all(actor.GetVisibility() for actor in window.scene.body_axes)
        window.axes_box.setChecked(False)

        window.play_button.click()
        now[0] = 100
        window.tick()
        assert not window.player.playing
        assert window.player.time == window.flight.end
        assert 't = 30.000 s' in window.telemetry.text()
        window.play_button.click()
        assert window.player.time == window.flight.start
        assert window.player.playing
        window.pause_button.click()

        old_observer = window.scene._grid_observer
        window.install_flight(load_csv(path, config), path)
        assert window.plotter.renderer.GetCommand(old_observer) is None
        assert not window.scene.show_axes
        assert not window.scene.show_trail
        assert not window.player.playing
        assert 't = 0.000 s' in window.telemetry.text()
    finally:
        window.close()
        app.processEvents()


def test_qt_gap_pauses_and_explicit_play_resumes_without_interpolation(demo_dir):
    os.environ['QT_API'] = 'pyside6'
    pytest.importorskip('pyvistaqt')
    from PySide6 import QtWidgets
    from uav_viewer.app import ViewerWindow
    from uav_viewer.config import load_config
    from uav_viewer.data import load_csv
    path = demo_dir / 'demo_gap.csv'
    with pytest.warns(UserWarning, match='lacune'):
        flight = load_csv(path, load_config())
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    window = ViewerWindow(flight, load_config(), path)
    window.timer.stop()
    now = [0.]
    window.player.clock = lambda: now[0]
    window.player._anchor()
    try:
        window.play_button.click()
        now[0] = 4
        window.tick()
        assert window.player.blocked
        assert not window.player.playing
        assert window.player.time == 3
        assert not window.notice.isHidden()
        assert 'Lacuna' in window.notice.text()
        window.play_button.click()
        assert window.player.time == 6
        assert window.player.playing
        assert not window.player.allow_gaps
        assert window.notice.isHidden()
    finally:
        window.close()
        app.processEvents()


def test_qt_shutdown_owns_controls_once_and_ignores_late_callbacks(demo_dir):
    os.environ['QT_API'] = 'pyside6'
    pytest.importorskip('pyvistaqt')
    from PySide6 import QtCore, QtWidgets
    from shiboken6 import isValid, ownedByPython
    from uav_viewer.app import ViewerWindow
    from uav_viewer.config import load_config
    from uav_viewer.data import load_csv
    path = demo_dir / 'demo_flight.csv'
    application = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    window = ViewerWindow(load_csv(path, load_config()), load_config(), path)
    controls = [window.axes_box, window.trail_box, window.speed_box,
                window.play_button, window.pause_button, window.telemetry]
    for control in controls:
        assert control.parent() is window.centralWidget()
        assert not ownedByPython(control)
    window.play()
    scene = window.scene
    renderer = window.plotter.renderer
    observer = scene._grid_observer
    window.shutdown()
    assert not window.timer.isActive()
    assert not window.player.playing
    assert window.plotter._closed
    assert renderer.GetCommand(observer) is None
    window.shutdown()
    scene.close()
    scene._update_grid()
    window.close()
    QtCore.QCoreApplication.sendPostedEvents(window, QtCore.QEvent.Type.DeferredDelete)
    assert not isValid(window)
    assert not any(isValid(control) for control in controls)
    # Simulate events queued before the close: none may touch deleted Qt/VTK.
    for callback in (window.tick, window.update_frame, window.play, window.pause,
                     window.change_speed, lambda: window.set_visibility(axes=False)):
        QtCore.QTimer.singleShot(0, callback)
    application.processEvents()


@pytest.mark.parametrize('mode', ['close-paused', 'close-playing', 'quit-playing'])
def test_cli_shutdown_survives_interpreter_exit(demo_dir, tmp_path, mode):
    pytest.importorskip('pyvistaqt')
    script = Path(__file__).with_name('shutdown_probe.py')
    env = dict(os.environ, PYTHONFAULTHANDLER='1', OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1',
               MPLCONFIGDIR=str(tmp_path / 'mpl'))
    env.setdefault('QT_QPA_PLATFORM', 'offscreen')
    result = subprocess.run([sys.executable, str(script), str(demo_dir / 'demo_flight.csv'), mode],
                            env=env, capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stdout + result.stderr
    assert f'shutdown OK {mode}' in result.stdout
