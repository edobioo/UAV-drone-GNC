"""Scene actors are created once; UAV poses use absolute rigid transforms."""
import numpy as np
import pyvista as pv
from vtkmodules.vtkRenderingCore import vtkBillboardTextActor3D
from vtkmodules.vtkCommonMath import vtkMatrix4x4
from .model import build_model, AXIS_COLORS
from .grid import AdaptiveGrid


def label(plotter, text, position, color='#d7e0ed', size=15):
    actor = vtkBillboardTextActor3D()
    actor.SetInput(text)
    actor.SetPosition(*position)
    prop = actor.GetTextProperty()
    prop.SetColor(*pv.Color(color).float_rgb)
    prop.SetFontSize(size)
    prop.SetBold(True)
    plotter.add_actor(actor, reset_camera=False)
    return actor


class FlightScene:
    def __init__(self, plotter, flight, config):
        self.plotter = plotter
        self.flight = flight
        self.config = config
        self._closed = False
        self.mode = config['display']['camera']
        self.last_position = None
        self.pose = flight.sample(flight.start)
        self.show_axes = True
        self.show_trail = True
        plotter.set_background('#101925', top='#243449')
        # Prefer frame rate to supersampling; FXAA can silently fall back to
        # expensive SSAA on software/EGL renderers, so disable AA explicitly.
        plotter.render_window.SetMultiSamples(0)
        plotter.disable_anti_aliasing()
        self.actors, self.body_axes = build_model(plotter, config['model'])
        self._model_matrix = vtkMatrix4x4()
        for actor in self.actors:
            actor.SetUserMatrix(self._model_matrix)
        self.body_labels = [label(plotter, name, (0, 0, 0), color, size=11)
                            for name, color in zip(('Xb', 'Yb', 'Zb'), AXIS_COLORS)]
        positions = flight.positions
        lower, upper = positions.min(axis=0), positions.max(axis=0)
        center = (lower + upper) / 2
        extent = max(float(np.max(upper - lower)), 1.)
        self.grid = AdaptiveGrid(plotter, config['model'])
        # World axes have their own fixed metric length, independent of the
        # smaller body triad and of the trajectory extent.
        enu_length = config['model']['world_axes_length_m']
        self.enu_axes = []
        for direction, name, color in zip(np.eye(3), ('E / X', 'N / Y', 'U / Z'), AXIS_COLORS):
            self.enu_axes.append(plotter.add_mesh(
                pv.Arrow(direction=direction, scale=enu_length,
                         shaft_radius=.008, tip_radius=.025, tip_length=.12),
                color=color, reset_camera=False))
            label(plotter, name, direction * enu_length * 1.12, color)
        max_points = config['display']['max_trail_points']
        self.trail_indices = np.unique(np.linspace(0, len(positions) - 1, min(len(positions), max_points), dtype=int))
        self.trail_positions = positions[self.trail_indices]
        self.trail_times = flight.times[self.trail_indices]
        self.full_trail = None
        if len(positions) > 1:
            # Never draw a segment across a detected CSV gap.
            segments = []
            for k in range(len(self.trail_indices) - 1):
                left, right = self.trail_indices[k:k + 2]
                if not np.any((flight.gaps >= left) & (flight.gaps < right)):
                    segments.extend([2, k, k + 1])
            if segments:
                full = pv.PolyData(self.trail_positions, lines=np.asarray(segments))
                self.full_trail = plotter.add_mesh(full, color='#61778e', line_width=1.5, reset_camera=False)
        # Mutable tail with fixed connectivity. Unused points collapse to current CoM.
        tail_points = np.repeat(positions[:1], len(self.trail_indices) + 1, axis=0)
        tail_lines = np.hstack(([len(tail_points)], np.arange(len(tail_points))))
        self.tail = pv.PolyData(tail_points, lines=tail_lines)
        self.tail_actor = plotter.add_mesh(self.tail, color='#45d1c0', line_width=3, reset_camera=False)
        self._tail_points = np.asarray(self.tail.points)
        self.overview_center = center
        self.overview_extent = extent
        self.set_view('iso')
        self.update(self.pose)
        # VTK interaction renders also happen while playback is paused.
        self._renderer = plotter.renderer
        self._grid_observer = self._renderer.AddObserver('StartEvent', self._update_grid)

    def close(self):
        self._closed = True
        if self._grid_observer is not None:
            self._renderer.RemoveObserver(self._grid_observer)
            self._grid_observer = None
            self._renderer = None

    def _update_grid(self, *_):
        if not self._closed:
            self.grid.update()

    def set_mode(self, mode):
        self.mode = mode
        if mode == 'free':
            self.set_view('iso')
        elif mode == 'attitude':
            self.set_view('iso')
        self.last_position = self.pose.position.copy()

    def set_view(self, view):
        center = self.overview_center if self.mode == 'free' else self.pose.position
        scale = self.overview_extent * 1.6 + .8 if self.mode == 'free' else (1. if self.mode == 'attitude' else 2.5)
        offsets = {'iso': (1.3, -1.6, 1.), 'top': (0, 0, 2), 'east': (2, 0, 0), 'north': (0, 2, 0)}
        offset = np.asarray(offsets[view]) * scale
        up = (0, 1, 0) if view == 'top' else (0, 0, 1)
        self.plotter.camera_position = [center + offset, center, up]
        self._update_grid()
        self.plotter.reset_camera_clipping_range()

    def visibility(self, axes=None, trail=None):
        if axes is not None:
            self.show_axes = axes
            for actor in self.body_axes:
                actor.visibility = axes
            for actor in self.body_labels:
                actor.SetVisibility(axes)
        if trail is not None:
            self.show_trail = trail
            self.tail_actor.visibility = trail
            if self.full_trail is not None:
                self.full_trail.visibility = trail
        self.update(self.pose)

    def update(self, pose):
        self.pose = pose
        # All body parts share one VTK matrix; avoid allocating/validating a
        # Python/PyVista transform per actor on every frame.
        self._model_matrix.DeepCopy(pose.matrix.ravel())
        endpoints = np.eye(3) * self.config['model']['axes_length_m'] * 1.12
        endpoints = pose.rotation.apply(endpoints) + pose.position
        for actor, point in zip(self.body_labels, endpoints):
            actor.SetPosition(*point)
        count = int(np.searchsorted(self.trail_times, pose.time, side='right'))
        # Show only the continuous segment after the latest data gap.
        past_gaps = self.flight.gaps[self.flight.times[self.flight.gaps + 1] <= pose.time]
        segment_start = self.flight.times[past_gaps[-1] + 1] if len(past_gaps) else self.flight.start
        first = int(np.searchsorted(self.trail_times, segment_start))
        points = self._tail_points
        points[:] = pose.position
        used = count - first
        if used:
            points[:used] = self.trail_positions[first:count]
        self.tail.GetPoints().Modified()
        self.tail.Modified()
        if self.mode != 'free' and self.last_position is not None:
            delta = pose.position - self.last_position
            camera = self.plotter.camera
            camera.position = np.asarray(camera.position) + delta
            camera.focal_point = np.asarray(camera.focal_point) + delta
        self.last_position = pose.position.copy()
        self._update_grid()
        self.plotter.reset_camera_clipping_range()
