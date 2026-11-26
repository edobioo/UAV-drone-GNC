"""World-fixed ground grid with bounded geometry and continuous fading."""
from dataclasses import dataclass
import math
import numpy as np
import pyvista as pv


def smoothstep(value):
    value = float(np.clip(value, 0., 1.))
    return value * value * (3 - 2 * value)


@dataclass(frozen=True)
class GridLayout:
    center: np.ndarray
    spacing: float
    blend: float
    opacity: float


def grid_layout(camera, model_size, aspect):
    """Two doubling scales crossfade, always anchored to ENU z=0.

    Fade the reference out when it is far from the viewed subject rather than
    moving it towards the UAV. No altitude bands, playback state or timers.
    """
    focus = np.asarray(camera.focal_point)
    eye = np.asarray(camera.position)
    distance = np.linalg.norm(eye - focus)
    height = (2 * camera.parallel_scale if camera.parallel_projection else
              2 * distance * math.tan(math.radians(camera.view_angle) / 2))
    width = max(height * max(aspect, 1.), 4 * model_size)
    opacity = 1 - smoothstep((abs(focus[2]) / max(height, model_size) - 2) / 4)
    level = math.log2(width / 12)
    spacing = 2 ** math.floor(level)
    blend = smoothstep(level - math.floor(level))
    center = focus.copy()
    ray = focus - eye
    # Project towards the ground only for a well-conditioned intersection.
    if abs(ray[2]) > .15 * max(distance, 1e-12):
        factor = -eye[2] / ray[2]
        if factor > 0:
            center = eye + factor * ray
    center[2] = 0.
    return GridLayout(center, spacing, blend, opacity)


def line_grid(radius=24):
    """Short polylines with faded borders; no tessellated plane or duplicate edges."""
    offsets = np.arange(-radius, radius + 1, dtype=float)
    along = np.array([-1., -.75, -.5, 0., .5, .75, 1.]) * radius
    points = np.zeros((2, len(offsets), len(along), 3))
    points[0, :, :, 0] = offsets[:, None]
    points[0, :, :, 1] = along
    points[1, :, :, 1] = offsets[:, None]
    points[1, :, :, 0] = along
    points = points.reshape(-1, 3)
    count = 2 * len(offsets)
    lines = np.column_stack((np.full(count, len(along)), np.arange(len(points)).reshape(count, -1)))
    mesh = pv.PolyData(points, lines=lines.ravel())
    distance = np.max(np.abs(points[:, :2]), axis=1) / radius
    fade = np.clip((1 - distance) * 2, 0., 1.)
    fade = fade * fade * (3 - 2 * fade)
    mesh['rgba'] = np.column_stack((np.tile([97, 119, 142], (len(points), 1)),
                                    np.round(fade * 255))).astype(np.uint8)
    return mesh


class AdaptiveGrid:
    RADIUS = 24

    def __init__(self, plotter, model):
        self.plotter = plotter
        self.model_size = max(2 * (model['arm_m'] + model['rotor_radius_m']),
                              model['body_length_m'], model['body_width_m'])
        self.mesh = line_grid(self.RADIUS)
        self.actors = [plotter.add_mesh(self.mesh, scalars='rgba', rgb=True,
                                       lighting=False, line_width=1, opacity=.32,
                                       pickable=False, reset_camera=False)
                       for _ in range(2)]
        for actor in self.actors:
            actor.SetUseBounds(False)
        self._camera_state = None

    def update(self):
        camera = self.plotter.camera
        width, height = self.plotter.window_size
        state = (camera.position, camera.focal_point, camera.parallel_projection,
                 camera.parallel_scale, camera.view_angle, width, height)
        if state != self._camera_state:
            layout = grid_layout(camera, self.model_size, width / max(height, 1))
            for actor, spacing, weight in zip(self.actors,
                                             (layout.spacing, layout.spacing * 2),
                                             (1 - layout.blend, layout.blend)):
                opacity = .32 * layout.opacity * weight
                actor.SetVisibility(opacity > 0)
                if opacity > 0:
                    center = np.round(layout.center[:2] / spacing) * spacing
                    actor.SetScale(spacing, spacing, 1)
                    actor.SetPosition(float(center[0]), float(center[1]), 0)
                    actor.GetProperty().SetOpacity(opacity)
            self._camera_state = state

        # Keep visible ground inside clipping without including the large
        # reference patches in automatic camera fitting. This also runs when
        # interaction resets clipping while the camera pose stays unchanged.
        eye = np.asarray(camera.position)
        view = np.asarray(camera.focal_point) - eye
        distance = np.linalg.norm(view)
        if distance > 0:
            view /= distance
            near, far = camera.clipping_range
            for actor in self.actors:
                if actor.GetVisibility():
                    span = self.RADIUS * actor.GetScale()[0]
                    far_grid = (np.dot(np.asarray(actor.GetPosition()) - eye, view) +
                                span * (abs(view[0]) + abs(view[1])))
                    far = max(far, far_grid * 1.05)
            clip = (min(near, max(1e-4, distance * .01)), far)
            if clip != camera.clipping_range:
                camera.clipping_range = clip
