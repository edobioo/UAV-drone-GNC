"""Procedural quadrotor: local coordinates are meters in the body frame."""
import numpy as np
import pyvista as pv

AXIS_COLORS = ('#f06c70', '#72d19b', '#68aaff')


def build_model(plotter, config):
    actors = []
    parts = {}

    def add(mesh, color, **kwargs):
        parts.setdefault((color, kwargs.get('opacity', 1.)), []).append(mesh)

    length, width, height = (config[f'body_{key}_m'] for key in ('length', 'width', 'height'))
    fuselage = pv.Sphere(theta_resolution=16, phi_resolution=12)
    fuselage.points *= np.array([length, width, height])  # sphere radius = 0.5
    add(fuselage, '#a5adb7')
    add(pv.Cube(center=(-0.015, 0, height * .32), x_length=length * .58,
                y_length=width * .72, z_length=height * .25), '#545e6b')
    # Recognizable front: gimbal housing and camera, along +X body.
    add(pv.Sphere(theta_resolution=12, phi_resolution=8, center=(length * .42, 0, -height * .25), radius=width * .23), '#363f4b')
    add(pv.Cylinder(resolution=16, center=(length * .54, 0, -height * .25), direction=(1, 0, 0),
                    radius=width * .15, height=width * .12), '#101c2c')
    add(pv.Disc(center=(length * .605, 0, -height * .25), normal=(1, 0, 0),
                inner=0, outer=width * .105, c_res=24), '#60b5db')
    a = config['arm_m'] / np.sqrt(2)
    motor_positions = [(a, a, 0), (-a, a, 0), (-a, -a, 0), (a, -a, 0)]
    for x, y, z in motor_positions:
        add(pv.Line((x * .2, y * .15, 0), (x, y, z)).tube(radius=width * .09, n_sides=8), '#737e8c')
        add(pv.Cylinder(resolution=16, center=(x, y, .008), direction=(0, 0, 1),
                        radius=width * .15, height=height * .45), '#343f4b')
        add(pv.Disc(center=(x, y, height * .37), normal=(0, 0, 1), inner=width * .1,
                    outer=config['rotor_radius_m'], c_res=24), '#a6c0d3', opacity=.24)
        add(pv.Sphere(theta_resolution=12, phi_resolution=8, center=(x, y, height * .37), radius=width * .09), '#d4dbe3')
        add(pv.Line((x * .85, y, -.005), (x * .85, y, -height * .65)).tube(radius=width * .045, n_sides=8), '#343f4b')
    add(pv.Sphere(theta_resolution=12, phi_resolution=8, radius=width * .065), '#ffd166')
    # Merge parts sharing a material once: fewer draw calls, same rigid pose.
    for (color, opacity), meshes in parts.items():
        mesh = pv.merge(meshes, merge_points=False) if len(meshes) > 1 else meshes[0]
        actors.append(plotter.add_mesh(mesh, color=color, opacity=opacity,
                                       reset_camera=False))
    axes = []
    for direction, color in zip(np.eye(3), AXIS_COLORS):
        actor = plotter.add_mesh(pv.Arrow(direction=direction, scale=config['axes_length_m'],
                                          tip_radius=.025, tip_length=.10, shaft_radius=.012,
                                          tip_resolution=8, shaft_resolution=8),
                                 color=color, reset_camera=False)
        actors.append(actor)
        axes.append(actor)
    return actors, axes
