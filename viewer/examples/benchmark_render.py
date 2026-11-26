#!/usr/bin/env python3
"""Repeatable offscreen frame timings; no simulator or network required."""
import argparse
import json
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np
import pyvista as pv
from scipy.spatial.transform import Rotation
from uav_viewer.config import load_config
from uav_viewer.data import FlightData, load_csv
from uav_viewer.scene import FlightScene


def scenarios(csv_path=None):
    times = np.linspace(0, 100, 20001)
    zero = np.zeros_like(times)
    rotations = Rotation.from_rotvec(np.column_stack((.1 * np.sin(times), zero, .03 * times)))
    paths = {
        'ground': np.column_stack((np.cos(times / 10), np.sin(times / 10), zero + 1)),
        'vertical': np.column_stack((zero, zero, times ** 2 * .1)),
        'wide': np.column_stack((times * 100, 100 * np.sin(times), times * 10)),
        'descent': np.column_stack((zero, zero, -times ** 2 * .1)),
    }
    for name, positions in paths.items():
        velocity = np.gradient(positions, times, axis=0)
        flight = FlightData(times, positions, rotations, velocity, np.zeros_like(positions))
        yield name, flight, 'follow'
        if name == 'wide':
            yield 'overview', flight, 'free'
            yield 'attitude', flight, 'attitude'
    if csv_path:
        yield 'csv', load_csv(csv_path, load_config()), 'follow'


def benchmark(flight, mode, frames, size):
    config = load_config()
    config['display']['camera'] = mode
    plotter = pv.Plotter(off_screen=True, window_size=size)
    try:
        scene = FlightScene(plotter, flight, config)
        plotter.show(auto_close=False, interactive=False)
        records = []
        for frame in range(frames + 5):
            instant = flight.start + flight.duration * .8 + (frame / 60)
            start = time.perf_counter()
            scene.update(flight.sample(min(instant, flight.end), allow_gaps=True))
            middle = time.perf_counter()
            plotter.render()
            end = time.perf_counter()
            if frame >= 5:
                records.append(((middle - start) * 1000, (end - middle) * 1000))
        samples = np.asarray(records)
        return {'update_ms': round(float(np.median(samples[:, 0])), 2),
                'render_ms': round(float(np.median(samples[:, 1])), 2),
                'frame_ms': round(float(np.median(samples.sum(axis=1))), 2),
                'p95_ms': round(float(np.percentile(samples.sum(axis=1), 95)), 2)}
    finally:
        if 'scene' in locals():
            scene.close()
        plotter.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--csv', type=Path)
    parser.add_argument('--frames', type=int, default=30)
    parser.add_argument('--width', type=int, default=1000)
    parser.add_argument('--height', type=int, default=700)
    args = parser.parse_args()
    if args.frames < 1 or min(args.width, args.height) < 1:
        parser.error('Frame e dimensioni devono essere positivi')
    results = {name: benchmark(flight, mode, args.frames, (args.width, args.height))
               for name, flight, mode in scenarios(args.csv)}
    print(json.dumps(results, indent=2))


if __name__ == '__main__':
    main()
