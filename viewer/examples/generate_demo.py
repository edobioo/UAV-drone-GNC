#!/usr/bin/env python3
"""Generate deterministic test CSVs using only the Python standard library."""
import argparse
import csv
import math
from pathlib import Path

# Same names and ordering as the simulator log; vectors remain optional.
HEADER = ['Time', 'Position_ENU[0]', 'Position_ENU[1]', 'Position_ENU[2]',
          'Quaternion_ENU_body[0]', 'Quaternion_ENU_body[1]',
          'Quaternion_ENU_body[2]', 'Quaternion_ENU_body[3]']
VELOCITY_HEADER = [f'Velocity_ENU[{i}]' for i in range(3)]
OMEGA_HEADER = [f'Omega_body[{i}]' for i in range(3)]



def quaternion(roll=0., pitch=0., yaw=0.):
    # Hamilton q = qz(yaw) * qy(pitch) * qx(roll); maps body vectors into ENU.
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return [cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy]


def sample(name, t):
    p, q = [0., 0., 1.], quaternion()
    vectors = None
    if name == 'translation':
        p = [min(t, 4.) * .5, min(max(t - 4., 0), 4.) * .5, 1 + max(t - 8., 0) * .4]
    elif name in ('yaw90', 'roll90', 'pitch90'):
        angle = math.pi / 2 * min(t / 4., 1.)
        q = quaternion(**{name[:-2]: angle})
    elif name == 'q_sign':
        q = quaternion(yaw=.4 * t)
        if round(t * 20) % 2:
            q = [-value for value in q]
    elif name in ('circle', 'flight'):
        omega = 2 * math.pi / 20
        theta = omega * t
        roll = .18 * math.sin(.7 * t) if name == 'flight' else 0.
        pitch = .12 * math.sin(.4 * t) if name == 'flight' else 0.
        roll_dot = .126 * math.cos(.7 * t) if name == 'flight' else 0.
        pitch_dot = .048 * math.cos(.4 * t) if name == 'flight' else 0.
        p = [2 * math.cos(theta), 2 * math.sin(theta), 1.3 + .35 * math.sin(2 * theta)]
        q = quaternion(roll, pitch, theta + math.pi / 2)
        velocity = [-2 * omega * math.sin(theta), 2 * omega * math.cos(theta), .7 * omega * math.cos(2 * theta)]
        body_rates = [roll_dot - omega * math.sin(pitch),
                      pitch_dot * math.cos(roll) + omega * math.sin(roll) * math.cos(pitch),
                      -pitch_dot * math.sin(roll) + omega * math.cos(roll) * math.cos(pitch)]
        vectors = velocity + body_rates
    elif name == 'gap':
        p = [.2 * t, 0., 1.]
    return [t, *p, *q] + ([] if vectors is None else vectors)


def generate(directory):
    directory.mkdir(parents=True, exist_ok=True)
    for name, duration in [('hover', 10), ('translation', 12), ('yaw90', 6), ('roll90', 6),
                           ('pitch90', 6), ('q_sign', 10), ('circle', 20), ('flight', 30), ('gap', 10)]:
        path = directory / f'demo_{name}.csv'
        # Exclusive creation: never overwrite a user's simulation or previous demo.
        if path.exists():
            print(f'Esistente, conservato: {path}')
            continue
        with path.open('x', newline='') as stream:
            writer = csv.writer(stream)
            has_vectors = name in ('circle', 'flight')
            writer.writerow(HEADER[:4] + VELOCITY_HEADER + HEADER[4:] + OMEGA_HEADER
                            if has_vectors else HEADER)
            for i in range(duration * 20 + 1):
                t = i / 20
                if name == 'gap' and 3 < t < 6:
                    continue
                row = sample(name, t)
                if has_vectors:
                    row = row[:4] + row[8:11] + row[4:8] + row[11:14]
                writer.writerow([f'{v:.12g}' for v in row])
        print(f'Creato: {path}')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, default=Path(__file__).resolve().parents[2] / 'outputs')
    args = parser.parse_args()
    generate(args.output_dir)
