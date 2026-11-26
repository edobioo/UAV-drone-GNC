"""Configuration resolved relative to the viewer, never the working directory."""
from copy import deepcopy
from pathlib import Path
import math
import tomllib

DEFAULT_PATH = Path(__file__).resolve().parents[1] / 'viewer.toml'


def load_config(path=None):
    with DEFAULT_PATH.open('rb') as stream:
        config = tomllib.load(stream)
    if path is not None and Path(path).resolve() != DEFAULT_PATH:
        with Path(path).open('rb') as stream:
            override = tomllib.load(stream)
        for section, values in override.items():
            if section not in config or not isinstance(values, dict):
                raise ValueError(f'Sezione configurazione sconosciuta: {section}')
            unknown = values.keys() - config[section].keys()
            if unknown:
                raise ValueError(f'Opzioni sconosciute in {section}: {sorted(unknown)}')
            config[section].update(values)
    validate_config(config)
    return deepcopy(config)


def validate_config(config):
    for section in ('model', 'display', 'data'):
        for key, value in config[section].items():
            if key in ('camera', 'quaternion_direction'):
                continue
            if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or value <= 0:
                raise ValueError(f'{section}.{key} deve essere un numero positivo finito')
    for key in ('fps', 'max_trail_points'):
        if not isinstance(config['display'][key], int):
            raise ValueError(f'display.{key} deve essere intero')
    if config['display']['max_trail_points'] < 2:
        raise ValueError('max_trail_points deve essere almeno 2')
    if config['display']['camera'] not in ('free', 'follow', 'attitude'):
        raise ValueError('Camera valida: free, follow, attitude')
    if config['data']['quaternion_direction'] not in ('body_to_enu', 'enu_to_body'):
        raise ValueError('quaternion_direction: body_to_enu oppure enu_to_body')
    columns = config['columns']
    if not isinstance(columns['time'], str) or not columns['time']:
        raise ValueError('columns.time deve essere un nome di colonna')
    for key, length in [('position', 3), ('quaternion', 4), ('velocity', 3), ('angular_velocity', 3)]:
        names = columns[key]
        if not isinstance(names, list) or len(names) != length or any(not isinstance(n, str) or not n for n in names):
            raise ValueError(f'columns.{key}: attesi {length} nomi di colonna')
    names = [columns['time']] + sum((columns[k] for k in ('position', 'quaternion', 'velocity', 'angular_velocity')), [])
    if len(set(names)) != len(names):
        raise ValueError('La mappatura contiene nomi di colonna duplicati')
