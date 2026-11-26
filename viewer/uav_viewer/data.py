"""CSV validation and interpolation, independent of Qt/VTK."""
from dataclasses import dataclass
from pathlib import Path
import csv
import warnings
import numpy as np
from scipy.spatial.transform import Rotation, Slerp


class DataError(ValueError):
    pass


class GapError(DataError):
    pass


@dataclass(frozen=True)
class Pose:
    time: float
    position: np.ndarray
    rotation: Rotation
    velocity: np.ndarray | None = None
    angular_velocity: np.ndarray | None = None

    @property
    def matrix(self):
        transform = np.eye(4)
        transform[:3, :3] = self.rotation.as_matrix()
        transform[:3, 3] = self.position
        return transform

    @property
    def quaternion_wxyz(self):
        return self.rotation.as_quat()[[3, 0, 1, 2]]


class FlightData:
    def __init__(self, times, positions, rotations, velocity=None, angular_velocity=None, gap_threshold=1.0):
        self.times = np.asarray(times, dtype=float)
        self.positions = np.asarray(positions, dtype=float)
        self.rotations = rotations
        self.velocity = velocity
        self.angular_velocity = angular_velocity
        self.gaps = np.flatnonzero(np.diff(self.times) > gap_threshold)
        self._slerp = Slerp(self.times, rotations) if len(self.times) > 1 else None

    @property
    def start(self):
        return float(self.times[0])

    @property
    def end(self):
        return float(self.times[-1])

    @property
    def duration(self):
        return self.end - self.start

    def safe_time(self, time, allow_gaps=False):
        time = float(np.clip(time, self.start, self.end))
        if not allow_gaps:
            index = int(np.searchsorted(self.times, time, side='right') - 1)
            if index in self.gaps and self.times[index] < time < self.times[index + 1]:
                return float(self.times[index])
        return time

    def sample(self, time, allow_gaps=False):
        if not np.isfinite(time):
            raise DataError('Tempo richiesto non finito')
        time = float(np.clip(time, self.start, self.end))
        if self.safe_time(time, allow_gaps) != time:
            raise GapError('Interpolazione in una lacuna: abilitarla esplicitamente')
        left = int(np.searchsorted(self.times, time, side='right') - 1)
        right = min(left + 1, len(self.times) - 1)
        factor = 0.0 if left == right else (time - self.times[left]) / (self.times[right] - self.times[left])

        def interpolate(values):
            return None if values is None else (1 - factor) * values[left] + factor * values[right]

        rotation = self.rotations[0] if self._slerp is None else self._slerp(time)
        return Pose(time, interpolate(self.positions), rotation, interpolate(self.velocity), interpolate(self.angular_velocity))


def load_csv(path, config):
    columns = config['columns']
    required = [columns['time'], *columns['position'], *columns['quaternion']]
    with Path(path).open('r', encoding='utf-8-sig', newline='') as stream:
        reader = csv.DictReader(stream)
        header = reader.fieldnames
        if not header or len(set(header)) != len(header):
            raise DataError('Intestazione assente o con colonne duplicate')
        missing = set(required) - set(header)
        if missing:
            raise DataError(f'Colonne obbligatorie mancanti: {", ".join(sorted(missing))}')
        optional = []
        for key in ('velocity', 'angular_velocity'):
            present = set(columns[key]) & set(header)
            if present and len(present) != 3:
                raise DataError(f'Vettore {key} incompleto: servono tutte le tre componenti')
            if present:
                optional.append(key)
        names = required + sum((columns[k] for k in optional), [])
        rows, line_numbers = [], []
        for row in reader:
            line = reader.line_num
            if None in row or any(value is None for value in row.values()):
                raise DataError(f'Riga {line}: numero di campi incoerente con intestazione')
            try:
                values = [float(row[name]) for name in names]
            except (ValueError, TypeError) as exc:
                raise DataError(f'Riga {line}: valore numerico non valido') from exc
            if not np.all(np.isfinite(values)):
                raise DataError(f'Riga {line}: NaN o infinito')
            rows.append(values)
            line_numbers.append(line)
    if not rows:
        raise DataError('Il CSV non contiene campioni')
    array = np.asarray(rows)
    times = array[:, 0]
    invalid = np.flatnonzero(np.diff(times) <= 0)
    if len(invalid):
        raise DataError(f'Riga {line_numbers[invalid[0] + 1]}: tempi non strettamente crescenti')
    wxyz = array[:, 4:8]
    scale = np.max(np.abs(wxyz), axis=1)
    if np.any(scale < 1e-12):
        index = int(np.flatnonzero(scale < 1e-12)[0])
        raise DataError(f'Riga {line_numbers[index]}: quaternion degenere')
    scaled = wxyz / scale[:, None]
    norm_scaled = np.linalg.norm(scaled, axis=1)
    with np.errstate(over='ignore'):
        norm = scale * norm_scaled
    if np.any(np.abs(norm - 1) > 0.01):
        warnings.warn('Quaternioni con norma distante da 1: normalizzazione applicata', stacklevel=2)
    wxyz = scaled / norm_scaled[:, None]
    # q e -q rappresentano la stessa posa: mantieni continuità di segno nel file.
    for index in range(1, len(wxyz)):
        if np.dot(wxyz[index - 1], wxyz[index]) < 0:
            wxyz[index] *= -1
    rotations = Rotation.from_quat(wxyz[:, [1, 2, 3, 0]])
    if config['data']['quaternion_direction'] == 'enu_to_body':
        rotations = rotations.inv()
    vectors = dict(velocity=None, angular_velocity=None)
    offset = 8
    for key in optional:
        vectors[key] = array[:, offset:offset + 3]
        offset += 3
    flight = FlightData(times, array[:, 1:4], rotations, **vectors,
                        gap_threshold=config['data']['gap_threshold_s'])
    if len(flight.gaps):
        warnings.warn(f'{len(flight.gaps)} lacune temporali: playback bloccato ai bordi per default', stacklevel=2)
    return flight
