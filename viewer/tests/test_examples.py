import csv
import importlib.util
import math
import warnings
from pathlib import Path
import pytest

path = Path(__file__).resolve().parents[1] / 'examples' / 'generate_demo.py'
spec = importlib.util.spec_from_file_location('demo', path)
demo = importlib.util.module_from_spec(spec)
spec.loader.exec_module(demo)


def test_generator_preserves_files_and_produces_valid_data(tmp_path):
    demo.generate(tmp_path)
    files = list(tmp_path.glob('demo_*.csv'))
    assert len(files) == 9
    for path in files:
        with path.open() as stream:
            rows = list(csv.DictReader(stream))
        times = [float(row['Time']) for row in rows]
        assert all(b > a for a, b in zip(times, times[1:]))
        for row in rows:
            values = [float(v) for v in row.values()]
            assert all(math.isfinite(v) for v in values)
            assert sum(float(row[k]) ** 2 for k in [f'Quaternion_ENU_body[{i}]' for i in range(4)]) == pytest.approx(1, abs=2e-11)
    preserved = files[0]
    preserved.write_text('preserve me')
    demo.generate(tmp_path)
    assert preserved.read_text() == 'preserve me'


@pytest.mark.parametrize('name,axis', [('roll90', 1), ('pitch90', 2), ('yaw90', 3)])
def test_analytic_quaternion_examples(name, axis):
    row = demo.sample(name, 4)
    expected = [2 ** -.5, 0, 0, 0]
    expected[axis] = 2 ** -.5
    assert row[4:8] == pytest.approx(expected)


def test_csv_examples_with_real_loader(demo_dir):
    pytest.importorskip('scipy')
    from uav_viewer.config import load_config
    from uav_viewer.data import load_csv
    config = load_config()
    for path in demo_dir.glob('demo_*.csv'):
        with warnings.catch_warnings():
            warnings.simplefilter('ignore')
            flight = load_csv(path, config)
        for time in flight.times[::50]:
            flight.sample(time)
        if path.stem in ('demo_circle', 'demo_flight'):
            expected = demo.sample(path.stem.removeprefix('demo_'), 2.5)
            pose = flight.sample(2.5)
            assert pose.position == pytest.approx(expected[1:4])
            assert pose.quaternion_wxyz == pytest.approx(expected[4:8])
            assert pose.velocity == pytest.approx(expected[8:11])
            assert pose.angular_velocity == pytest.approx(expected[11:14])


@pytest.mark.parametrize('time', [0.5, 4., 9., 21.])
def test_demo_body_rates_match_quaternion_derivative(time):
    # Independent finite difference: Hamilton qdot = 0.5 * q * [0, omega_body].
    h = 1e-5
    row = demo.sample('flight', time)
    derivative = [(b - a) / (2 * h) for a, b in zip(
        demo.sample('flight', time - h)[4:8], demo.sample('flight', time + h)[4:8])]
    w, x, y, z = row[4:8]
    a, b, c = row[-3:]
    expected = [-(x*a + y*b + z*c)/2, (w*a + y*c - z*b)/2,
                (w*b + z*a - x*c)/2, (w*c + x*b - y*a)/2]
    assert derivative == pytest.approx(expected, abs=1e-9)
