import csv
import numpy as np
import pytest

pytest.importorskip('scipy')
from uav_viewer.config import load_config
from uav_viewer.data import DataError, GapError, load_csv

HEADER = ['Time', 'Position_ENU[0]', 'Position_ENU[1]', 'Position_ENU[2]',
          'Quaternion_ENU_body[0]', 'Quaternion_ENU_body[1]',
          'Quaternion_ENU_body[2]', 'Quaternion_ENU_body[3]']


def write(tmp_path, rows, header=HEADER):
    path = tmp_path / 'flight.csv'
    with path.open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(header)
        writer.writerows(rows)
    return path


def test_identity_translation_and_nonuniform_times(tmp_path):
    rows = [[5, 0, 0, 0, 1, 0, 0, 0], [5.2, 2, 4, 6, 1, 0, 0, 0], [5.9, 3, 5, 7, 1, 0, 0, 0]]
    flight = load_csv(write(tmp_path, rows), load_config())
    np.testing.assert_allclose(flight.sample(5.1).position, [1, 2, 3])
    np.testing.assert_allclose(flight.sample(5.1).rotation.as_matrix(), np.eye(3))
    for row in rows:
        np.testing.assert_allclose(flight.sample(row[0]).position, row[1:4])
    assert flight.sample(0).time == 5
    assert flight.sample(100).time == 5.9


@pytest.mark.parametrize('axis,expected', [(0, [0, 0, 1]), (1, [1, 0, 0]), (2, [0, 1, 0])])
def test_positive_90_degree_rotations(tmp_path, axis, expected):
    q = [2 ** -.5, 0, 0, 0]
    q[axis + 1] = 2 ** -.5
    flight = load_csv(write(tmp_path, [[0, 3, 4, 5, *q]]), load_config())
    basis = np.eye(3)[(axis + 1) % 3]
    np.testing.assert_allclose(flight.sample(0).rotation.apply(basis), expected, atol=1e-12)
    np.testing.assert_allclose(flight.sample(0).matrix @ [0, 0, 0, 1], [3, 4, 5, 1])


def test_slerp_midpoint_and_sign_equivalence(tmp_path):
    r = 2 ** -.5
    flight = load_csv(write(tmp_path, [[0, 0, 0, 1, 1, 0, 0, 0], [1, 0, 0, 1, -r, 0, 0, -r]]), load_config())
    np.testing.assert_allclose(flight.sample(.5).rotation.apply([1, 0, 0]), [r, r, 0], atol=1e-12)
    signed = load_csv(write(tmp_path, [[0, 0, 0, 1, r, 0, 0, r], [1, 0, 0, 1, -r, 0, 0, -r]]), load_config())
    np.testing.assert_allclose(signed.sample(.5).rotation.as_matrix(), signed.sample(0).rotation.as_matrix(), atol=1e-12)


def test_inverse_direction_is_explicit(tmp_path):
    r = 2 ** -.5
    config = load_config()
    config['data']['quaternion_direction'] = 'enu_to_body'
    flight = load_csv(write(tmp_path, [[0, 0, 0, 0, r, 0, 0, r]]), config)
    np.testing.assert_allclose(flight.sample(0).rotation.apply([1, 0, 0]), [0, -1, 0], atol=1e-12)


def test_gaps_and_single_static_sample(tmp_path):
    with pytest.warns(UserWarning, match='lacune'):
        flight = load_csv(write(tmp_path, [[0, 0, 0, 0, 1, 0, 0, 0], [3, 3, 0, 0, 1, 0, 0, 0]]), load_config())
    with pytest.raises(GapError):
        flight.sample(1)
    assert flight.safe_time(1) == 0
    assert flight.safe_time(3) == 3
    np.testing.assert_allclose(flight.sample(1, allow_gaps=True).position, [1, 0, 0])
    single = load_csv(write(tmp_path, [[5, 1, 2, 3, 1, 0, 0, 0]]), load_config())
    assert single.sample(100).time == 5


def test_optional_vectors(tmp_path):
    config = load_config()
    names = config['columns']['velocity'] + config['columns']['angular_velocity']
    flight = load_csv(write(tmp_path, [[0, 0, 0, 0, 1, 0, 0, 0, 1, 2, 3, 4, 5, 6]], HEADER + names), config)
    np.testing.assert_allclose(flight.sample(0).velocity, [1, 2, 3])
    np.testing.assert_allclose(flight.sample(0).angular_velocity, [4, 5, 6])


def test_simulator_column_order(tmp_path):
    # Velocity precedes the quaternion in the official log. Distinct values catch
    # positional parsing errors; yaw +90 verifies w,x,y,z and body -> ENU.
    header = ['Time', 'Position_ENU[0]', 'Position_ENU[1]', 'Position_ENU[2]',
              'Velocity_ENU[0]', 'Velocity_ENU[1]', 'Velocity_ENU[2]',
              'Quaternion_ENU_body[0]', 'Quaternion_ENU_body[1]',
              'Quaternion_ENU_body[2]', 'Quaternion_ENU_body[3]',
              'Omega_body[0]', 'Omega_body[1]', 'Omega_body[2]']
    r = 2 ** -.5
    flight = load_csv(write(tmp_path, [[0, 10, 20, 30, 1, 2, 3, r, 0, 0, r, 4, 5, 6]], header), load_config())
    pose = flight.sample(0)
    np.testing.assert_allclose(pose.position, [10, 20, 30])
    np.testing.assert_allclose(pose.velocity, [1, 2, 3])
    np.testing.assert_allclose(pose.angular_velocity, [4, 5, 6])
    np.testing.assert_allclose(pose.rotation.apply([1, 0, 0]), [0, 1, 0], atol=1e-12)


@pytest.mark.parametrize('rows,match', [
    ([], 'campioni'),
    ([[0, 0, 0, 0, 0, 0, 0, 0]], 'degenere'),
    ([[0, 'nan', 0, 0, 1, 0, 0, 0]], 'NaN'),
    ([[0, 'bad', 0, 0, 1, 0, 0, 0]], 'numerico'),
    ([[0, 0, 0]], 'campi'),
    ([[0, 0, 0, 0, 1, 0, 0, 0, 7]], 'campi'),
    ([[0, 0, 0, 0, 1, 0, 0, 0], [0, 1, 0, 0, 1, 0, 0, 0]], 'crescenti'),
    ([[1, 0, 0, 0, 1, 0, 0, 0], [0, 1, 0, 0, 1, 0, 0, 0]], 'crescenti'),
])
def test_invalid_rows(tmp_path, rows, match):
    with pytest.raises(DataError, match=match):
        load_csv(write(tmp_path, rows), load_config())


def test_missing_and_partial_headers(tmp_path):
    with pytest.raises(DataError, match='mancanti'):
        load_csv(write(tmp_path, [], ['Time']), load_config())
    with pytest.raises(DataError, match='incompleto'):
        load_csv(write(tmp_path, [], HEADER + ['Velocity_ENU[0]']), load_config())


def test_normalization_warning(tmp_path):
    with pytest.warns(UserWarning, match='normalizzazione'):
        flight = load_csv(write(tmp_path, [[0, 0, 0, 0, 2, 0, 0, 0]]), load_config())
    np.testing.assert_allclose(flight.sample(0).rotation.as_matrix(), np.eye(3))
