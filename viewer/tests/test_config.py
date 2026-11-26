import pytest
from uav_viewer.config import load_config


def test_default_conventions():
    config = load_config()
    assert config['columns']['quaternion'] == [f'Quaternion_ENU_body[{i}]' for i in range(4)]
    assert config['data']['quaternion_direction'] == 'body_to_enu'


def test_partial_override(tmp_path):
    path = tmp_path / 'config.toml'
    path.write_text('[model]\narm_m = 0.3\n')
    config = load_config(path)
    assert config['model']['arm_m'] == .3
    assert config['model']['body_length_m'] == .22


@pytest.mark.parametrize('text', [
    '[model]\narm_m = -1', '[model]\narm_m = nan',
    '[display]\nfps = 1.5', '[display]\nmax_trail_points = 1',
    '[data]\nquaternion_direction = "guess"', '[display]\ncamera = "other"',
    '[columns]\nquaternion = ["a", "b"]', '[columns]\ntime = "Position_ENU[0]"',
    '[columns]\nposition = "xyz"', '[other]\nx = 1', '[model]\narm = 1',
])
def test_invalid_configuration(tmp_path, text):
    path = tmp_path / 'bad.toml'
    path.write_text(text)
    with pytest.raises(ValueError):
        load_config(path)
