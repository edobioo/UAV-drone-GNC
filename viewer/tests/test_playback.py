from types import SimpleNamespace
import numpy as np
import pytest
from uav_viewer.playback import Playback


class Clock:
    def __init__(self):
        self.now = 100.

    def __call__(self):
        return self.now


def make_player(times=(10., 11., 12., 13.), gaps=()):
    times = np.asarray(times)
    flight = SimpleNamespace(times=times, start=times[0], end=times[-1],
                             duration=times[-1] - times[0], gaps=np.asarray(gaps, dtype=int),
                             safe_time=lambda time, allow_gaps: float(np.clip(time, times[0], times[-1])))
    clock = Clock()
    return Playback(flight, clock), clock


def test_delayed_callback_uses_elapsed_time():
    player, clock = make_player()
    player.play()
    clock.now += 2.4
    assert player.tick() == pytest.approx(12.4)


def test_pause_resume_and_speed_change():
    player, clock = make_player()
    player.play()
    clock.now += .5
    player.set_speed(2)
    clock.now += .5
    assert player.tick() == pytest.approx(11.5)
    player.pause()
    clock.now += 20
    assert player.tick() == pytest.approx(11.5)
    player.play()
    clock.now += .25
    assert player.tick() == pytest.approx(12)


def test_seek_reanchors_playback():
    player, clock = make_player()
    player.play()
    clock.now += 1
    player.seek(10.2)
    clock.now += .1
    assert player.tick() == pytest.approx(10.3)


def test_stops_at_exact_last_sample_and_restarts():
    player, clock = make_player()
    player.play()
    clock.now += 10
    assert player.tick() == 13
    assert not player.playing
    player.play()
    assert player.time == 10


def test_loop_and_single_sample():
    player, clock = make_player()
    player.loop = True
    player.play()
    clock.now += 4
    assert player.tick() == 10
    assert player.playing
    static, _ = make_player((5.,))
    static.play()
    assert not static.playing
    assert static.tick() == 5


def test_gap_is_not_skipped_even_by_late_callback():
    player, clock = make_player((0., .1, 3., 3.1), (1,))
    player.play()
    clock.now += 10
    assert player.tick() == .1
    assert player.blocked and not player.playing
    player.allow_gaps = True
    player.play()
    clock.now += 1
    assert player.tick() == pytest.approx(1.1)


def test_step_can_cross_gap_without_interpolating():
    player, _ = make_player((0., .1, 3., 3.1), (1,))
    player.seek(.1)
    player.step(1)
    assert player.time == 3
    player.step(-1)
    assert player.time == .1
    player.step(-1)
    player.step(-1)
    assert player.time == 0


@pytest.mark.parametrize('speed', [0, -1, float('inf'), float('nan')])
def test_invalid_speed(speed):
    player, _ = make_player()
    with pytest.raises(ValueError):
        player.set_speed(speed)
