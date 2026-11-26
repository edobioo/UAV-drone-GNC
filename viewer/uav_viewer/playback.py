"""Playback driven by elapsed monotonic time rather than rendered frames."""
import time
import numpy as np


class Playback:
    def __init__(self, flight, clock=time.monotonic):
        self.flight = flight
        self.clock = clock
        self.time = flight.start
        self.playing = False
        self.speed = 1.0
        self.loop = False
        self.allow_gaps = False
        self.blocked = False
        self._anchor_time = self.time
        self._anchor_wall = clock()

    def _anchor(self):
        self._anchor_time = self.time
        self._anchor_wall = self.clock()

    def play(self):
        if self.flight.duration == 0:
            return
        if self.time >= self.flight.end:
            self.time = self.flight.start
        self.blocked = False
        self._anchor()
        self.playing = True

    def pause(self):
        self.tick()
        self.playing = False

    def seek(self, time):
        if not np.isfinite(time):
            raise ValueError('Tempo non finito')
        self.time = self.flight.safe_time(time, self.allow_gaps)
        self.blocked = False
        self._anchor()

    def set_speed(self, speed):
        if not np.isfinite(speed) or speed <= 0:
            raise ValueError('Velocità di riproduzione non valida')
        self.tick()
        self.speed = speed
        self._anchor()

    def step(self, direction):
        self.playing = False
        side = 'right' if direction > 0 else 'left'
        index = np.searchsorted(self.flight.times, self.time, side=side)
        if direction < 0:
            index -= 1
        self.seek(self.flight.times[np.clip(index, 0, len(self.flight.times) - 1)])

    def tick(self):
        if not self.playing:
            return self.time
        target = self._anchor_time + (self.clock() - self._anchor_wall) * self.speed
        if not self.allow_gaps:
            for index in self.flight.gaps:
                left, right = self.flight.times[index:index + 2]
                if self.time <= left < target and right > self.time:
                    self.time = float(left)
                    self.blocked = True
                    self.playing = False
                    return self.time
        if target >= self.flight.end:
            self.time = self.flight.end
            if self.loop:
                self.time = self.flight.start
                self._anchor()
            else:
                self.playing = False
        else:
            self.time = target
        return self.time
