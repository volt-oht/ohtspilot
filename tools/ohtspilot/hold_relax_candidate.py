"""Offline-only hold experiment. Not imported by the driving controller.

No grade or pressure model: these trial constants are NOT validated settings.
"""
import math


class HoldRelaxCandidate:
  SPEED_MAX = .02
  DWELL = 1.0
  RATE = .1
  MAX_CHANGE = .3

  def __init__(self):
    self.reset()

  def reset(self):
    self.elapsed = 0.
    self.anchor = None
    self.output = None
    self.target = None
    self.blocked = False

  def update(self, command, target, dt, *, stopping, active, standstill,
             gas, brake, v_ego, v_raw, wheels, valid=True):
    # State transitions and starting commands belong to the original controller.
    if not stopping or not active:
      self.reset()
      return command
    sane = (valid and len(wheels) == 4 and
            all(math.isfinite(v) for v in (command, target, dt, v_ego, v_raw, *wheels)) and
            0. < dt <= .05 and target < 0.)
    stationary = (sane and standstill and not gas and not brake and
                  max(abs(v) for v in (v_ego, v_raw, *wheels)) <= self.SPEED_MAX)
    if sane and command > target:
      # Do not hold back an original release/start request, even if a recorded
      # state label lags the command by one cycle.
      self.reset()
      return command
    if self.anchor is not None:
      if (not stationary or target != self.target or
          (math.isfinite(command) and command < self.anchor - 1e-6)):
        self.blocked = True
      if self.blocked:
        # Restore the previous hold, never weaken a newly stronger brake request.
        return min(command, self.anchor) if math.isfinite(command) else self.anchor
      self.output = min(target, self.output + self.RATE * dt)
      return self.output
    if not stationary or not (target - self.MAX_CHANGE <= command < target):
      self.elapsed = 0.
      return command
    self.elapsed += dt
    if self.elapsed + 1e-9 < self.DWELL:
      return command
    self.anchor = self.output = command
    self.target = target
    return command
