import math
import unittest
from hold_relax_candidate import HoldRelaxCandidate


def step(h, **changes):
  args=dict(command=-2.24,target=-2.,dt=.01,stopping=True,active=True,
            standstill=True,gas=False,brake=False,v_ego=0.,v_raw=0.,wheels=[0.]*4)
  args.update(changes)
  return h.update(**args)


class HoldTests(unittest.TestCase):
  def relaxed(self):
    h=HoldRelaxCandidate()
    for _ in range(200):step(h)
    return h

  def test_dwell_and_rate_and_ceiling(self):
    h=HoldRelaxCandidate()
    for _ in range(100):self.assertEqual(step(h),-2.24)
    last=-2.24
    for _ in range(500):
      value=step(h);self.assertLessEqual(value-last,.001000001)
      self.assertLessEqual(value,-2.);self.assertGreaterEqual(value,last);last=value
    self.assertEqual(last,-2.)

  def test_final_stop_wheel_disagreement_and_reverse_motion(self):
    for changes in [dict(wheels=[0,0,.1,0]),dict(wheels=[0,0,-.1,0]),
                    dict(v_ego=.1),dict(v_raw=.1),dict(standstill=False)]:
      h=HoldRelaxCandidate()
      for _ in range(300):self.assertEqual(step(h,**changes),-2.24)

  def test_dropouts_restore_and_latch_for_this_stop(self):
    for changes in [dict(wheels=[.03,0,0,0]),dict(wheels=[0,0]),dict(valid=False),
                    dict(dt=.1),dict(dt=0.),dict(gas=True),dict(brake=True),
                    dict(wheels=[math.nan,0,0,0]),dict(v_ego=math.inf),
                    dict(standstill=False),dict(target=-1.9)]:
      h=self.relaxed()
      with self.subTest(changes=changes):
        self.assertEqual(step(h,**changes),-2.24)
        for _ in range(200):self.assertEqual(step(h),-2.24)

  def test_stronger_braking_is_never_reduced(self):
    h=self.relaxed();self.assertEqual(step(h,command=-2.6),-2.6)
    self.assertEqual(step(h),-2.24)

  def test_departure_and_deactivation_pass_through(self):
    for changes in [dict(stopping=False,command=2.),dict(active=False,command=0.)]:
      h=self.relaxed();self.assertEqual(step(h,**changes),changes['command'])
      self.assertIsNone(h.anchor)
      for _ in range(100):self.assertEqual(step(h),-2.24)

  def test_target_and_large_deviation_are_not_relaxed(self):
    for command in [-3.,-2.,-1.,0.,2.]:
      h=HoldRelaxCandidate()
      for _ in range(200):self.assertEqual(step(h,command=command),command)

  def test_lagging_stopping_label_does_not_override_departure(self):
    h=self.relaxed();self.assertEqual(step(h,command=2.),2.)
    self.assertIsNone(h.anchor)


if __name__=='__main__':unittest.main()
