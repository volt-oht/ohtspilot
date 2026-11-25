import math
import pathlib
import sys
import unittest
from types import SimpleNamespace as NS

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]))
from selfdrive.controls.lib.lead_catchup import catchup_accel_limit


def lead(**changes):
  values = dict(status=True, radar=True, dRel=40., vRel=2., vLead=12., aLeadK=0.5)
  values.update(changes)
  return NS(**values)


class CatchupTests(unittest.TestCase):
  def scale(self, **changes):
    args = dict(base_accel=0.7, enabled=True, active=True, v_ego=10., a_ego=0.2,
                t_follow=1.8, stop_distance=4., leads=(lead(), lead(status=False)))
    args.update(changes)
    return catchup_accel_limit(**args)

  def test_disabled_and_inactive_are_exact_baseline(self):
    self.assertEqual(self.scale(enabled=False), .7)
    self.assertEqual(self.scale(active=False), .7)

  def test_moving_away_with_space_has_bounded_allowance(self):
    self.assertEqual(self.scale(), 1.0)
    self.assertAlmostEqual(self.scale(leads=(lead(dRel=25.),)), .75)
    self.assertEqual(self.scale(base_accel=1.3), 1.4)
    self.assertEqual(self.scale(base_accel=0.), 0.)

  def test_slower_braking_close_or_vision_lead_blocks_reduction(self):
    for changes in [dict(vRel=0), dict(vLead=9), dict(aLeadK=-.01),
                    dict(dRel=24), dict(radar=False)]:
      with self.subTest(changes=changes):
        self.assertEqual(self.scale(leads=(lead(**changes),)), .7)

  def test_second_lead_can_veto_first(self):
    self.assertEqual(self.scale(leads=(lead(), lead(dRel=15.))), .7)
    self.assertEqual(self.scale(leads=(lead(), lead(vRel=-1.))), .7)

  def test_stop_and_deceleration_retain_baseline(self):
    for speed in [0., .4, 2., 30.]:
      self.assertEqual(self.scale(v_ego=speed), .7)
    self.assertEqual(self.scale(a_ego=-.01), .7)

  def test_loss_and_reacquisition_do_not_accumulate(self):
    for _ in range(100):
      self.assertEqual(self.scale(), 1.0)
      self.assertEqual(self.scale(leads=(lead(status=False), lead())), .7)
      self.assertEqual(self.scale(leads=()), .7)

  def test_nonfinite_and_invalid_parameters_fail_to_baseline(self):
    for value in [math.nan, math.inf, -math.inf]:
      for name in ['v_ego', 'a_ego', 't_follow', 'stop_distance']:
        self.assertEqual(self.scale(**{name: value}), .7)
      for name in ['dRel', 'vRel', 'vLead', 'aLeadK']:
        self.assertEqual(self.scale(leads=(lead(**{name: value}),)), .7)
    for follow in [0., -1., 3.]:
      self.assertEqual(self.scale(t_follow=follow), .7)
    self.assertEqual(self.scale(stop_distance=3.), .7)

  def test_grid_stays_in_bounds(self):
    for gap in [0., 10., 24., 25., 30., 80.]:
      for relative in [-10., 0., .5, .51, 1., 2., 10.]:
        scale = self.scale(leads=(lead(dRel=gap, vRel=relative, vLead=10+relative),))
        self.assertTrue(.7 <= scale <= 1.)


if __name__ == '__main__':
  unittest.main()
