"""Execute the actual planner update with an input-recording MPC, not a solver."""
import ast
import pathlib
import sys
import unittest
from types import SimpleNamespace as NS
import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from common.numpy_fast import interp, clip
from selfdrive.controls.lib import lead_catchup

tree = ast.parse((ROOT/'selfdrive/controls/lib/longitudinal_planner.py').read_text())
update = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == 'update')
definitions = [n for n in tree.body if isinstance(n, ast.FunctionDef) and
               n.name in ('get_max_accel', 'limit_accel_in_turns')]
state = NS(off='off',pid='pid',stopping='stopping',starting='starting')
env = dict(np=np, math=__import__('math'), interp=interp, clip=clip, lead_catchup=lead_catchup,
  LongCtrlState=state, V_CRUISE_MAX=145., CV=NS(KPH_TO_MS=1/3.6,MS_TO_KPH=3.6,DEG_TO_RAD=np.pi/180),
  DT_MDL=.05, A_CRUISE_MIN=-1.2, AWARENESS_DECEL=-.2,
  A_CRUISE_MAX_BP=[0.,13.9,19.4,33.], A_CRUISE_MAX_VALS=[1.4,.7,.5,.3],
  _A_TOTAL_MAX_BP=[0.,25.,55.], _A_TOTAL_MAX_V=[3.5,4.,5.],
  T_IDXS=np.linspace(0,10,33), T_IDXS_MPC=np.linspace(0,10,13), CONTROL_N=17)
exec(compile(ast.Module(body=definitions+[update],type_ignores=[]),'actual_planner_update','exec'),env)


class SM(dict):
  healthy = True
  def all_checks(self, service_list):
    assert service_list == ['radarState']
    return self.healthy


def fixture():
  lead = NS(status=True,radar=True,dRel=45.,vRel=3.,vLead=16.9,aLeadK=.5)
  sm = SM(carState=NS(vEgo=13.9,aEgo=.1,gasPressed=False,brakePressed=False,
                     standstill=False,steeringAngleDeg=0.,cruiseGap=2.),
          controlsState=NS(vCruise=90.,longControlState='pid',forceDecel=False,active=True),
          radarState=NS(leadOne=lead,leadTwo=lead,radarErrors=[]),modelV2=NS())
  m = NS(mode='acc',stop_line=False,applyLongDynamicCost=False,stopDistance=4.,
         crash_cnt=0,v_solution=np.zeros(13),a_solution=np.zeros(13),j_solution=np.zeros(12),
         set_weights=lambda *a:None,set_cur_state=lambda *a:None,update=lambda *a:None)
  m.set_accel_limits = lambda lo,hi:setattr(m,'limits',(lo,hi))
  obj = NS(mpc=m,CP=NS(steerRatio=16.7,wheelbase=2.69),catchup_enabled=True,
           use_cluster_speed=True,a_desired=.1,
           v_desired_filter=NS(x=13.9,update=lambda v:v),
           cruise_solutions=lambda *a:('cruise',float('inf'),25.),
           parse_model=lambda *a:[np.zeros(13) for _ in range(5)])
  return obj,sm


class PlannerIntegrationTests(unittest.TestCase):
  def run_update(self, obj, sm):
    env['update'](obj,sm)
    return obj.mpc.limits

  def test_candidate_only_raises_upper_limit(self):
    obj,sm=fixture()
    lo,hi=self.run_update(obj,sm)
    self.assertEqual(lo,-1.2)
    self.assertAlmostEqual(hi,1.0,places=12)

  def test_baseline_gates(self):
    for field,value in [('active',False),('forceDecel',True),('longControlState','off'),
                        ('longControlState','stopping'),('longControlState','starting')]:
      obj,sm=fixture()
      setattr(sm['controlsState'],field,value)
      self.assertAlmostEqual(self.run_update(obj,sm)[1],.7,places=12)
    for field in ('gasPressed','brakePressed','standstill'):
      obj,sm=fixture()
      setattr(sm['carState'],field,True)
      self.assertAlmostEqual(self.run_update(obj,sm)[1],.7,places=12)

  def test_stale_radar_and_existing_experiments(self):
    obj,sm=fixture();sm.healthy=False
    self.assertAlmostEqual(self.run_update(obj,sm)[1],.7,places=12)
    obj,sm=fixture();sm['radarState'].radarErrors=['canError']
    self.assertAlmostEqual(self.run_update(obj,sm)[1],.7,places=12)
    for field,value in [('applyLongDynamicCost',True),('stop_line',True),('mode','blended')]:
      obj,sm=fixture();setattr(obj.mpc,field,value)
      self.assertAlmostEqual(self.run_update(obj,sm)[1],.7,places=12)

  def test_disabled_and_low_cruise_target(self):
    obj,sm=fixture();obj.catchup_enabled=False
    self.assertAlmostEqual(self.run_update(obj,sm)[1],.7,places=12)
    obj,sm=fixture();sm['controlsState'].vCruise=40.
    self.assertAlmostEqual(self.run_update(obj,sm)[1],.7,places=12)

  def test_turn_limit_still_applies(self):
    obj,sm=fixture();sm['carState'].steeringAngleDeg=90.
    self.assertLess(self.run_update(obj,sm)[1],.7)


if __name__ == '__main__':
  unittest.main()
