"""Offroad-only original acados comparison; ideal plant, NOT vehicle validation.

Run on LEPRO from an isolated directory containing lead_catchup.py.
Does not publish messages or send CAN. Uses the unchanged installed MPC.
"""
import importlib.util
import json
import pathlib
import sys
from types import SimpleNamespace as NS
import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, '/data/openpilot')


def offroad():
  if pathlib.Path('/data/params/d/IsOnroad').read_bytes() != b'0':
    raise RuntimeError('Offroad state required')


def load(name, path):
  spec = importlib.util.spec_from_file_location(name, str(path))
  module = importlib.util.module_from_spec(spec)
  sys.modules[name] = module
  spec.loader.exec_module(module)
  return module


def run(cls, candidate, scenario):
  m = cls()
  m.stop_line = False
  m.AChangeCost = 150.
  m.JEgoCost = 5.
  m.XEgoObstacleCost = 5.
  m.DangerZoneCost = 100.
  m.leadDangerFactor = .75
  m.stopDistance = 4.
  m.applyLongDynamicCost = False
  v = 13.9
  a = 0.
  lead_v = 13.9
  gap = 45.
  if scenario == 'stationary':
    lead_v, gap = 0., 90.
  if scenario == 'close':
    lead_v, gap = 10., 22.
  rows = []
  dt = .05
  controls = NS(active=True, longControlState='pid', forceDecel=False)
  model = NS(position=NS(x=[1000.]*33), stopLine=NS(prob=0.))
  for i in range(500):
    if i % 20 == 0:
      offroad()
    t = i * dt
    la = .7 if 2. <= t < 10. and scenario not in ('stationary', 'close') else 0.
    if scenario in ('brake_after_accel', 'second_lead') and 10. <= t < 18.:
      la = -3. if lead_v > 0 else 0.
    present = not (scenario == 'lead_loss' and 8. <= t < 10.)
    lead = NS(status=present, radar=True, dRel=gap, vRel=lead_v-v,
              vLead=lead_v, aLeadK=la, aLeadTau=1.5, modelProb=1.)
    second = NS(**vars(lead))
    if scenario == 'second_lead' and 5. <= t < 8.:
      second.dRel = min(gap, 25.)
      second.vLead = min(lead_v, v)
      second.vRel = second.vLead-v
      second.aLeadK = -.5
    cs = NS(aEgo=a, vEgo=v, cruiseGap=2., brakePressed=False,
            gasPressed=False, standstill=v < .01)
    m.lo_timer = -1000  # fixed captured settings; do not poll changing Params
    base_limit = float(np.interp(v, [0.,13.9,19.4,33.], [1.4,.7,.5,.3]))
    limit = catchup.catchup_accel_limit(base_limit, candidate, True, v, a, 1.8, 4., (lead, second))
    m.set_accel_limits(-1.2, max(limit, a - .05))
    m.set_cur_state(v, a)
    vectors = [np.zeros(13) for _ in range(5)]
    m.update(cs, NS(leadOne=lead, leadTwo=second), model, controls, 25.,
             *vectors, prev_accel_constraint=True)
    status = m.solution_status
    next_a = float(np.interp(dt, mpc_times, m.a_solution))
    next_v = max(0., v + .5*(a+next_a)*dt)
    next_lead_v = max(0., lead_v+la*dt)
    gap += (.5*(lead_v+next_lead_v)-.5*(v+next_v))*dt
    rows.append(dict(t=t, gap=gap, v=next_v, lead_v=next_lead_v, a=next_a,
                     jerk=(next_a-a)/dt, scale=limit-base_limit, status=status))
    v, a, lead_v = next_v, next_a, next_lead_v
  return rows


if __name__ == '__main__':
  offroad()
  from selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc, T_IDXS
  mpc_times = T_IDXS
  catchup = load('catchup_candidate', HERE/'lead_catchup.py')
  candidate = LongitudinalMpc
  results = {}
  for scenario in ('pull_away', 'brake_after_accel', 'stationary', 'close', 'lead_loss', 'second_lead'):
    runs = {name: run(cls, changed, scenario) for name, cls, changed in
            [('baseline',LongitudinalMpc,False),('candidate',candidate,True)]}
    metrics = {}
    for name, rows in runs.items():
      metrics[name] = dict(min_gap=min(r['gap'] for r in rows),
        max_abs_jerk=max(abs(r['jerk']) for r in rows),
        mean_gap=float(np.mean([r['gap'] for r in rows])),
        modified_samples=sum(r['scale'] > 0 for r in rows),
        solver_errors=sum(r['status'] != 0 for r in rows),
        speed_at_10s=rows[200]['v'], final_speed=rows[-1]['v'])
    results[scenario] = dict(metrics=metrics, traces=runs)
    print(json.dumps(dict(scenario=scenario, metrics=metrics)), flush=True)
  (HERE/'native_results.json').write_text(json.dumps(results))
