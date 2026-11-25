"""Counterfactual state replay on selected logs; never controls a vehicle.

Uses actual long_control_state_trans with recorded trajectories, not a closed
loop rollout. Input contains private driving signals; output is local only.
"""
import argparse
import ast
import gzip
import json
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np

def transition_function(root):
  tree=ast.parse((root/'selfdrive/controls/lib/longcontrol.py').read_text())
  fn=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='long_control_state_trans')
  env={'LongCtrlState':NS(off='off',pid='pid',starting='starting',stopping='stopping')}
  exec(compile(ast.Module(body=[fn],type_ignores=[]),'actual_long_control_state_trans','exec'),env)
  return env[fn.name]

def replay(segment,transition,variant):
  cp=NS(**segment['params'])
  if variant=='threshold_03':cp.vEgoStarting=.3
  state=None;cs=None;control=None;plan=None;plan_t=None;raw=None
  starts=[];stops=[];mismatches=0;compared=0
  for row in sorted(segment['rows'],key=lambda r:r['t']):
    kind=row['kind'];t=row['t']
    if kind=='carState':cs=row['data']
    elif kind=='controlsState':control=row['data']
    elif kind=='longitudinalPlan':plan=row['data'];plan_t=t
    elif kind=='can' and row['address']==452 and row['bus']==0:raw=bytes.fromhex(row['hex'])[1]>>5
    elif kind=='carControl' and cs and control and plan and raw is not None:
      if cs['gasPressed']:break  # censor before driver intervention
      if state is None:state=row['data']['actuators']['longControlState']
      speeds=plan['speeds'];idx=[10*(i/32)**2 for i in range(len(speeds))]
      age=max(0.,t-plan_t)
      lo=cp.longitudinalActuatorDelayLowerBound+age;hi=cp.longitudinalActuatorDelayUpperBound+age
      target=min(float(np.interp(lo,idx,speeds)),float(np.interp(hi,idx,speeds)))
      future=float(np.interp(hi+1.,idx,speeds))
      hold=cs['cruiseState']['standstill']
      if variant=='invert_raw':hold=raw==4
      elif variant=='ignore_hold':hold=False
      new=transition(cp,row['data']['longActive'] and cs['cruiseState']['enabledAcc'],state,cs['vEgo'],target,future,cs['brakePressed'],hold)
      if new=='starting' and state!='starting':starts.append(t)
      if new=='stopping' and state!='stopping':stops.append(t)
      mismatches+=new!=row['data']['actuators']['longControlState'];compared+=1
      state=new
  return dict(starts=starts,stops=stops,compared=compared,recorded_state_mismatches=mismatches)

def main():
  p=argparse.ArgumentParser();p.add_argument('input',type=Path);p.add_argument('output',type=Path);a=p.parse_args()
  data=json.loads(gzip.decompress(a.input.read_bytes()))
  transition=transition_function(Path(__file__).resolve().parents[2])
  result={'scope':'Recorded input counterfactual; publication time approximates receipt time. Neither a plant simulation nor proof of safe deployment.','cases':[]}
  for segment in data['launches']:
    variants={v:replay(segment,transition,v) for v in ['baseline','invert_raw','ignore_hold','threshold_03']}
    recorded=next(r['t'] for r in segment['rows'] if r['kind']=='carControl' and r['data']['actuators']['longControlState']=='starting')
    result['cases'].append(dict(segment=segment['segment'],recorded_start=recorded,variants=variants))
  a.output.write_text(json.dumps(result,indent=2)+'\n')
  print(json.dumps(result,indent=2))

if __name__=='__main__':main()
