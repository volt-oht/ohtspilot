"""Offline recorded-input command comparison, not a brake/vehicle model."""
import argparse
import gzip
import json
from pathlib import Path
from hold_relax_candidate import HoldRelaxCandidate


def main():
  p=argparse.ArgumentParser();p.add_argument('input',type=Path);p.add_argument('output',type=Path);a=p.parse_args()
  data=json.loads(gzip.decompress(a.input.read_bytes()));h=HoldRelaxCandidate()
  cs=None;previous=None;rows=[]
  for r in sorted(data['rows'],key=lambda r:r['t']):
    if r['k']=='carState':cs=r
    elif r['k']=='carControl' and cs is not None:
      dt=r['t']-previous if previous is not None else .01;previous=r['t']
      output=h.update(r['command'],data['params']['stopAccel'],dt,
        stopping=r['state']=='stopping',active=r['active'],standstill=cs['standstill'],
        gas=cs['gas'],brake=cs['brake'],v_ego=cs['v'],v_raw=cs['raw_v'],
        wheels=list(cs['wheels'].values()),valid=0<=r['t']-cs['t']<=.03)
      rows.append(dict(t=r['t'],baseline=r['command'],candidate=output,
        state=r['state'],max_wheel=max(abs(v) for v in cs['wheels'].values()),
        speed=cs['v'],elapsed=h.elapsed,blocked=h.blocked))
  cases=[]
  for lo,hi in [(590,626),(708,721)]:
    sample=[r for r in rows if lo<=r['t']<=hi]
    changed=[r for r in sample if abs(r['candidate']-r['baseline'])>1e-7]
    departures=[r for r in sample if r['baseline']>0 and r['state'] in ('stopping','starting')]
    assert all(r['candidate']==r['baseline'] for r in departures)
    assert all(r['candidate']<=-2. and r['max_wheel']<=.02 and abs(r['speed'])<=.02 for r in changed)
    assert all(r['candidate']>=r['baseline'] for r in changed)
    cases.append(dict(window=[lo,hi],changed_samples=len(changed),
      first_change=changed[0] if changed else None,last_change=changed[-1] if changed else None,
      reached_target=next((r['t'] for r in changed if r['candidate']==-2.),None),
      maximum_command_change=max((r['candidate']-r['baseline'] for r in changed),default=0),
      departure_commands_unchanged=True))
  output={'scope':'Recorded input offline-only command relaxation; recorded motion is NOT a simulated response. No brake-pressure or grade model.',
          'dwell_s':h.DWELL,'speed_threshold_mps':h.SPEED_MAX,'rate_mps3':h.RATE,
          'cases':cases,'rows':rows,'vehicle_validated':False,'device_deployed':False}
  a.output.write_text(json.dumps(output,indent=2)+'\n')
  print(json.dumps({k:v for k,v in output.items() if k!='rows'},indent=2))


if __name__=='__main__':main()
