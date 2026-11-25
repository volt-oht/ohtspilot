"""Source stopping-branch sensitivity only; NOT vehicle acceptance."""
import ast,json,pathlib,types,collections,sys
ROOT=pathlib.Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))
from common.numpy_fast import interp,clip
src=ROOT/'selfdrive/controls/lib/longcontrol.py';tree=ast.parse(src.read_text())
update=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='update')
branch=next(n for n in ast.walk(update) if isinstance(n,ast.If) and ast.unparse(n.test)=='self.long_control_state == LongCtrlState.stopping')
code=compile(ast.Module(body=branch.body,type_ignores=[]),str(src),'exec')
def step(v,a,rate):
 self=types.SimpleNamespace(CP=types.SimpleNamespace(stopAccel=-2.,stoppingDecelRate=rate),reset=lambda v:None)
 env=dict(self=self,CS=types.SimpleNamespace(standstill=v<.1,vEgo=v),output_accel=a,DT_CTRL=.01,interp=interp,clip=clip,accel_limits=(-3.5,3.))
 exec(code,env);return env['output_accel']
def run(v,a,rate,delay):
 queue=collections.deque([a]*round(delay/.01));x=0.;t=0
 for i in range(2000):
  cmd=step(v,a,rate);assert cmd<=0
  queue.append(cmd);physical=queue.popleft()
  nv=max(0.,v+physical*.01);x+=(v+nv)*.005;t+=.01;v=nv;a=cmd
  if v==0:break
 assert v==0
 return {'distance_m':x,'time_s':t,'end_command':a}
rows=[]
for v in [.2,.5,.8]:
 for a in [-.2,-.8,-1.5]:
  for delay in [0.,.42,1.1]:
   old=run(v,a,3.,delay);new=run(v,a,.9,delay)
   rows.append({'initial_v':v,'initial_a':a,'assumed_delay':delay,'baseline':old,'candidate':new,'extra_distance_m':new['distance_m']-old['distance_m']})
result={'scope':'Ideal delayed acceleration plant; only actual stopping branch executed. Not GM plant, full planner, or safety validation.','baseline_rate':3.,'candidate_rate':.9,'cases':rows,'max_extra_distance_m':max(r['extra_distance_m'] for r in rows),'device_use':'Not approved by this check; needs measured brake response and full stopping evaluation.'}
pathlib.Path(__file__).with_name('stopping_sensitivity.json').write_text(json.dumps(result,indent=2))
print(json.dumps({k:v for k,v in result.items() if k!='cases'}))
