"""Offroad import/constructor check. Does not publish messages or command CAN."""
import argparse
import json
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from common.params import Params
from tools.lib.logreader import LogReader
from selfdrive.controls.lib import lead_catchup
from selfdrive.controls.lib.longitudinal_planner import Planner

p = argparse.ArgumentParser()
p.add_argument('--car-params-log', required=True)
args = p.parse_args()
if Params().get('IsOnroad') != b'0':
  raise RuntimeError('Offroad state required')
cp = next(msg.carParams for msg in LogReader(args.car_params_log) if msg.which() == 'carParams')
instance = Planner(cp)
assert lead_catchup.ENABLED and instance.catchup_enabled
print(json.dumps({'enabled': lead_catchup.ENABLED, 'vehicle_gate': instance.catchup_enabled,
                  'solver': type(instance.mpc.solver).__name__,
                  'scope': 'Offroad constructor only; not onroad or vehicle validation'}))
