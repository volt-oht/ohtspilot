import argparse,hashlib,json,pathlib,sys
p=argparse.ArgumentParser();p.add_argument('--source-only',action='store_true');p.add_argument('--runtime-root',default='/data');args=p.parse_args()
root=pathlib.Path(__file__).resolve().parents[2];base=json.loads(pathlib.Path(__file__).with_name('baseline.json').read_text());bad=[]
for name,expected in base['source_sha256'].items():
 f=root/name
 if not f.is_file() or hashlib.sha256(f.read_bytes()).hexdigest()!=expected:bad.append('source:'+name)
if not args.source_only:
 runtime=pathlib.Path(args.runtime_root)
 for key,expected in base['params'].items():
  f=runtime/'params'/'d'/key;actual=f.read_bytes().decode(errors='replace') if f.exists() else None
  if actual!=expected:bad.append('param:'+key)
 for name,expected in base['ntune'].items():
  f=runtime/'ntune'/name
  if not f.exists() or json.loads(f.read_text())!=expected:bad.append('ntune:'+name)
print(json.dumps({'scope':'source_only' if args.source_only else 'source_and_runtime','match':not bad,'mismatches':bad}))
sys.exit(bool(bad))
