#!/usr/bin/env python3
"""Independent core/model/sensor builds; no SKE/GRE dependency or core ABI change."""
from pathlib import Path
import hashlib,subprocess,tempfile
root=Path(__file__).resolve().parents[1]
results=[]
with tempfile.TemporaryDirectory(prefix='ympme-skin-prune-') as tmp:
 for name,model,sensor in [('core',False,False),('model',True,False),('sensor',False,True),('both',True,True)]:
  b=Path(tmp)/name
  def run(cmd):
   p=subprocess.run(cmd,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
   if p.returncode:raise RuntimeError(p.stdout[-4000:])
   return p.stdout
  run(['cmake','-S',str(root),'-B',str(b),'-DCMAKE_BUILD_TYPE=Release','-DYMPME_BUILD_DEMOS=OFF','-DCMAKE_EXPORT_COMPILE_COMMANDS=ON','-DYMPME_MODEL_SKIN='+('ON' if model else 'OFF'),'-DYMPME_SENSOR_TACTILE='+('ON' if sensor else 'OFF')])
  run(['cmake','--build',str(b),'-j','4']);run(['ctest','--test-dir',str(b),'--output-on-failure'])
  archives=[b/'YMPME/MODEL/SKIN/libympme_skin_contact.a',b/'YMPME/SENSOR/TACTILE/libympme_tactile.a']
  for archive,expected in zip(archives,[model,sensor]):
   assert archive.exists()==expected
   if expected:
    symbols=run(['nm','-u',str(archive)])
    assert not any(x in symbols for x in ['YMSKE_','YMGRE_','YMGUI_','SDL_','malloc','calloc','realloc'])
    if archive==archives[1]:assert 'YMPME_SkinContact' not in symbols
  commands=(b/'compile_commands.json').read_text();assert 'YMSKE' not in commands and '0502_YMGRE' not in commands
  abi=run([str(b/'test_features_core')]).strip();results.append((hashlib.sha256((b/'libympme.a').read_bytes()).hexdigest(),abi))
  print('PASS',name,abi,flush=True)
 assert len(set(results))==1
 print('PASS 4 configurations, independent skin model/tactile, unchanged core archive/ABI, no renderer/skeleton/heap dependency')
