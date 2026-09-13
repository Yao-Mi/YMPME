#!/usr/bin/env python3
"""Six independent builds: optional sensor sources, core ABI and no renderer/heap dependency."""
from pathlib import Path
import concurrent.futures, hashlib, subprocess, tempfile
ROOT=Path(__file__).resolve().parents[1]
SENSORS={'IMU':'imu','ENCODER':'encoder','FORCE':'force_sensor'}
CASES=[('core',[],False)]+[(n.lower(),[n],False) for n in SENSORS]+[('sensors',list(SENSORS),False),('control',list(SENSORS),True)]
def check(work,case):
    name,enabled,air=case;b=work/name;b.mkdir()
    with (b/'verify.log').open('w') as log:
        def run(args):
            p=subprocess.run(args,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True);log.write(p.stdout);log.flush()
            if p.returncode:raise RuntimeError(str(b)+'\n'+p.stdout[-2000:])
            return p.stdout
        flags=['-DYMPME_SENSOR_'+n+'='+('ON' if n in enabled else 'OFF') for n in SENSORS]
        run(['cmake','-S',str(ROOT),'-B',str(b),'-DCMAKE_BUILD_TYPE=Release','-DYMPME_BUILD_DEMOS=OFF','-DCMAKE_EXPORT_COMPILE_COMMANDS=ON','-DYMPME_MODEL_AIRCRAFT='+('ON' if air else 'OFF'),'-DYMPME_ENABLE_FLIGHT='+('ON' if air else 'OFF'),*flags])
        run(['cmake','--build',str(b),'-j','2']);run(['ctest','--test-dir',str(b),'--output-on-failure'])
        for sensor,lib in SENSORS.items():
            archive=b/'YMPME/SENSOR'/sensor/('libympme_'+lib+'.a');assert archive.exists()==(sensor in enabled)
            if archive.exists():
                symbols=run(['nm','-u',str(archive)])
                for forbidden in ['malloc','calloc','realloc','YMGRE_','YMGUI_','SDL_','YMPME_Aircraft_']:assert forbidden not in symbols,(name,forbidden)
        assert '0502_YMGRE' not in (b/'compile_commands.json').read_text()
        core=b/'libympme.a';abi=run([str(b/'test_features_core')]).strip()
        print('PASS',name,abi,flush=True);return hashlib.sha256(core.read_bytes()).hexdigest(),abi
if __name__=='__main__':
    with tempfile.TemporaryDirectory(prefix='ympme-sensors-') as d:
        with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:results=list(pool.map(lambda case:check(Path(d),case),CASES))
        assert len(set(results))==1
        print('PASS 6 sensor/control configurations; core archive/ABI invariant across flags; disabled sources absent; sensors independent of aircraft/renderer/heap')
