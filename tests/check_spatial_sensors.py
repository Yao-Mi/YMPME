#!/usr/bin/env python3
"""Spatial sensors can use custom ray/contact providers without World query or GRE."""
from pathlib import Path
import concurrent.futures
import hashlib
import subprocess
import tempfile
ROOT = Path(__file__).resolve().parents[1]
FLAGS = ['ENABLE_QUERY', 'SENSOR_LIDAR', 'SENSOR_DEPTH', 'SENSOR_TACTILE']
CASES = [('core', [])] + [(flag.lower(), [flag]) for flag in FLAGS] + [('all', FLAGS)]
LIBS = ['YMPME/CORE/QUERY/libympme_query.a', 'YMPME/SENSOR/LIDAR/libympme_lidar.a',
        'YMPME/SENSOR/DEPTH/libympme_depth.a', 'YMPME/SENSOR/TACTILE/libympme_tactile.a']


def check(work, case):
    name, enabled = case
    build = work / name
    build.mkdir()
    with (build/'verify.log').open('w') as log:
        def run(args):
            p = subprocess.run(args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            log.write(p.stdout)
            log.flush()
            if p.returncode:
                raise RuntimeError(str(build)+'\n'+p.stdout[-3000:])
            return p.stdout
        run(['cmake', '-S', str(ROOT), '-B', str(build), '-DCMAKE_BUILD_TYPE=Release',
             '-DYMPME_BUILD_DEMOS=OFF', '-DCMAKE_EXPORT_COMPILE_COMMANDS=ON',
             *['-DYMPME_'+f+'='+('ON' if f in enabled else 'OFF') for f in FLAGS]])
        run(['cmake', '--build', str(build), '-j', '2'])
        run(['ctest', '--test-dir', str(build), '--output-on-failure'])
        for flag, path in zip(FLAGS, LIBS):
            library = build / path
            assert library.exists() == (flag in enabled), (name, flag)
            if library.exists():
                symbols = run(['nm', '-u', str(library)])
                forbidden = ['malloc', 'calloc', 'realloc', 'YMGRE_', 'YMGUI_', 'SDL_']
                if flag != 'ENABLE_QUERY':
                    forbidden += ['YMPME_Query_', 'YMPME_World_', 'YMPME_Aircraft_']
                assert all(f not in symbols for f in forbidden), (name, symbols)
        commands = (build/'compile_commands.json').read_text()
        assert '0502_YMGRE' not in commands
        for flag, source in zip(FLAGS, ['/CORE/QUERY/', '/SENSOR/LIDAR/', '/SENSOR/DEPTH/', '/SENSOR/TACTILE/']):
            assert (source in commands) == (flag in enabled)
        abi = run([str(build/'test_features_core')]).strip()
        print('PASS', name, abi, flush=True)
        return hashlib.sha256((build/'libympme.a').read_bytes()).hexdigest(), abi


if __name__ == '__main__':
    with tempfile.TemporaryDirectory(prefix='ympme-spatial-') as tmp:
        with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
            results = list(pool.map(lambda case: check(Path(tmp), case), CASES))
        assert len(set(results)) == 1
        print('PASS 6 builds; disabled sources absent; unchanged core; sensors have no World query/renderer/heap dependency')
