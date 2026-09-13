#!/usr/bin/env python3
"""Build isolated core/rigid/joint/model/control configurations without a renderer."""
from pathlib import Path
import concurrent.futures
import hashlib
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
FLAGS = ['ENABLE_RIGID', 'ENABLE_JOINT', 'MODEL_PENDULUM', 'SENSOR_ENCODER']
CASES = [('core', []), ('rigid', FLAGS[:1]), ('joint', FLAGS[1:2]),
         ('pendulum', FLAGS[2:3]), ('control', FLAGS[2:])]


def check(work, case):
    name, enabled = case
    build = work / name
    build.mkdir()
    with (build / 'verify.log').open('w') as log:
        def run(args):
            result = subprocess.run(args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            log.write(result.stdout)
            log.flush()
            if result.returncode:
                raise RuntimeError(str(build) + '\n' + result.stdout[-3000:])
            return result.stdout
        flags = ['-DYMPME_' + flag + '=' + ('ON' if flag in enabled else 'OFF') for flag in FLAGS]
        run(['cmake', '-S', str(ROOT), '-B', str(build), '-DCMAKE_BUILD_TYPE=Release',
             '-DYMPME_BUILD_DEMOS=OFF', '-DCMAKE_EXPORT_COMPILE_COMMANDS=ON', *flags])
        run(['cmake', '--build', str(build), '-j', '2'])
        run(['ctest', '--test-dir', str(build), '--output-on-failure'])
        archives = [('YMPME/CORE/RIGID/libympme_rigid.a', name != 'core'),
                    ('YMPME/JOINT/libympme_joint.a', name in ('joint', 'pendulum', 'control')),
                    ('YMPME/MODEL/PENDULUM/libympme_pendulum.a', name in ('pendulum', 'control'))]
        for path, expected in archives:
            archive = build / path
            assert archive.exists() == expected, (name, path)
            if expected:
                symbols = run(['nm', '-u', str(archive)])
                for forbidden in ('malloc', 'calloc', 'realloc', 'YMGRE_', 'YMGUI_', 'SDL_', 'YMPME_Encoder_', 'YMPME_Aircraft_'):
                    assert forbidden not in symbols, (name, forbidden)
        commands = (build / 'compile_commands.json').read_text()
        assert '0502_YMGRE' not in commands and 'AIRCRAFT' not in commands
        if name == 'core':
            assert '/JOINT/' not in commands and '/RIGID/' not in commands
        abi = run([str(build / 'test_features_core')]).strip()
        print('PASS', name, abi, flush=True)
        return hashlib.sha256((build / 'libympme.a').read_bytes()).hexdigest(), abi


if __name__ == '__main__':
    with tempfile.TemporaryDirectory(prefix='ympme-joint-features-') as directory:
        with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
            results = list(pool.map(lambda case: check(Path(directory), case), CASES))
        assert len(set(results)) == 1
        print('PASS 5 configurations; core ABI/archive unchanged; disabled sources absent; rigid/joint/model have no renderer/sensor/heap dependency')
