#!/usr/bin/env python3
"""Independent module builds and dependency checks, without GRE or demo source files."""
from pathlib import Path
import concurrent.futures
import hashlib
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
MODELS = ['ROPE', 'DRIVE', 'TERRAIN', 'SUSPENSION', 'GEAR', 'BELT', 'PULLEY', 'VEHICLE']
CASES = [('core', [])] + [(m.lower(), [m]) for m in MODELS if m != 'VEHICLE'] + [
    ('vehicle', ['VEHICLE', 'DRIVE', 'TERRAIN', 'SUSPENSION']), ('all', MODELS)]


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
                raise RuntimeError(str(build / 'verify.log') + '\n' + result.stdout[-2000:])
            return result.stdout
        flags = ['-DYMPME_MODEL_' + m + '=' + ('ON' if m in enabled else 'OFF') for m in MODELS]
        run(['cmake', '-S', str(ROOT), '-B', str(build), '-DCMAKE_BUILD_TYPE=Release',
             '-DCMAKE_EXPORT_COMPILE_COMMANDS=ON', '-DYMPME_BUILD_TESTS=ON',
             '-DYMPME_BUILD_DEMOS=OFF', '-DYMPME_BUILD_GRE_DEMO=OFF',
             '-DYMPME_BUILD_GRE_BRIDGE=OFF', '-DYMPME_BUILD_ROPE_GRE=OFF',
             '-DYMPME_ENABLE_FLIGHT=OFF', *flags])
        run(['cmake', '--build', str(build), '-j', '2'])
        run(['ctest', '--test-dir', str(build), '--output-on-failure'])
        commands = (build / 'compile_commands.json').read_text()
        assert 'project_Demo' not in commands and '0502_YMGRE' not in commands, name
        for model in MODELS:
            archive = build / 'YMPME/MODEL' / model / ('libympme_' + model.lower() + '.a')
            assert archive.exists() == (model in enabled), (name, model)
            if model in enabled:
                undefined = run(['nm', '-u', str(archive)])
                for forbidden in ['malloc', 'calloc', 'realloc', 'YMGRE_', 'YMGUI_', 'SDL_', 'fopen', 'fwrite']:
                    assert forbidden not in undefined, (name, model, forbidden)
                defined = run(['nm', '--defined-only', str(archive)])
                for line in defined.splitlines():
                    parts = line.split()
                    assert not (len(parts) == 3 and parts[1] in ('b','B','d','D')), (name, model, line)
        abi = run([str(build / 'test_features_core')]).strip()
        fingerprint = hashlib.sha256((build / 'libympme.a').read_bytes()).hexdigest()
        print('PASS', name, ','.join(enabled) or 'core only', flush=True)
        return fingerprint, abi


if __name__ == '__main__':
    with tempfile.TemporaryDirectory(prefix='ympme-models-') as directory:
        work = Path(directory)
        with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
            results = list(pool.map(lambda case: check(work, case), CASES))
        assert len(set(results)) == 1, 'Model flags changed core archive or ABI'
        bad = subprocess.run(['cmake', '-S', str(ROOT), '-B', str(work / 'bad-vehicle'),
                              '-DYMPME_MODEL_VEHICLE=ON'], capture_output=True, text=True)
        assert bad.returncode != 0 and 'VEHICLE requires' in bad.stdout + bad.stderr
        print('PASS 10 configurations: independent models, no renderer/heap/IO symbols, no writable globals, unchanged core archive/ABI; missing dependencies rejected.')
