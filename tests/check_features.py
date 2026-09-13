#!/usr/bin/env python3
"""Build independent pruning configurations; optional developer check, not a library dependency."""
import argparse
import concurrent.futures
import hashlib
from pathlib import Path
import subprocess
import tempfile

CASES = [('core', False, True, True, False, False), ('flight_base', True, False, False, False, False),
         ('rotor', True, True, False, False, False), ('aero', True, False, True, False, False),
         ('full', True, True, True, False, False), ('rope', False, True, True, True, False),
         ('rope_flight', True, True, True, True, False), ('aircraft', True, True, True, False, True)]


def check(source, work, case):
    name, flight, rotor, aero, rope, aircraft = case
    build = work / name
    build.mkdir(parents=True)
    log_path = build / 'verify.log'
    with log_path.open('w') as log:
        def run(args):
            result = subprocess.run(args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            log.write(result.stdout)
            log.flush()
            if result.returncode:
                raise RuntimeError(str(log_path) + '\n' + result.stdout[-3000:])
            return result.stdout
        flags = [f'-DYMPME_MODEL_AIRCRAFT={"ON" if aircraft else "OFF"}',
                 f'-DYMPME_MODEL_ROPE={"ON" if rope else "OFF"}',
                 f'-DYMPME_ENABLE_FLIGHT={"ON" if flight else "OFF"}',
                 f'-DYMPME_FLIGHT_ROTORS={"ON" if rotor else "OFF"}',
                 f'-DYMPME_FLIGHT_AERODYNAMICS={"ON" if aero else "OFF"}']
        run(['cmake', '-S', str(source), '-B', str(build), '-DCMAKE_BUILD_TYPE=Release',
             '-DYMPME_BUILD_DEMOS=OFF', '-DYMPME_BUILD_TESTS=ON',
             '-DYMPME_BUILD_GRE_DEMO=OFF', '-DYMPME_BUILD_GRE_BRIDGE=OFF', *flags])
        run(['cmake', '--build', str(build), '-j', '2'])
        run(['ctest', '--test-dir', str(build), '--output-on-failure'])
        core_archive = build / 'libympme.a'
        core_symbols = run(['nm', '-g', '--defined-only', str(core_archive)])
        assert 'YMPME_Flight_' not in core_symbols and 'YMPME_Rope_' not in core_symbols, name
        core_hash = hashlib.sha256(core_archive.read_bytes()).hexdigest()
        abi = run([str(build / 'test_features_core')]).strip()
        archive = build / 'YMPME/EXT/FLIGHT/libympme_flight.a'
        if flight:
            symbols = run(['nm', '-g', '--defined-only', str(archive)])
            assert 'YMPME_Flight_ZeroLoad' in symbols, name
            assert ('YMPME_Flight_RotorLoad' in symbols) == rotor, name
            assert ('YMPME_Flight_AeroLoad' in symbols) == aero, name
            # Verify which object files entered the archive, not just public declarations.
            members = run(['ar', 't', str(archive)])
            assert ('YMPME_Rotor.c.o' in members) == rotor, name
            assert ('YMPME_Aerodynamics.c.o' in members) == aero, name
        else:
            assert not archive.exists(), name
            assert not (build / 'YMPME/EXT/FLIGHT').exists(), name
        rope_archive = build / 'YMPME/MODEL/ROPE/libympme_rope.a'
        if rope:
            symbols = run(['nm', '-g', '--defined-only', str(rope_archive)])
            assert 'YMPME_Rope_Step' in symbols, name
            undefined = run(['nm', '-u', str(rope_archive)])
            for forbidden in ('malloc', 'calloc', 'realloc', 'YMGRE_', 'YMGUI_', 'SDL_'):
                assert forbidden not in undefined, (name, forbidden)
        else:
            assert not rope_archive.exists(), name
            assert not (build / 'YMPME/MODEL/ROPE').exists(), name
        aircraft_archive = build / 'YMPME/MODEL/AIRCRAFT/libympme_aircraft.a'
        assert aircraft_archive.exists() == aircraft
        assert 'YMPME_Aircraft_' not in core_symbols
        if aircraft:
            symbols = run(['nm', '-u', str(aircraft_archive)])
            for forbidden in ('malloc', 'calloc', 'realloc', 'YMGRE_', 'YMGUI_', 'SDL_'):
                assert forbidden not in symbols, (name, forbidden)
            assert 'YMPME_Flight_RotorLoad' in symbols and 'YMPME_Flight_AeroLoad' in symbols
        print('PASS', name, abi, flush=True)
        return core_hash, abi


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--source', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--work-dir', type=Path, help='Keep logs/builds in a new directory')
    args = parser.parse_args()
    temporary = None
    if args.work_dir:
        args.work_dir.mkdir(parents=True, exist_ok=False)
        work = args.work_dir.resolve()
    else:
        temporary = tempfile.TemporaryDirectory(prefix='ympme-features-')
        work = Path(temporary.name)
    try:
        with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
            results = list(pool.map(lambda case: check(args.source.resolve(), work, case), CASES))
        assert len(set(results)) == 1, 'Optional model/flight settings changed core archive bytes or core ABI'
        for missing in ('YMPME_ENABLE_FLIGHT', 'YMPME_FLIGHT_ROTORS', 'YMPME_FLIGHT_AERODYNAMICS'):
            flags = ['-DYMPME_MODEL_AIRCRAFT=ON', '-DYMPME_ENABLE_FLIGHT=ON',
                     '-DYMPME_FLIGHT_ROTORS=ON', '-DYMPME_FLIGHT_AERODYNAMICS=ON', '-D'+missing+'=OFF']
            bad = subprocess.run(['cmake', '-S', str(args.source.resolve()), '-B', str(work / missing), *flags], capture_output=True, text=True)
            assert bad.returncode != 0 and 'AIRCRAFT requires' in bad.stderr
        print('PASS all 8 configurations: byte-identical core archive and ABI; excluded sources/symbols absent.')
    finally:
        if temporary:
            temporary.cleanup()


if __name__ == '__main__':
    main()
