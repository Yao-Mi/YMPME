#!/usr/bin/env python3
"""Build the modular PME SDK; shared verification also supports the unified builder."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import platform
import re
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
MODULE_OPTIONS = [
    "ENABLE_RIGID", "ENABLE_JOINT", "ENABLE_DYNAMICS", "ENABLE_QUERY", "ENABLE_FLIGHT",
    "ENABLE_SOFTBODY", "MODEL_DRIVE", "MODEL_TERRAIN", "MODEL_SUSPENSION", "MODEL_VEHICLE",
    "MODEL_GEAR", "MODEL_BELT", "MODEL_PULLEY", "MODEL_ROPE", "MODEL_AIRCRAFT", "MODEL_PENDULUM",
    "MODEL_SURFACE_ATLAS", "MODEL_SKIN", "SENSOR_IMU", "SENSOR_ENCODER", "SENSOR_FORCE",
    "SENSOR_LASER", "SENSOR_LIDAR", "SENSOR_DEPTH", "SENSOR_TACTILE",
]

def run(args, log):
    args = [str(x) for x in args]
    with log.open("a") as stream:
        stream.write("\n$ " + repr(args) + "\n")
        stream.flush()
        result = subprocess.run(args, stdout=stream, stderr=subprocess.STDOUT, cwd=ROOT)
    if result.returncode:
        print(log.read_text()[-10000:], flush=True)
        raise RuntimeError(f"Command failed; see {log}")

def source_hash():
    paths = [ROOT / "CMakeLists.txt", ROOT / "API.md", ROOT / "ARCHITECTURE.md", ROOT / "MESH_COLLISION.md"]
    for folder in ("YMPME", "cmake", "sdk"):
        paths.extend(p for p in (ROOT / folder).rglob("*") if p.is_file() and "__pycache__" not in p.parts)
    h = hashlib.sha256()
    for p in sorted(paths):
        h.update(p.relative_to(ROOT).as_posix().encode() + b"\0" + p.read_bytes() + b"\0")
    return h.hexdigest()

def main(unified=False, argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    if not unified:
        parser.add_argument("--profile", choices=("full", "core"), default="full")
    else:
        parser.set_defaults(profile="full")
    parser.add_argument("--output", type=Path, default=ROOT / ("build/YMPME_libs" if unified else "build/YMPME_libs_modular"))
    parser.add_argument("--index-bits", choices=(16,32), type=int, default=32)
    parser.add_argument("--jobs", type=int, default=4)
    args = parser.parse_args(argv)
    if args.jobs < 1:
        parser.error("--jobs must be positive")
    output = args.output.resolve()
    build_root = (ROOT / "build").resolve()
    if build_root not in output.parents or output == build_root / "sdk-work":
        parser.error("Publish to a dedicated directory under YMPME/build")
    work = build_root / "sdk-work" / (("unified" if unified else "modular-" + args.profile) + "-index" + str(args.index_bits))
    if output == work or output in work.parents or work in output.parents:
        parser.error("Output must be separate from sdk-work")
    if output.exists() and not (output / ".ympme-sdk").is_file():
        parser.error("Refusing to replace a directory without a .ympme-sdk marker")
    output.parent.mkdir(parents=True, exist_ok=True)
    work.mkdir(parents=True, exist_ok=True)
    build_log = work / "build.log"
    build_log.write_text("")
    before = source_hash()
    config = ["cmake", "-S", ROOT, "-B", work / "engine", "-DCMAKE_BUILD_TYPE=Release",
              "-DCMAKE_POSITION_INDEPENDENT_CODE=ON", "-DYMPME_INSTALL_SDK=ON",
              "-DYMPME_BUILD_TESTS=OFF", "-DYMPME_BUILD_DEMOS=OFF",
              "-DYMPME_BUILD_GRE_BRIDGE=OFF", "-DYMPME_BUILD_GRE_DEMO=OFF",
              "-DYMPME_BUILD_ROPE_GRE=OFF", "-DYMPME_BUILD_HAND_BALL_DEMO=OFF",
              "-DYMPME_BUILD_HAND_SKIN_DEMO=OFF", "-DYMPME_ENABLE_SANITIZERS=OFF",
              "-DYMPME_FLIGHT_ROTORS=ON", "-DYMPME_FLIGHT_AERODYNAMICS=ON"]
    config += [f"-DYMPME_{opt}={'ON' if args.profile == 'full' else 'OFF'}" for opt in MODULE_OPTIONS]
    config.append(f"-DYMPME_INDEX_BITS={args.index_bits}")
    config.append(f"-DYMPME_SDK_UNIFIED={'ON' if unified else 'OFF'}")
    print(f"Building {'unified' if unified else args.profile} SDK in {work / 'engine'}", flush=True)
    run(config, build_log)
    run(["cmake", "--build", work / "engine", "--parallel", args.jobs] +
        (["--target", "ympme_unified"] if unified else []), build_log)
    with tempfile.TemporaryDirectory(prefix=".ympme-sdk-stage-", dir=output.parent) as temp:
        stage = Path(temp) / "YMPME_libs"
        run(["cmake", "--install", work / "engine", "--prefix", stage], build_log)
        (stage / ".ympme-sdk").write_text("YMPME standalone SDK\n")
        verify = stage / "verification"
        verify.mkdir()
        shutil.copy2(build_log, verify / "build.log")
        # A different path containing spaces proves imports are relocatable.
        with tempfile.TemporaryDirectory(prefix="relocated SDK ", dir=work) as relocated_dir:
            moved = Path(relocated_dir) / "YMPME_libs"
            shutil.copytree(stage, moved)
            check_build = Path(relocated_dir) / "consumer-build"
            log = verify / "external-examples.log"
            print("Verifying relocated SDK examples (no engine sources)", flush=True)
            run(["cmake", "-S", moved / "examples", "-B", check_build,
                 "-DCMAKE_BUILD_TYPE=Release", f"-DYMPME_DIR={moved / 'cmake'}"], log)
            run(["cmake", "--build", check_build, "--parallel", args.jobs], log)
            run(["ctest", "--test-dir", check_build, "--output-on-failure"], log)
            run(["cmake", "--install", check_build, "--prefix", stage], log)
            # Force all object files in all modules to link, so unused APIs cannot hide
            # missing dependencies. This test does not execute foreign declarations.
            probe = Path(relocated_dir) / "probe"
            probe.mkdir()
            (probe / "main.c").write_text('#include "YMPME.h"\nint main(void) { return 0; }\n')
            (probe / "CMakeLists.txt").write_text(r"""cmake_minimum_required(VERSION 3.16)
project(YMPMEPackageLinkCheck C)
find_package(YMPME CONFIG REQUIRED COMPONENTS ympme)
add_executable(all_modules main.c)
set(modules "")
if(YMPME_SDK_UNIFIED)
    list(APPEND modules YMPME::ympme)
else()
    foreach(component IN LISTS YMPME_AVAILABLE_COMPONENTS)
        list(APPEND modules YMPME::${component})
    endforeach()
endif()
if(UNIX AND NOT APPLE)
    target_link_libraries(all_modules PRIVATE -Wl,--whole-archive ${modules} -Wl,--no-whole-archive)
else()
    target_link_libraries(all_modules PRIVATE ${modules})
endif()
""")
            log = verify / "all-modules-link.log"
            run(["cmake", "-S", probe, "-B", probe / "build",
                 f"-DCMAKE_PREFIX_PATH={moved}"], log)
            run(["cmake", "--build", probe / "build", "--parallel", args.jobs], log)
            run([probe / "build" / "all_modules"], log)
            bad = Path(relocated_dir) / "missing-component"
            bad.mkdir()
            required = "dynamics" if args.profile == "core" else "not_a_real_module"
            (bad / "CMakeLists.txt").write_text("cmake_minimum_required(VERSION 3.16)\nproject(MissingModule C)\n"
                f"find_package(YMPME CONFIG REQUIRED COMPONENTS {required})\n")
            result = subprocess.run(["cmake", "-S", str(bad), "-B", str(bad / "build"),
                f"-DYMPME_DIR={moved / 'cmake'}"], capture_output=True, text=True)
            (verify / "missing-component.log").write_text(result.stdout + result.stderr)
            if result.returncode == 0 or f"Component '{required}'" not in result.stdout + result.stderr:
                raise RuntimeError("Missing component was not correctly rejected")
            if unified:
                # No CMake definitions: installed headers enable the complete API by default.
                log = verify / "single-library-direct-link.log"
                compiler_cache = (work / "engine/CMakeCache.txt").read_text()
                cc = re.search(r"^CMAKE_C_COMPILER:FILEPATH=(.+)$", compiler_cache, re.M).group(1)
                for name in ("demo_sdk_dynamics", "demo_sdk_encoder", "demo_sdk_mesh_indices"):
                    exe = Path(relocated_dir) / (name + "-direct")
                    run([cc, "-std=c99", "-Wall", "-Wextra", "-Werror",
                         "-I", moved / "include/YMPME", moved / "examples" / (name + ".c"),
                         moved / "lib/libympme.a", "-lm", "-o", exe], log)
                    run([exe], log)
                archives = list((stage / "lib").glob("*.a"))
                if len(archives) != 1 or archives[0].name != "libympme.a":
                    raise RuntimeError("Unified SDK must contain exactly one libympme.a")
            else:
                symbols = subprocess.check_output(["nm", str(stage / "bin/demo_sdk_minimal")], text=True)
                if re.search(r"YMPME_(Flight|Rigid|Joint|Dynamics|Skin|Encoder|Lidar|IMU)", symbols):
                    raise RuntimeError("Core-only executable pulled optional modules")
                (verify / "core-isolation.log").write_text("PASS: core-only example has no dynamics/flight/skin/sensor symbols.\n")
        for p in (stage / "cmake").glob("*.cmake"):
            text = p.read_text()
            if str(ROOT) in text or str(stage) in text:
                raise RuntimeError(f"Non-relocatable path in {p}")
        if source_hash() != before:
            raise RuntimeError("Source changed during packaging; rerun to publish a consistent snapshot")
        cache = (work / "engine/CMakeCache.txt").read_text()
        compiler = re.search(r"^CMAKE_C_COMPILER:FILEPATH=(.+)$", cache, re.M).group(1)
        version = re.search(r"project\(YMPME VERSION ([\d.]+)", (ROOT / "CMakeLists.txt").read_text()).group(1)
        info = {
            "version": version, "built_at_utc": datetime.now(timezone.utc).isoformat(),
            "system": platform.system(), "machine": platform.machine(), "profile": args.profile, "layout": "unified" if unified else "modular", "index_bits": args.index_bits,
            "compiler": subprocess.check_output([compiler, "--version"], text=True).splitlines()[0],
            "build_type": "Release", "position_independent": True,
            "source_sha256": before, "static_libraries": sorted(p.name for p in (stage / "lib").glob("*.a")),
            "enabled_options": [opt for opt in MODULE_OPTIONS if args.profile == "full"],
            "graphics_dependencies": [], "requires_engine_source": False,
            "relocated_examples_passed": True, "all_modules_link_passed": True,
            "rebuild_command": "python3 sdk/build.py" if unified else "python3 sdk/build_modular.py --profile " + args.profile,
        }
        (stage / "BUILD_INFO.json").write_text(json.dumps(info, ensure_ascii=False, indent=2) + "\n")
        lines = [hashlib.sha256(p.read_bytes()).hexdigest() + "  " + p.relative_to(stage).as_posix()
                 for p in sorted(stage.rglob("*")) if p.is_file()]
        (stage / "checksums.sha256").write_text("\n".join(lines) + "\n")
        # Keep the last successful package until every check above passes.
        backup = output.with_name(output.name + ".previous")
        if backup.exists():
            raise RuntimeError(f"Backup already exists: {backup}; resolve it before replacing the SDK")
        if output.exists():
            output.rename(backup)
        try:
            stage.rename(output)
        except Exception:
            if backup.exists():
                backup.rename(output)
            raise
        if backup.exists():
            shutil.rmtree(backup)
    print(f"SDK ready: {output}", flush=True)
    print(f"{len(info['static_libraries'])} static libraries; relocated examples and module linking PASS", flush=True)

if __name__ == "__main__":
    main()
