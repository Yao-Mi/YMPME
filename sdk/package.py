#!/usr/bin/env python3
"""Validate and archive the dual-index PME SDK; --release writes releases/."""
import argparse
import gzip
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import platform
import re
import shutil
import subprocess
import sys
import tarfile
import tempfile

from build_modular import ROOT, source_hash


def digest(path):
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(chunk)
    return value.hexdigest()


def check_sdk(sdk, expected_source=None):
    """Refuse stale, modified, incomplete or externally linked packages."""
    if not sdk.is_dir() or sdk.is_symlink() or not (sdk / ".ympme-sdk").is_file():
        raise ValueError("SDK must be a managed YMPME_libs directory")
    for path in sdk.rglob("*"):
        if path.is_symlink() or not (path.is_file() or path.is_dir()):
            raise ValueError(f"Unsupported SDK entry: {path}")
    info = json.loads((sdk / "BUILD_INFO.json").read_text())
    if not re.fullmatch(r"[0-9]+\.[0-9]+\.[0-9]+", info.get("version", "")):
        raise ValueError("Missing SDK version")
    if info.get("layout") != "unified-dual-index" or info.get("index_bits") != [16, 32]:
        raise ValueError("Expected a complete dual-index SDK")
    if info.get("system") != platform.system() or info.get("machine") != platform.machine():
        raise ValueError("Extracted binaries must be verified on their recorded native platform")
    if expected_source is not None and info.get("source_sha256") != expected_source:
        raise ValueError("SDK does not match current sources; rebuild with --rebuild")
    for name in ("LICENSE", "README.md", "cmake/YMPMEConfig.cmake", "examples/CMakeLists.txt",
                 "include/YMPME/YMPME.h", "lib/libympme_index16.a", "lib/libympme_index32.a"):
        if not (sdk / name).is_file():
            raise ValueError(f"Missing SDK file: {name}")
    recorded = set()
    for line in (sdk / "checksums.sha256").read_text().splitlines():
        parts = line.split("  ", 1)
        if len(parts) != 2 or not re.fullmatch(r"[0-9a-f]{64}", parts[0]):
            raise ValueError("Invalid checksum record")
        checksum, name = parts
        relative = PurePosixPath(name)
        if relative.is_absolute() or ".." in relative.parts or not relative.parts or name in recorded:
            raise ValueError(f"Unsafe or duplicate checksum path: {name}")
        path = sdk / name
        if not path.is_file() or digest(path) != checksum:
            raise ValueError(f"SDK checksum mismatch: {name}")
        recorded.add(name)
    actual = {p.relative_to(sdk).as_posix() for p in sdk.rglob("*")
              if p.is_file() and p.relative_to(sdk).as_posix() != "checksums.sha256"}
    if recorded != actual:
        raise ValueError("SDK checksum inventory is incomplete")
    return info


def run(args, log):
    with log.open("a") as stream:
        stream.write("\n$ " + repr([str(a) for a in args]) + "\n")
        stream.flush()
        result = subprocess.run([str(a) for a in args], cwd=ROOT, stdout=stream,
                                stderr=subprocess.STDOUT)
    if result.returncode:
        raise RuntimeError(f"Verification failed; see {log}")


def verify_examples(sdk, work, jobs, info):
    """Build outside the extracted SDK and run both index-width consumers."""
    for bits in (16, 32):
        build = work / f"consumer{bits}"
        log = work.parent / f"consumer{bits}.log"
        run(["cmake", "-S", sdk / "examples", "-B", build,
             "-DCMAKE_BUILD_TYPE=Release", f"-DYMPME_DIR={sdk / 'cmake'}",
             f"-DYMPME_INDEX_BITS={bits}", "-DYMPME_SDK_WITH_GRE=OFF"], log)
        run(["cmake", "--build", build, "--parallel", jobs], log)
        run(["ctest", "--test-dir", build, "--output-on-failure", "--timeout", "120"], log)
        version = subprocess.check_output([sdk / "bin" / f"index{bits}" / "demo_sdk_version"],
                                          text=True).strip()
        if version != "YMPME " + info["version"]:
            raise ValueError("Prebuilt library/header/package version mismatch")


def normalize(member):
    member.uid = member.gid = 0
    member.uname = member.gname = ""
    member.mtime = 0
    member.mode = 0o755 if member.isdir() or member.mode & 0o111 else 0o644
    return member


def compress_sdk(sdk, pending):
    # Stable names/order/timestamps: the same SDK bytes reproduce the same archive.
    with pending.open("wb") as output:
        with gzip.GzipFile(filename="", mode="wb", fileobj=output, mtime=0) as compressed:
            with tarfile.open(fileobj=compressed, mode="w", format=tarfile.PAX_FORMAT) as bundle:
                bundle.add(sdk, arcname="YMPME_libs", recursive=False, filter=normalize)
                for path in sorted(sdk.rglob("*")):
                    bundle.add(path, arcname="YMPME_libs/" + path.relative_to(sdk).as_posix(),
                               recursive=False, filter=normalize)


def unpack_sdk(archive, dest):
    with tarfile.open(archive, "r:gz") as bundle:
        members = bundle.getmembers()
        for member in members:
            path = PurePosixPath(member.name)
            if (not path.parts or path.parts[0] != "YMPME_libs" or ".." in path.parts or
                    path.is_absolute() or not (member.isfile() or member.isdir())):
                raise ValueError(f"Unsafe archive member: {member.name}")
        bundle.extractall(dest, members=members)
    return dest / "YMPME_libs"


def publish_pair(pending, archive, checksum):
    """Replace only after verification; roll back if either final replacement fails."""
    targets = [(pending, archive), (checksum, archive.with_name(archive.name + ".sha256"))]
    backups = []
    replaced = []
    for i, (_, target) in enumerate(targets):
        if target.is_symlink() or (target.exists() and not target.is_file()):
            raise ValueError(f"Refusing unmanaged release destination: {target}")
        backup = pending.parent / f"previous-{i}"
        existed = target.exists()
        if existed:
            shutil.copy2(target, backup)
        backups.append((target, backup, existed))
    try:
        for source, target in targets:
            os.replace(source, target)
            replaced.append(target)
    except Exception:
        for target, backup, existed in reversed(backups):
            if target in replaced:
                if existed:
                    os.replace(backup, target)
                else:
                    target.unlink()
        raise


def write_archive(sdk, archive, work, jobs=4, expected_source=None, verifier=None):
    info = check_sdk(sdk, expected_source)
    if verifier is None:
        verifier = verify_examples
    archive.parent.mkdir(parents=True, exist_ok=True)
    # Pending files share the final filesystem so publication is an atomic rename.
    with tempfile.TemporaryDirectory(prefix=".ympme-release-", dir=archive.parent) as directory:
        pending = Path(directory) / archive.name
        compress_sdk(sdk, pending)
        with tempfile.TemporaryDirectory(prefix="unpacked SDK ", dir=work) as unpacked:
            extracted = unpack_sdk(pending, Path(unpacked))
            check_sdk(extracted, expected_source)
            verifier(extracted, Path(unpacked), jobs, info)
        value = digest(pending)
        checksum = pending.with_name(pending.name + ".sha256")
        checksum.write_text(value + "  " + archive.name + "\n")
        if expected_source is not None and source_hash() != expected_source:
            raise ValueError("Sources changed during packaging; retry from a consistent snapshot")
        publish_pair(pending, archive, checksum)
    return value


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--release", action="store_true", help="Write releases/ instead of build/archives/")
    parser.add_argument("--rebuild", action="store_true", help="Rebuild/verify build/YMPME_libs first")
    parser.add_argument("--jobs", type=int, default=4)
    parser.add_argument("--gre-sdk", type=Path, default=ROOT.parent / "0502_YMGRE/build/YMGRE_libs",
                        help="Paired GRE SDK used by --rebuild for shared-geometry verification")
    args = parser.parse_args(argv)
    if args.jobs < 1:
        parser.error("--jobs must be positive")
    for command in ("cmake", "ctest"):
        if not shutil.which(command):
            parser.error(f"Missing build dependency: {command}")
    if args.rebuild:
        subprocess.run([sys.executable, ROOT / "sdk/build.py", "--jobs", str(args.jobs),
                        "--gre-sdk", args.gre_sdk.resolve()], cwd=ROOT, check=True)
    sdk = ROOT / "build/YMPME_libs"
    before = source_hash()
    info = check_sdk(sdk, before)
    archive_dir = ROOT / "releases" if args.release else ROOT / "build/archives"
    archive = archive_dir / f"YMPME_libs-{info['system'].lower()}-{info['machine']}.tar.gz"
    work = ROOT / "build/.cache/release-package"
    work.mkdir(parents=True, exist_ok=True)
    checksum = write_archive(sdk, archive, work, args.jobs, before)
    print(f"YMPME {info['version']} release ready: {archive}\nSHA256: {checksum}", flush=True)


if __name__ == "__main__":
    main()
