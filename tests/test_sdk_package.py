"""Release publication regression: python3 tests/test_sdk_package.py."""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import platform
import sys
import tarfile
import tempfile
import unittest
from unittest.mock import patch

sys.dont_write_bytecode = True
SDK = Path(__file__).resolve().parents[1] / "sdk"
sys.path.insert(0, str(SDK))
spec = importlib.util.spec_from_file_location("ympme_package", SDK / "package.py")
package = importlib.util.module_from_spec(spec)
spec.loader.exec_module(package)


class PublicationTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="ympme-publish-test-")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.sdk = self.root / "build/YMPME_libs"
        self.sdk.mkdir(parents=True)
        self.work = self.root / "build/.cache/release-package"
        self.work.mkdir(parents=True)
        self.releases = self.root / "releases"
        self.releases.mkdir()
        self.name = f"YMPME_libs-{platform.system().lower()}-{platform.machine()}.tar.gz"
        self.archive = self.releases / self.name
        self.archive.write_bytes(b"previous archive")
        self.checksum = self.releases / (self.name + ".sha256")
        self.checksum.write_bytes(b"previous checksum")
        for name in (".ympme-sdk", "LICENSE", "README.md", "cmake/YMPMEConfig.cmake",
                     "examples/CMakeLists.txt", "include/YMPME/YMPME.h",
                     "lib/libympme_index16.a", "lib/libympme_index32.a", "手册.md"):
            path = self.sdk / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text("SDK fixture " + name)
        (self.sdk / "BUILD_INFO.json").write_text(json.dumps({
            "version": "0.4.0", "layout": "unified-dual-index", "index_bits": [16, 32],
            "system": platform.system(), "machine": platform.machine(), "source_sha256": "fixture"}))
        self.inventory()

    def inventory(self):
        lines = [package.digest(p) + "  " + p.relative_to(self.sdk).as_posix()
                 for p in sorted(self.sdk.rglob("*"))
                 if p.is_file() and p.name != "checksums.sha256"]
        (self.sdk / "checksums.sha256").write_text("\n".join(lines) + "\n")

    def old_release(self):
        self.assertEqual(self.archive.read_bytes(), b"previous archive")
        self.assertEqual(self.checksum.read_bytes(), b"previous checksum")
        self.assertFalse(list(self.releases.glob(".ympme-release-*")))

    @staticmethod
    def verified(sdk, work, jobs, info):
        package.check_sdk(sdk)
        if sdk.name != "YMPME_libs" or info["version"] != "0.4.0":
            raise AssertionError("Wrong archive layout/version")

    def publish(self, verifier=None):
        return package.write_archive(self.sdk, self.archive, self.work,
                                     verifier=verifier or self.verified)

    def test_archive_root_checksum_and_extracted_inventory(self):
        value = self.publish()
        self.assertEqual(value, hashlib.sha256(self.archive.read_bytes()).hexdigest())
        self.assertEqual(self.checksum.read_text(), value + "  " + self.name + "\n")
        dest = self.root / "unpacked"; dest.mkdir()
        sdk = package.unpack_sdk(self.archive, dest)
        self.assertEqual(package.check_sdk(sdk)["version"], "0.4.0")
        with tarfile.open(self.archive) as bundle:
            self.assertIn("YMPME_libs/手册.md", bundle.getnames())
            self.assertFalse(any(m.issym() or m.islnk() for m in bundle.getmembers()))

    def test_same_sdk_produces_same_archive_bytes(self):
        first = self.publish()
        os.utime(self.sdk / "README.md", (12345, 12345))
        second = self.publish()
        self.assertEqual(first, second)

    def test_local_archive_does_not_change_release(self):
        with patch.object(package, "ROOT", self.root), patch.object(package, "source_hash", return_value="fixture"), \
             patch.object(package, "verify_examples", self.verified):
            package.main([])
        self.old_release()
        self.assertTrue((self.root / "build/archives" / self.name).is_file())

    def test_bad_checksum_preserves_release(self):
        (self.sdk / "README.md").write_text("modified after SDK validation")
        with self.assertRaisesRegex(ValueError, "checksum mismatch"):
            self.publish()
        self.old_release()

    def test_failed_extracted_consumer_preserves_release(self):
        def fail(*args):
            raise RuntimeError("injected consumer failure")
        with self.assertRaisesRegex(RuntimeError, "consumer failure"):
            self.publish(fail)
        self.old_release()

    def test_compression_failure_preserves_release(self):
        with patch.object(package.tarfile, "open", side_effect=OSError("injected compression failure")):
            with self.assertRaisesRegex(OSError, "compression failure"):
                self.publish()
        self.old_release()

    def test_checksum_publication_failure_restores_archive(self):
        replace = os.replace
        def fail_checksum(source, target):
            if Path(source).name == self.name + ".sha256":
                raise OSError("injected checksum publication failure")
            return replace(source, target)
        with patch.object(package.os, "replace", side_effect=fail_checksum):
            with self.assertRaisesRegex(OSError, "publication failure"):
                self.publish()
        self.old_release()

    def test_stale_sources_refused(self):
        with self.assertRaisesRegex(ValueError, "does not match"):
            package.check_sdk(self.sdk, "changed-sources")
        self.old_release()

    def test_unsafe_archive_refused(self):
        malicious = self.root / "unsafe.tar.gz"
        with tarfile.open(malicious, "w:gz") as bundle:
            member = tarfile.TarInfo("YMPME_libs/../../outside")
            member.type = tarfile.SYMTYPE
            member.linkname = "/tmp/outside"
            bundle.addfile(member)
        dest = self.root / "unsafe-extracted"; dest.mkdir()
        with self.assertRaisesRegex(ValueError, "Unsafe archive member"):
            package.unpack_sdk(malicious, dest)
        self.assertEqual(list(dest.iterdir()), [])


if __name__ == "__main__":
    unittest.main()
