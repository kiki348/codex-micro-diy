from __future__ import annotations

import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CPP = (ROOT / "src" / "CodexMicroBle.cpp").read_text(encoding="utf-8")


class SpiffsInitializationTests(unittest.TestCase):
    def test_mounts_named_partition_without_formatting_on_boot(self) -> None:
        """Mount the declared partition, but never format it implicitly."""
        self.assertRegex(
            CPP,
            r"SPIFFS\.begin\(\s*false\s*,\s*\"/spiffs\"\s*,\s*10\s*,\s*\"spiffs\"\s*\)",
        )
        self.assertNotRegex(CPP, r"SPIFFS\.begin\(\s*true\b")
        self.assertNotIn("SPIFFS.format(", CPP)

    def test_filesystem_ready_only_after_mount_result(self) -> None:
        """The readiness flag must be derived from the mount call."""
        match = re.search(
            r"bool\s+mountFilesystem\s*\(\s*\)\s*\{(?P<body>.*?)\n\}",
            CPP,
            re.DOTALL,
        )
        self.assertIsNotNone(match, "mountFilesystem() should own fsReady")
        body = match.group("body")
        self.assertIn("const bool mounted = SPIFFS.begin", body)
        self.assertIn("fsReady = mounted", body)
        self.assertLess(
            CPP.index("mountFilesystem();"),
            CPP.index("USB.begin()"),
            "mount attempt must finish before the USB stack is started",
        )

    def test_all_file_rpc_methods_are_present_and_mount_gated(self) -> None:
        for method in ("fs.list", "fs.readbin", "fs.writebin", "fs.delete"):
            self.assertIn(f'"{method}"', CPP)
        self.assertGreaterEqual(CPP.count("fsReady"), 5)
        self.assertIn("filesystem_unavailable", CPP)


if __name__ == "__main__":
    unittest.main()
