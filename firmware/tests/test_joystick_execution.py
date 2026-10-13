from __future__ import annotations

import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CPP = (ROOT / "src" / "CodexMicroBle.cpp").read_text(encoding="utf-8")
HEADER = (ROOT / "include" / "CodexMicroBle.h").read_text(encoding="utf-8")


class JoystickExecutionTests(unittest.TestCase):
    def test_radial_sectors_are_loaded_from_the_active_layer(self) -> None:
        self.assertIn("Parse the complete Work axis document", CPP)
        self.assertIn('selectedLayer["layout"]["joystick"]', CPP)
        self.assertIn(
            "document, file, DeserializationOption::NestingLimit(kJsonNestingLimit)",
            CPP,
        )
        self.assertNotIn('filter["profiles"][0]', CPP)
        self.assertIn("std::vector<JoystickSector>", HEADER)
        self.assertNotIn("kMaxJoystickSectors", HEADER)
        self.assertIn("candidateJoystickSectors.reserve(sectors.size())", CPP)
        self.assertIn('joystick["type"]', CPP)
        for field in ("k", "a1", "a2"):
            self.assertIn(f'sourceSector["{field}"]', CPP)

    def test_sector_matching_supports_ranges_that_wrap_through_zero(self) -> None:
        matcher = re.search(
            r"bool CodexMicroBle::joystickSectorContains\(.*?\n\}(?=\n\n)",
            CPP,
            re.DOTALL,
        )
        self.assertIsNotNone(matcher)
        self.assertIn("start <= end", matcher.group(0))
        self.assertIn("angle >= start || angle < end", matcher.group(0))

    def test_moving_between_sectors_releases_before_pressing(self) -> None:
        executor = re.search(
            r"void CodexMicroBle::applyConfiguredJoystick\(.*?\n\}(?=\n\n)",
            CPP,
            re.DOTALL,
        )
        self.assertIsNotNone(executor)
        body = executor.group(0)
        release = body.index("dispatchResolvedToken(previous.token, 0, 0, 12)")
        press = body.index("dispatchResolvedToken(next.token, 1, 0, 12)")
        self.assertLess(release, press)


if __name__ == "__main__":
    unittest.main()
