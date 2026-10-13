from __future__ import annotations

import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CPP = (ROOT / "src" / "CodexMicroBle.cpp").read_text(encoding="utf-8")
HEADER = (ROOT / "include" / "CodexMicroBle.h").read_text(encoding="utf-8")


class MultiActionExecutionTests(unittest.TestCase):
    def test_configuration_loads_all_four_multi_action_targets(self) -> None:
        self.assertIn("std::vector<MultiActionDefinition>", HEADER)
        self.assertNotIn("kMaxMultiActions", HEADER)
        self.assertNotIn("tappingTermMs > 2000", CPP)
        self.assertIn("Parse the complete Work axis document", CPP)
        self.assertNotIn('filter["multiActions"] = true', CPP)
        self.assertRegex(CPP, r'document\["multiActions"\].*?as<JsonArrayConst>')
        self.assertIn("candidateMultiActions.reserve(multiActions.size())", CPP)
        for field in ("kcOnTap", "kcOnHold", "kcOnDoubleTap", "kcOnTapHold", "tt"):
            self.assertIn(f'sourceMulti["{field}"]', CPP)

    def test_physical_multi_actions_use_a_non_blocking_state_machine(self) -> None:
        self.assertIn("multiActionStates_", HEADER)
        self.assertIn("void processMultiActions()", HEADER)
        self.assertNotIn("kMultiHoldMs", HEADER)
        scheduler = re.search(
            r"void CodexMicroBle::processMultiActions\(\).*?\n\}(?=\n\n)",
            CPP,
            re.DOTALL,
        )
        self.assertIsNotNone(scheduler)
        self.assertNotIn("delay(", scheduler.group(0))
        self.assertIn("releasedAtMs", scheduler.group(0))
        self.assertIn("tappingTermMs", scheduler.group(0))
        self.assertIn("definition->tappingTermMs", scheduler.group(0))

    def test_tap_double_hold_and_tap_hold_have_distinct_dispatches(self) -> None:
        for target in ("tapToken", "doubleTapToken", "holdToken", "tapHoldToken"):
            self.assertRegex(CPP, rf"dispatchResolvedToken\([^;]*{target}")

    def test_multi_actions_are_claimed_only_when_the_definition_exists(self) -> None:
        self.assertIn('parseTokenId(token, "KA_M", multiActionId)', CPP)
        self.assertIn("configuredMultiActionCount_", CPP)


if __name__ == "__main__":
    unittest.main()
