from __future__ import annotations

import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CPP = (ROOT / "src" / "CodexMicroBle.cpp").read_text(encoding="utf-8")
HEADER = (ROOT / "include" / "CodexMicroBle.h").read_text(encoding="utf-8")


class MacroExecutionTests(unittest.TestCase):
    def test_configuration_loads_all_macro_definitions_without_product_cap(self) -> None:
        self.assertIn("std::vector<MacroDefinition>", HEADER)
        self.assertIn("std::vector<MacroStep>", HEADER)
        self.assertIn("std::deque<QueuedAction>", HEADER)
        self.assertNotIn("kMaxMacros", HEADER)
        self.assertNotIn("kMaxMacroSteps", HEADER)
        self.assertNotIn("delayMs > 60000", CPP)
        self.assertIn("Parse the complete Work axis document", CPP)
        self.assertNotIn('filter["macros"] = true', CPP)
        self.assertRegex(CPP, r'document\["macros"\].*?as<JsonArrayConst>')
        self.assertIn("candidateMacros.reserve(macros.size())", CPP)
        self.assertIn("actionReferenceExists", CPP)
        self.assertIn("configuredMacroCount_ + configuredMultiActionCount_ + 1", CPP)
        self.assertNotIn("depth > 4", CPP)

    def test_macro_steps_are_scheduled_without_blocking_delay(self) -> None:
        self.assertIn("void processActionQueue()", HEADER)
        poll = re.search(
            r"void CodexMicroBle::poll\(\).*?\n\}(?=\n\nvoid CodexMicroBle::setBattery)",
            CPP,
            re.DOTALL,
        )
        self.assertIsNotNone(poll)
        self.assertIn("processActionQueue()", poll.group(0))
        scheduler = re.search(
            r"void CodexMicroBle::processActionQueue\(\).*?\n\}(?=\n\n)",
            CPP,
            re.DOTALL,
        )
        self.assertIsNotNone(scheduler)
        self.assertNotIn("delay(", scheduler.group(0))
        self.assertIn("nextActionAtMs_", scheduler.group(0))

    def test_macro_and_physically_held_keys_share_one_report(self) -> None:
        self.assertIn("macroHeldTokens_", HEADER)
        self.assertIn("void emitCombinedReports()", HEADER)
        combined = re.search(
            r"void CodexMicroBle::emitCombinedReports\(\).*?\n\}(?=\n\n)",
            CPP,
            re.DOTALL,
        )
        self.assertIsNotNone(combined)
        self.assertIn("configuredKeysPressed_", combined.group(0))
        self.assertIn("macroHeldTokens_", combined.group(0))
        self.assertIn("sendKeyboardReport", combined.group(0))
        self.assertIn("sendConsumerReport", combined.group(0))

    def test_host_smart_actions_are_not_claimed_as_device_executed(self) -> None:
        self.assertIn("bool canExecuteDeviceToken(", HEADER)
        self.assertIn('if (token.startsWith("SA_")) return false', CPP)
        self.assertIn('if (action == 1) sendSmartAction(token)', CPP)
        self.assertNotIn('params["device_executed"]', CPP)


if __name__ == "__main__":
    unittest.main()
