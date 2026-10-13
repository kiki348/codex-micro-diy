from __future__ import annotations

import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CPP = (ROOT / "src" / "CodexMicroBle.cpp").read_text(encoding="utf-8")
HEADER = (ROOT / "include" / "CodexMicroBle.h").read_text(encoding="utf-8")


class SmartActionNotificationTests(unittest.TestCase):
    def test_official_smart_action_schema_is_validated_before_commit(self) -> None:
        self.assertIn("bool validateSmartActionsFile(", HEADER)
        self.assertIn('document["smartActions"].as<JsonObjectConst>()', CPP)
        for action_type in ("TEXT_STEP", "CMD_STEP", "URL_STEP", "APP_STEP"):
            self.assertIn(f'"{action_type}"', CPP)
        self.assertRegex(
            CPP,
            r'(?s)path == "/smart_actions\.json".*?validateSmartActionsFile\(stagingPath\.c_str\(\)\)',
        )

    def test_smart_actions_emit_the_methods_consumed_by_work_axis(self) -> None:
        for method in (
            "kb.sa.inserttext",
            "kb.sa.exec",
            "kb.sa.openurl",
            "kb.sa.openapp",
        ):
            self.assertIn(f'"{method}"', CPP)
        for parameter in ("text", "cmd", "url", "path"):
            self.assertIn(f'parameter = "{parameter}"', CPP)

    def test_physical_keys_encoders_macros_and_multi_actions_resolve_sa_tokens(self) -> None:
        sender = re.search(
            r"bool CodexMicroBle::sendConfiguredKey\(.*?\n\}(?=\n\nbool CodexMicroBle::canExecuteDeviceToken)",
            CPP,
            re.DOTALL,
        )
        self.assertIsNotNone(sender)
        self.assertGreaterEqual(sender.group(0).count('token.startsWith("SA_")'), 2)
        queued = re.search(
            r"void CodexMicroBle::applyQueuedAction\(.*?\n\}(?=\n\nbool CodexMicroBle::dispatchResolvedToken)",
            CPP,
            re.DOTALL,
        )
        self.assertIsNotNone(queued)
        self.assertIn("sendSmartAction(token)", queued.group(0))

    def test_smart_actions_remain_host_only(self) -> None:
        self.assertIn('if (token.startsWith("SA_")) return false', CPP)

    def test_keymap_rejects_smart_action_references_missing_from_committed_file(self) -> None:
        self.assertIn("bool smartActionReferenceExists(", HEADER)
        self.assertIn('document["smartActions"][token].as<JsonObjectConst>()', CPP)
        self.assertRegex(
            CPP,
            r"(?s)if \(isValidSmartActionToken\(token\)\) \{.*?return smartActionReferenceExists\(token\);",
        )

    def test_smart_action_storage_has_no_product_level_payload_cap(self) -> None:
        self.assertNotIn("kMaxSmartActionsFileBytes", CPP)
        self.assertIn("fileSize > (SIZE_MAX - 4096) / 2", CPP)


if __name__ == "__main__":
    unittest.main()
