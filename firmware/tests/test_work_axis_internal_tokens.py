from __future__ import annotations

import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CPP = (ROOT / "src" / "CodexMicroBle.cpp").read_text(encoding="utf-8")
HEADER = (ROOT / "include" / "CodexMicroBle.h").read_text(encoding="utf-8")


class WorkAxisInternalTokenTests(unittest.TestCase):
    def test_internal_tokens_have_one_validator_and_one_dispatcher(self) -> None:
        for token in (
            "KI_BLDW",
            "KI_BLUP",
            "KI_CUSB",
            "KI_CS_SHOW",
            "KI_CS_SHOW_TMP",
            "KI_CS_HIDE",
            "KI_CS_TOGGLE",
            "KI_X",
        ):
            self.assertIn(f'"{token}"', CPP)
        for prefix in ("KI_LS", "KI_LM", "KI_PS", "KI_CBT", "KI_CBTP"):
            self.assertIn(f'parseTokenId(token, "{prefix}"', CPP)
        self.assertIn("bool isValidInternalToken", CPP)
        self.assertIn("bool CodexMicroBle::handleInternalToken", CPP)
        self.assertIn("handleInternalToken(token, action", CPP)
        self.assertIn("isValidInternalToken(token) || isValidCodexVendorToken(token)", CPP)

    def test_codex_vendor_tokens_cover_agents_actions_and_encoder(self) -> None:
        validator = re.search(
            r"bool isValidCodexVendorToken\(.*?\n\}", CPP, re.DOTALL
        )
        self.assertIsNotNone(validator)
        body = validator.group(0)
        for token in ("ENC_CC", "ENC_CW", "ENC_CLK", '"AG"', '"ACT"'):
            self.assertIn(token, body)
        self.assertIn('value <= 5', body)
        self.assertIn('value >= 6 && value <= 12', body)
        self.assertIn("sendCodexVendorToken", HEADER)
        self.assertIn('params["k"] = raw', CPP)
        self.assertIn('params["act"] = action', CPP)
        self.assertIn('params["ag"]', CPP)
        self.assertNotIn('params["device_executed"]', CPP)
        self.assertNotIn('params["token"]', CPP)

    def test_cheatsheet_and_radial_use_official_notification_contract(self) -> None:
        self.assertIn('sendCheatSheetEvent("kb.cs.show")', CPP)
        self.assertIn('sendCheatSheetEvent("kb.cs.hide")', CPP)
        self.assertIn('"kb.cs.toggle"', CPP)
        self.assertIn('radial["method"] = "kb.radial"', CPP)
        self.assertIn('radialParams["o"]', CPP)
        self.assertIn('radialParams["s"]', CPP)

    def test_fp_function_keycode_is_explicitly_not_saved_as_an_inert_token(self) -> None:
        validator = re.search(
            r"bool isValidInternalToken\(.*?\n\}", CPP, re.DOTALL
        )
        self.assertIsNotNone(validator)
        self.assertIn('if (token == "KI_FP") return false', validator.group(0))
        self.assertIn("fpLayer", CPP)


if __name__ == "__main__":
    unittest.main()
