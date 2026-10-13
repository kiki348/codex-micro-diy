from __future__ import annotations

import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CPP = (ROOT / "src" / "CodexMicroBle.cpp").read_text(encoding="utf-8")
HEADER = (ROOT / "include" / "CodexMicroBle.h").read_text(encoding="utf-8")


class WorkAxisCompatibilityFixTests(unittest.TestCase):
    def test_firmware_version_meets_codex_micro_feature_floor(self) -> None:
        match = re.search(r'kFirmwareVersion\[\]\s*=\s*"([^"]+)"', CPP)
        self.assertIsNotNone(match)
        version = match.group(1)
        base = tuple(int(part) for part in re.match(r"^(\d+)\.(\d+)\.(\d+)", version).groups())
        self.assertGreaterEqual(base, (0, 5, 0))

    def test_configuration_load_failure_keeps_previous_valid_runtime_state(self) -> None:
        start = CPP.index("bool CodexMicroBle::loadDeviceConfiguration(")
        end = CPP.index("\nvoid CodexMicroBle::handleRpc", start)
        body = CPP[start:end]
        self.assertNotIn("configLoaded_ = false;", body.split("if (!fsReady", 1)[0])
        self.assertIn("configLoaded_ = true;", body)

    def test_cusb_checks_physical_mount_before_transport_preference(self) -> None:
        start = CPP.index('if (token == "KI_CUSB")')
        end = CPP.index("\n  int32_t id = -1;", start)
        body = CPP[start:end]
        self.assertIn("usbMounted_", body)
        self.assertIn("preferredTransport_ = 0;", body)
        self.assertLess(body.index("usbMounted_"), body.index("preferredTransport_ = 0;"))
        self.assertNotIn("if (!transportAvailable(true))", body)

    def test_dispatch_gives_each_physical_edge_one_owner(self) -> None:
        self.assertIn("bool sendConfiguredKey(const char* key, uint8_t action);", HEADER)
        start = CPP.index("void CodexMicroBle::sendKey(")
        end = CPP.index("\nbool CodexMicroBle::sendConfiguredKey(", start)
        body = CPP[start:end]
        self.assertIn("if (!nativeCodexMode_)", body)
        self.assertIn("if (!configLoaded_ || isValidCodexVendorToken(configuredToken)) return;", body)
        self.assertIn("sendConfiguredKey(key, action)", body)
        self.assertIn('message["method"] = "v.oai.hid"', body)
        self.assertNotIn('params["device_executed"]', body)
        self.assertIn('if (isValidInternalToken(token)) return handleInternalToken(token, action);', CPP)

    def test_transient_cheatsheet_click_is_paired(self) -> None:
        start = CPP.index('if (token == "KI_CS_SHOW_TMP")')
        end = CPP.index("\n    } else if (pressed)", start)
        body = CPP[start:end]
        self.assertIn('if (action == 2)', body)
        self.assertIn('sendCheatSheetEvent("kb.cs.show")', body)
        self.assertIn('sendCheatSheetEvent("kb.cs.hide")', body)

    def test_legacy_macro_vendor_tokens_cannot_escape_general_mode(self) -> None:
        start = CPP.index("bool CodexMicroBle::sendCodexVendorToken(")
        end = CPP.index("\nbool CodexMicroBle::switchToLayerIndex(", start)
        self.assertIn("if (!nativeCodexMode_ ||", CPP[start:end])

    def test_held_joystick_waits_for_neutral_after_mode_switch(self) -> None:
        self.assertIn("modeSuppressedJoystick_ = joystickHeld_;", CPP)
        start = CPP.index("void CodexMicroBle::sendJoystick(")
        end = CPP.index("\nbool CodexMicroBle::joystickSectorContains(", start)
        body = CPP[start:end]
        self.assertLess(body.index("if (modeSuppressedJoystick_)"), body.index("applyConfiguredJoystick("))
        self.assertIn("if (!joystickHeld_) modeSuppressedJoystick_ = false;", body)


if __name__ == "__main__":
    unittest.main()
