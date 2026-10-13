from __future__ import annotations

import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
WORKSPACE = ROOT.parents[1]
CPP = (ROOT / "src" / "CodexMicroBle.cpp").read_text(encoding="utf-8")
MAIN = (ROOT / "src" / "main.cpp").read_text(encoding="utf-8")
HEADER = (ROOT / "include" / "CodexMicroBle.h").read_text(encoding="utf-8")
WORK_AXIS_ASSET = WORKSPACE / "work-axis-app-v1" / "dist" / "assets" / "device_config_data-O6ezP1mk.js"
MODEL = WORK_AXIS_ASSET.read_text(encoding="utf-8")


class CompositeConfiguredHidTests(unittest.TestCase):
    def test_descriptor_keeps_vendor_and_adds_keyboard_and_consumer_reports(self) -> None:
        self.assertIn("kKeyboardReportId = 1", HEADER)
        self.assertIn("kConsumerReportId = 2", HEADER)
        self.assertIn("kReportId = 6", HEADER)
        self.assertIn("kSystemReportId = 3", HEADER)
        for report_id in ("0x85, 0x01", "0x85, 0x02", "0x85, 0x03", "0x85, 0x06"):
            self.assertIn(report_id, CPP)
        self.assertIn("keyboardInput_ = hid_->inputReport(kKeyboardReportId)", CPP)
        self.assertIn("consumerInput_ = hid_->inputReport(kConsumerReportId)", CPP)
        self.assertIn("systemInput_ = hid_->inputReport(kSystemReportId)", CPP)
        self.assertIn("vendorInput_ = hid_->inputReport(kReportId)", CPP)

    def test_keyboard_report_advertises_extended_work_axis_usages(self) -> None:
        self.assertIn("0x29, 0xA4", CPP)
        self.assertIn("0x25, 0xA4", CPP)

    def test_every_visible_basic_key_token_has_a_firmware_output_entry(self) -> None:
        ui_tokens = set(re.findall(r'keycode:\s*"(KC_[A-Z0-9_]+)"', MODEL))
        firmware_tokens = set(re.findall(r'\{"(KC_[A-Z0-9_]+)"\s*,', CPP))
        self.assertTrue(ui_tokens, "Work axis must expose basic key tokens")
        self.assertEqual(ui_tokens - firmware_tokens, set())

    def test_loaded_layout_uses_the_exact_physical_to_logical_mapping(self) -> None:
        self.assertIn("kPhysicalToLogicalToken[12]", CPP)
        self.assertRegex(
            CPP,
            r"kPhysicalToLogicalToken\[12\]\s*=\s*\{\s*0,\s*1,\s*5,\s*4,\s*3,\s*2,\s*9,\s*8,\s*7,\s*6,\s*12,\s*10",
        )
        self.assertIn("Parse the complete Work axis document", CPP)
        self.assertIn(
            "deserializeJson(document, file, DeserializationOption::NestingLimit(kJsonNestingLimit))",
            CPP,
        )
        self.assertNotIn('filter["profiles"][0]', CPP)

    def test_custom_board_has_twelve_keys_and_skips_official_act11(self) -> None:
        expected = [
            "AG00", "AG01", "AG05", "AG04", "AG03", "AG02",
            "ACT09", "ACT08", "ACT07", "ACT06", "ACT12", "ACT10",
        ]
        service_order = re.search(
            r"kPhysicalInputNames\[12\]\s*=\s*\{(.*?)\};", CPP, re.DOTALL
        )
        scanner_order = re.search(
            r"kKeyBindings\[12\]\s*=\s*\{(.*?)\};", MAIN, re.DOTALL
        )
        self.assertIsNotNone(service_order)
        self.assertIsNotNone(scanner_order)
        self.assertEqual(re.findall(r'"([A-Z0-9]+)"', service_order.group(1)), expected)
        self.assertEqual(re.findall(r'"([A-Z0-9]+)"', scanner_order.group(1)), expected)
        self.assertNotRegex(CPP, r'kPhysicalInputNames\[12\][^;]*"ACT11"')

    def test_shift_register_bits_match_the_pcb_pin_order(self) -> None:
        mapping = re.search(
            r"kRawBitBySwitch\[12\]\s*=\s*\{(.*?)\};", MAIN, re.DOTALL
        )
        self.assertIsNotNone(mapping)
        self.assertEqual(
            [int(value) for value in re.findall(r"\d+", mapping.group(1))],
            [4, 5, 6, 7, 0, 1, 2, 3, 8, 9, 10, 11],
        )
        self.assertIn("SW1=E and SW2=F", MAIN)

    def test_general_key_and_codex_vendor_event_are_mutually_exclusive(self) -> None:
        start = CPP.index("void CodexMicroBle::sendKey(")
        end = CPP.index("\nbool CodexMicroBle::sendConfiguredKey(", start)
        body = CPP[start:end]

        self.assertIn("sendConfiguredKey(key, action)", body)
        self.assertIn("isValidCodexVendorToken(configuredToken)", body)
        self.assertIn("return;", body)
        self.assertLess(
            body.index("isValidCodexVendorToken(configuredToken)"),
            body.index("sendConfiguredKey(key, action)"),
        )
        self.assertLess(
            body.index("sendConfiguredKey(key, action)"),
            body.index('message["method"] = "v.oai.hid"'),
        )

    def test_codex_vendor_event_uses_the_official_payload_fields(self) -> None:
        start = CPP.index("void CodexMicroBle::sendKey(")
        end = CPP.index("\nbool CodexMicroBle::sendConfiguredKey(", start)
        body = CPP[start:end]

        self.assertIn('params["k"]', body)
        self.assertIn('params["act"]', body)
        self.assertIn('params["ag"]', body)
        self.assertNotIn('params["token"]', body)
        self.assertNotIn('params["device_executed"]', body)

    def test_configured_outputs_share_usb_and_ble_report_paths(self) -> None:
        self.assertIn("bool CodexMicroBle::sendConfiguredKey(", CPP)
        self.assertIn("sendConfiguredKey(key, action)", CPP)
        self.assertIn("usbHid_->SendReport(kKeyboardReportId", CPP)
        self.assertIn("keyboardInput_->notify()", CPP)
        self.assertIn("usbHid_->SendReport(kConsumerReportId", CPP)
        self.assertIn("consumerInput_->notify()", CPP)
        self.assertIn("usbHid_->SendReport(kSystemReportId", CPP)
        self.assertIn("systemInput_->notify()", CPP)


if __name__ == "__main__":
    unittest.main()
