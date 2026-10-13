from __future__ import annotations

import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CPP = (ROOT / "src" / "CodexMicroBle.cpp").read_text(encoding="utf-8")
HEADER = (ROOT / "include" / "CodexMicroBle.h").read_text(encoding="utf-8")
WORK_AXIS_ASSET = next((ROOT.parents[1] / "work-axis-app" / "dist" / "assets").glob("App-*.js"))
WORK_AXIS_RPC_API = ROOT.parents[1] / "work-axis-app" / "vendor" / "wl-device-kit" / "src" / "wl_rpc_api" / "wl_rpc_api.ts"


class TransactionalFilesystemRpcTests(unittest.TestCase):
    def test_official_chunked_writes_commit_through_a_staging_file(self) -> None:
        self.assertIn('params["completed"]', CPP)
        self.assertIn('path + ".part"', CPP)
        self.assertIn("replaceFsFile(stagingPath, path)", CPP)
        self.assertIn('loadDeviceConfiguration(stagingPath.c_str())', CPP)
        self.assertIn('validateSmartActionsFile(stagingPath.c_str())', CPP)

    def test_failed_chunked_write_is_reported_as_json_rpc_error(self) -> None:
        write_handler = re.search(
            r'if \(strcmp\(method, "fs\.writebin"\).*?\n  \}',
            CPP,
            re.DOTALL,
        )
        self.assertIsNotNone(write_handler)
        self.assertIn("sendError(id, -32603", write_handler.group(0))

    def test_internal_staging_files_are_not_exposed_to_the_app(self) -> None:
        self.assertIn("isInternalFsArtifact", CPP)
        self.assertRegex(CPP, r"if \(isInternalFsArtifact\(name\)\)")

    def test_replace_rpc_uses_backup_and_restores_on_failure(self) -> None:
        self.assertIn('strcmp(method, "fs.replace")', CPP)
        self.assertIn('target + ".bak"', CPP)
        self.assertRegex(CPP, r"SPIFFS\.rename\(target, backup\)")
        self.assertRegex(CPP, r"SPIFFS\.rename\(source, target\)")
        self.assertRegex(CPP, r"SPIFFS\.rename\(backup, target\)")

    def test_mount_recovers_interrupted_known_config_replacements(self) -> None:
        self.assertIn('recoverTransactionalFile("/keymap.json")', CPP)
        self.assertIn('recoverTransactionalFile("/smart_actions.json")', CPP)
        self.assertLess(CPP.index("fsReady = mounted"), CPP.index('recoverTransactionalFile("/keymap.json")'))

    def test_unknown_rpc_is_an_explicit_json_rpc_error(self) -> None:
        self.assertIn("void sendError(", HEADER)
        handler = re.search(
            r"void CodexMicroBle::handleRpc\(.*?\n\}(?=\n\nvoid CodexMicroBle::sendResult)",
            CPP,
            re.DOTALL,
        )
        self.assertIsNotNone(handler)
        self.assertIn('sendError(id, -32601, "method_not_found")', handler.group(0))

    def test_committed_keymap_is_loaded_on_boot_and_by_rpc(self) -> None:
        self.assertIn('loadDeviceConfiguration("/keymap.json")', CPP)
        self.assertIn('strcmp(method, "config.reload")', CPP)
        self.assertIn('resultDoc["loaded"] = configLoaded_', CPP)

    def test_binary_rpc_chunk_limit_matches_official_work_axis_client(self) -> None:
        api = WORK_AXIS_RPC_API.read_text(encoding="utf-8")
        self.assertIn("CHUNK_RAW_BYTE_SIZE = Math.floor", api)
        self.assertIn("* 3 = 3072", api)
        self.assertIn("constexpr size_t kFsReadChunk = 3072", CPP)
        self.assertIn("constexpr size_t kRpcDocumentCapacity = 8192", CPP)
        self.assertIn("DynamicJsonDocument request(kRpcDocumentCapacity)", CPP)
        self.assertIn("DynamicJsonDocument resultDoc(kRpcDocumentCapacity)", CPP)

    def test_failed_keymap_commit_restores_the_committed_runtime_configuration(self) -> None:
        replace_handler = re.search(
            r'if \(strcmp\(method, "fs\.replace"\).*?\n  \}',
            CPP,
            re.DOTALL,
        )
        self.assertIsNotNone(replace_handler)
        self.assertRegex(
            replace_handler.group(0),
            r'if \(target == "/keymap\.json" && !replaced\)\s*\{\s*loadDeviceConfiguration\("/keymap\.json"\)',
        )

    def test_device_status_reports_loaded_profile_and_layer(self) -> None:
        self.assertIn('resultDoc["profile_index"] = activeProfileIndex_', CPP)
        self.assertIn(
            'resultDoc["layer_index"] = static_cast<uint16_t>(activeLayerIndex_ + 1)',
            CPP,
        )
        self.assertIn(
            "layerSelectedIndex = r - 1",
            WORK_AXIS_ASSET.read_text(encoding="utf-8"),
        )
        self.assertIn('resultDoc["config_loaded"] = configLoaded_', CPP)
        self.assertIn('bool loadDeviceConfiguration(', HEADER)

    def test_focused_app_switches_to_a_linked_work_axis_layer(self) -> None:
        handler = re.search(
            r'if \(strcmp\(method, "host\.focused_app"\).*?\n  \}',
            CPP,
            re.DOTALL,
        )
        self.assertIsNotNone(handler)
        source = handler.group(0)
        self.assertIn('document["linkedApps"]', source)
        self.assertIn('layer["linkedAppId"]', source)
        self.assertIn('loadDeviceConfiguration("/keymap.json", matchedProfileId', source)
        self.assertIn('resultDoc["matched"] = switched', source)
        self.assertNotIn('if (strcmp(method, "host.focused_app") == 0) {\n    sendSuccess(id);', source)


if __name__ == "__main__":
    unittest.main()
