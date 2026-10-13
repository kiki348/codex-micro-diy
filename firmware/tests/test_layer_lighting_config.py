from __future__ import annotations

import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CPP = (ROOT / "src" / "CodexMicroBle.cpp").read_text(encoding="utf-8")


class LayerLightingConfigurationTests(unittest.TestCase):
    def test_active_layer_is_selected_by_stable_id(self) -> None:
        self.assertIn("Parse the complete Work axis document", CPP)
        self.assertIn(
            "document, file, DeserializationOption::NestingLimit(kJsonNestingLimit)",
            CPP,
        )
        self.assertNotIn('filter["profiles"][0]', CPP)
        self.assertIn('selectedProfile["activeLayerId"]', CPP)
        self.assertIn('(layer["id"] | -1) == requestedLayerId', CPP)
        self.assertIn('activeLayerIndex_ = selectedLayerIndex', CPP)

    def test_official_layer_lighting_fields_are_loaded(self) -> None:
        self.assertIn('selectedLayer["lights"]', CPP)
        for zone in ("backlight", "underglow"):
            self.assertIn(f'lights["{zone}"]', CPP)
        for effect in ("off", "solid", "snake", "rainbow", "breath", "gradient"):
            self.assertIn(f'"{effect}"', CPP)

    def test_committed_lighting_updates_both_physical_zones(self) -> None:
        self.assertIn('thread = candidateBacklight', CPP)
        self.assertIn('lighting.ambient = candidateUnderglow', CPP)
        self.assertIn('++state_.threadsRevision', CPP)
        self.assertIn('++state_.ambientRevision', CPP)

    def test_official_lighting_preview_updates_hardware_without_persisting(self) -> None:
        self.assertIn('strcmp(method, "lights.preview")', CPP)
        self.assertIn('parseLightingConfig(params.as<JsonObjectConst>()', CPP)
        self.assertIn('thread = previewBacklight', CPP)
        self.assertIn('lighting.ambient = previewUnderglow', CPP)
        self.assertNotIn('writeFsChunk("/keymap.json"', CPP)

    def test_codex_runtime_lighting_does_not_overwrite_user_ambient_file(self) -> None:
        start = CPP.index("void CodexMicroBle::updateAmbientLighting(")
        end = CPP.index("\nvoid CodexMicroBle::requestAmbientPersist(", start)
        body = CPP[start:end]

        self.assertIn("++lighting.ambientRevision", body)
        self.assertIn("nativeCodexMode_ ? state_ : codexLighting_", body)
        self.assertNotIn("requestAmbientPersist", body)

    def test_codex_notifications_preserve_user_preset_lighting(self) -> None:
        start = CPP.index('if (strcmp(method, "v.oai.thstatus") == 0)')
        end = CPP.index('if (strcmp(method, "lights.preview") == 0)', start)
        body = CPP[start:end]
        self.assertIn("updateThreadLighting(", body)
        self.assertIn("updateAmbientLighting(", body)
        self.assertNotIn("requestAmbientPersist", body)
        self.assertEqual(body.count("sendSuccess(id)"), 2)

    def test_mode_switch_copies_lighting_without_copying_transport_state(self) -> None:
        start = CPP.index("void CodexMicroBle::switchLightingMode(")
        end = CPP.index("void CodexMicroBle::sendKey(", start)
        body = CPP[start:end]
        for field in ("threads", "ambient", "keys", "ambientSyncThread"):
            self.assertIn(f"target.{field} = source.{field}", body)
        self.assertNotIn("target = source", body)
        self.assertNotIn("state_.connected =", body)
        self.assertIn("++state_.threadsRevision", body)
        self.assertIn("++state_.ambientRevision", body)

    def test_deferred_preview_save_uses_general_bank_after_mode_switch(self) -> None:
        start = CPP.index("void CodexMicroBle::persistAmbientToFlash(")
        end = CPP.index("void CodexMicroBle::restoreAmbientFromFlash(", start)
        self.assertIn("nativeCodexMode_ ? generalLighting_ : state_", CPP[start:end])

    def test_saved_layer_lighting_takes_precedence_over_stale_preview(self) -> None:
        self.assertIn("if (!candidateLightingConfigured) restoreAmbientFromFlash();", CPP)


if __name__ == "__main__":
    unittest.main()
