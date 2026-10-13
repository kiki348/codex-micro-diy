// SPDX-License-Identifier: MIT
// Copyright (c) 2026 imliubo

#include "CodexMicroBle.h"

#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLESecurity.h>
#include <BLEUtils.h>
#include <FS.h>
#include <SPIFFS.h>
#include <USB.h>
#include <USBHID.h>
#include "esp32-hal-tinyusb.h"
#include <mbedtls/sha1.h>
#include <algorithm>
#include <cmath>
#include <vector>

#include "esp_gap_ble_api.h"

// Global (external linkage) so main.cpp can do lazy SPIFFS init.
bool fsReady = false;
// The LED renderer owns the device-wide brightness value. Internal Work axis
// brightness keycodes call this bridge instead of maintaining a second,
// disconnected brightness setting in the HID service.
extern void codexMicroAdjustGlobalBrightness(int delta);
extern void codexMicroInputSnapshot(JsonObject out);
extern void codexMicroShowLayerIndicator(uint32_t color);

namespace {

// These identity values and the HID descriptor are copied unchanged from
// codex-micro-4-core2 commit 2ee23a4ab696f94bb78d250f28cc4a9b879ba079.
constexpr char kDeviceName[] = "Codex Micro";
constexpr char kManufacturer[] = "Work Louder";
// Work axis enables Codex Micro smart actions and cheat-sheet controls from
// firmware 0.5.0 onward. Build metadata distinguishes this adapted image
// while semver feature checks still coerce to 0.5.0.
constexpr char kFirmwareVersion[] = "0.5.0+work-axis.18";
// Work axis wl-device-kit sends binary RPC chunks as 3072 raw bytes, which
// become exactly 4096 base64 characters on the wire. Keep the device-side
// limit aligned with that official client contract so large keymap/action
// writes do not fail after the first chunk.
constexpr size_t kFsReadChunk = 3072;
constexpr size_t kRpcDocumentCapacity = 8192;

// The official Work axis app stores the key matrix under profiles[0].layers[0]
// .layout.joystick.sectors[...], which nests 11 container levels deep:
// root(1) params(2) data(3) profiles(4) profile(5) layers(6) layer(7)
// layout(8) joystick(9) sectors(10) sector(11). ArduinoJson's default nesting
// limit is 10, so any fs.write carrying at least one radial sector failed to
// deserialize, was silently dropped in onOutput, and the client saw a bare
// timeout while the device stayed perfectly responsive to small requests.
// Raise the limit well above the documented official document shape.
constexpr uint8_t kJsonNestingLimit = 32;

String normalizeFsPath(const char* raw) {
  String path = raw == nullptr ? String() : String(raw);
  while (path.startsWith("/")) path.remove(0, 1);
  if (path.isEmpty() || path.indexOf("..") >= 0 || path.indexOf('\\') >= 0 || path.indexOf("//") >= 0) return String();
  return "/" + path;
}

bool isInternalFsArtifact(const String& path) {
  return path.endsWith(".part") || path.endsWith(".bak");
}

String sha1File(const String& path) {
  File file = SPIFFS.open(path, FILE_READ);
  if (!file) return String();
  mbedtls_sha1_context context;
  mbedtls_sha1_init(&context);
  if (mbedtls_sha1_starts_ret(&context) != 0) { file.close(); mbedtls_sha1_free(&context); return String(); }
  uint8_t buffer[512];
  while (file.available()) {
    const size_t count = file.read(buffer, sizeof(buffer));
    if (count == 0 || mbedtls_sha1_update_ret(&context, buffer, count) != 0) { file.close(); mbedtls_sha1_free(&context); return String(); }
  }
  uint8_t digest[20];
  const bool ok = mbedtls_sha1_finish_ret(&context, digest) == 0;
  mbedtls_sha1_free(&context);
  file.close();
  if (!ok) return String();
  String result;
  char hex[3];
  for (uint8_t byte : digest) { snprintf(hex, sizeof(hex), "%02x", byte); result += hex; }
  return result;
}

// Compute the SHA-1 of an ALREADY-OPEN file handle.
//
// sha1File() re-opens the path, which fails while the same file is held open by
// a directory iteration (root.openNextFile()): SPIFFS refuses the second open
// and the checksum silently degrades to an empty string. The desktop app treats
// an empty checksum as "file has no checksum, skipping it", so it never reads
// keymap.json and the device looks permanently unreadable. Hashing the handle
// we already own avoids the second open entirely.
String sha1OfOpen(File& file) {
  if (!file) return String();
  mbedtls_sha1_context context;
  mbedtls_sha1_init(&context);
  if (mbedtls_sha1_starts_ret(&context) != 0) { mbedtls_sha1_free(&context); return String(); }
  if (!file.seek(0)) { mbedtls_sha1_free(&context); return String(); }
  uint8_t buffer[512];
  while (file.available()) {
    const size_t count = file.read(buffer, sizeof(buffer));
    if (count == 0 || mbedtls_sha1_update_ret(&context, buffer, count) != 0) {
      mbedtls_sha1_free(&context);
      return String();
    }
  }
  uint8_t digest[20];
  const bool ok = mbedtls_sha1_finish_ret(&context, digest) == 0;
  mbedtls_sha1_free(&context);
  if (!ok) return String();
  String result;
  char hex[3];
  for (uint8_t byte : digest) { snprintf(hex, sizeof(hex), "%02x", byte); result += hex; }
  return result;
}

bool decodeBase64(const String& input, std::vector<uint8_t>& output) {
  auto value = [](char c) -> int { if (c >= 'A' && c <= 'Z') return c - 'A'; if (c >= 'a' && c <= 'z') return c - 'a' + 26; if (c >= '0' && c <= '9') return c - '0' + 52; if (c == '+') return 62; if (c == '/') return 63; return -1; };
  output.clear();
  int accumulator = 0;
  int bits = 0;
  for (size_t index = 0; index < input.length(); ++index) {
    const char c = input[index];
    if (c == '=') break;
    const int decoded = value(c);
    if (decoded < 0) return false;
    accumulator = (accumulator << 6) | decoded;
    bits += 6;
    if (bits >= 8) { bits -= 8; output.push_back(static_cast<uint8_t>((accumulator >> bits) & 0xFF)); }
  }
  return true;
}

String encodeBase64(const uint8_t* data, size_t length) {
  static const char table[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  String output;
  output.reserve(((length + 2) / 3) * 4);
  for (size_t index = 0; index < length; index += 3) {
    const uint32_t a = data[index];
    const uint32_t b = index + 1 < length ? data[index + 1] : 0;
    const uint32_t c = index + 2 < length ? data[index + 2] : 0;
    const uint32_t value = (a << 16) | (b << 8) | c;
    output += table[(value >> 18) & 0x3F];
    output += table[(value >> 12) & 0x3F];
    output += index + 1 < length ? table[(value >> 6) & 0x3F] : '=';
    output += index + 2 < length ? table[value & 0x3F] : '=';
  }
  return output;
}

bool writeFsChunk(const String& path, const std::vector<uint8_t>& bytes,
                  size_t offset, bool append) {
  if (bytes.empty()) return false;
  if (offset > 0 && !append) return false;

  // SPIFFS can transiently return zero from fwrite immediately after a file
  // is removed/created (the Arduino examples and field reports both show
  // this). A single attempt made a valid Work axis upload fail randomly and
  // left a zero-byte staging file. Retry the operation while keeping the
  // offset invariant; the first chunk is recreated with FILE_WRITE so a
  // partial/zero-byte attempt cannot poison the next attempt.
  constexpr uint8_t kWriteAttempts = 4;
  for (uint8_t attempt = 0; attempt < kWriteAttempts; ++attempt) {
    if (offset == 0) {
      if (SPIFFS.exists(path) && !SPIFFS.remove(path)) {
        delay(8);
        continue;
      }
    }

    const char* mode = offset == 0 ? FILE_WRITE : FILE_APPEND;
    File file = SPIFFS.open(path, mode);
    if (!file) {
      delay(8);
      continue;
    }
    // Arduino-ESP32 2.x VFSFileImpl leaves _stat uninitialized when fopen
    // creates a new file. size() before its first write therefore is not a
    // valid zero-length check. FILE_WRITE already establishes offset zero.
    // Existing append targets have a stat result; retain their offset guard.
    if (offset > 0 && file.size() != offset) {
      file.close();
      delay(8);
      continue;
    }
    const size_t written = file.write(bytes.data(), bytes.size());
    file.flush();
    file.close();
    if (written == bytes.size()) return true;

    // Do not append after a partial first chunk. Subsequent chunks retain the
    // established prefix and report failure so the host can restart safely.
    if (offset == 0) SPIFFS.remove(path);
    delay(8);
  }
  return false;
}

bool recoverTransactionalFile(const String& target) {
  const String backup = target + ".bak";
  if (!SPIFFS.exists(backup)) return true;
  if (SPIFFS.exists(target)) return SPIFFS.remove(backup);
  return SPIFFS.rename(backup, target);
}

bool replaceFsFile(const String& source, const String& target) {
  if (!fsReady || source.isEmpty() || target.isEmpty() || source == target ||
      !SPIFFS.exists(source)) {
    return false;
  }

  const String backup = target + ".bak";
  if (SPIFFS.exists(backup) && !SPIFFS.remove(backup)) return false;
  const bool hadTarget = SPIFFS.exists(target);
  if (hadTarget && !SPIFFS.rename(target, backup)) return false;
  if (!SPIFFS.rename(source, target)) {
    if (hadTarget) SPIFFS.rename(backup, target);
    return false;
  }
  if (hadTarget && SPIFFS.exists(backup)) SPIFFS.remove(backup);
  return true;
}

bool mountFilesystem() {
  // Never turn a transient mount failure into an automatic data wipe. A
  // damaged filesystem stays unavailable for explicit recovery, preserving
  // the user's keymap, actions and lighting whenever the bytes are salvageable.
  const bool mounted = SPIFFS.begin(false, "/spiffs", 10, "spiffs");
  fsReady = mounted;
  if (fsReady) {
    recoverTransactionalFile("/keymap.json");
    recoverTransactionalFile("/smart_actions.json");
    SPIFFS.remove("/keymap.json.part");
    SPIFFS.remove("/smart_actions.json.part");
  }
  Serial.printf("SPIFFS mount result=%s total=%u used=%u\n",
                fsReady ? "ready" : "unavailable",
                static_cast<unsigned>(fsReady ? SPIFFS.totalBytes() : 0),
                static_cast<unsigned>(fsReady ? SPIFFS.usedBytes() : 0));
  return mounted;
}

constexpr size_t kPayloadSize = 61;
constexpr size_t kReportBodySize = 63;
constexpr uint16_t kRequiredMtu = kReportBodySize + 3;
constexpr uint16_t kLocalMtu = 517;
constexpr uint16_t kUsbFirmwareVersion = 0x0100;
constexpr uint32_t kUsbReportTimeoutMs = 20;

constexpr uint16_t swapBytes(uint16_t value) {
  return static_cast<uint16_t>((value << 8) | (value >> 8));
}

float clampUnit(float value) {
  if (value < 0.0f) {
    return 0.0f;
  }
  if (value > 1.0f) {
    return 1.0f;
  }
  return value;
}

constexpr const char* kPhysicalInputNames[12] = {
    "AG00", "AG01", "AG05", "AG04", "AG03", "AG02",
    "ACT09", "ACT08", "ACT07", "ACT06", "ACT12", "ACT10",
};

constexpr uint8_t kPhysicalToLogicalToken[12] = {
    0, 1, 5, 4, 3, 2, 9, 8, 7, 6, 12, 10,
};

struct TokenOutput {
  const char* token;
  uint8_t modifiers;
  uint8_t keyboardUsage;
  uint16_t consumerUsage;
  uint8_t systemUsage;

  constexpr TokenOutput(const char* tokenValue, uint8_t modifiersValue,
                        uint8_t keyboardUsageValue,
                        uint16_t consumerUsageValue,
                        uint8_t systemUsageValue = 0)
      : token(tokenValue),
        modifiers(modifiersValue),
        keyboardUsage(keyboardUsageValue),
        consumerUsage(consumerUsageValue),
        systemUsage(systemUsageValue) {}
};

constexpr TokenOutput kTokenOutputs[] = {
    {"KC_NONE", 0, 0, 0},
    {"KC_A", 0, 0x04, 0}, {"KC_B", 0, 0x05, 0},
    {"KC_C", 0, 0x06, 0}, {"KC_D", 0, 0x07, 0},
    {"KC_E", 0, 0x08, 0}, {"KC_F", 0, 0x09, 0},
    {"KC_G", 0, 0x0A, 0}, {"KC_H", 0, 0x0B, 0},
    {"KC_I", 0, 0x0C, 0}, {"KC_J", 0, 0x0D, 0},
    {"KC_K", 0, 0x0E, 0}, {"KC_L", 0, 0x0F, 0},
    {"KC_M", 0, 0x10, 0}, {"KC_N", 0, 0x11, 0},
    {"KC_O", 0, 0x12, 0}, {"KC_P", 0, 0x13, 0},
    {"KC_Q", 0, 0x14, 0}, {"KC_R", 0, 0x15, 0},
    {"KC_S", 0, 0x16, 0}, {"KC_T", 0, 0x17, 0},
    {"KC_U", 0, 0x18, 0}, {"KC_V", 0, 0x19, 0},
    {"KC_W", 0, 0x1A, 0}, {"KC_X", 0, 0x1B, 0},
    {"KC_Y", 0, 0x1C, 0}, {"KC_Z", 0, 0x1D, 0},
    {"KC_1", 0, 0x1E, 0}, {"KC_2", 0, 0x1F, 0},
    {"KC_3", 0, 0x20, 0}, {"KC_4", 0, 0x21, 0},
    {"KC_5", 0, 0x22, 0}, {"KC_6", 0, 0x23, 0},
    {"KC_7", 0, 0x24, 0}, {"KC_8", 0, 0x25, 0},
    {"KC_9", 0, 0x26, 0}, {"KC_0", 0, 0x27, 0},
    {"KC_ENTER", 0, 0x28, 0}, {"KC_ESC", 0, 0x29, 0},
    {"KC_BSPC", 0, 0x2A, 0}, {"KC_TAB", 0, 0x2B, 0},
    {"KC_SPACE", 0, 0x2C, 0}, {"KC_MINUS", 0, 0x2D, 0},
    {"KC_EQUAL", 0, 0x2E, 0}, {"KC_LBRC", 0, 0x2F, 0},
    {"KC_RBRC", 0, 0x30, 0}, {"KC_BSLS", 0, 0x31, 0},
    {"KC_SCLN", 0, 0x33, 0}, {"KC_QUOT", 0, 0x34, 0},
    {"KC_GRV", 0, 0x35, 0}, {"KC_COMM", 0, 0x36, 0},
    {"KC_DOT", 0, 0x37, 0}, {"KC_SLSH", 0, 0x38, 0},
    {"KC_CAPS", 0, 0x39, 0},
    {"KC_F1", 0, 0x3A, 0}, {"KC_F2", 0, 0x3B, 0},
    {"KC_F3", 0, 0x3C, 0}, {"KC_F4", 0, 0x3D, 0},
    {"KC_F5", 0, 0x3E, 0}, {"KC_F6", 0, 0x3F, 0},
    {"KC_F7", 0, 0x40, 0}, {"KC_F8", 0, 0x41, 0},
    {"KC_F9", 0, 0x42, 0}, {"KC_F10", 0, 0x43, 0},
    {"KC_F11", 0, 0x44, 0}, {"KC_F12", 0, 0x45, 0},
    {"KC_PSCR", 0, 0x46, 0}, {"KC_INS", 0, 0x49, 0},
    {"KC_HOME", 0, 0x4A, 0}, {"KC_PGUP", 0, 0x4B, 0},
    {"KC_DEL", 0, 0x4C, 0}, {"KC_END", 0, 0x4D, 0},
    {"KC_PGDN", 0, 0x4E, 0}, {"KC_RIGHT", 0, 0x4F, 0},
    {"KC_LEFT", 0, 0x50, 0}, {"KC_DOWN", 0, 0x51, 0},
    {"KC_UP", 0, 0x52, 0}, {"KC_NUM", 0, 0x53, 0},
    {"KC_LCTL", 0x01, 0, 0}, {"KC_LSFT", 0x02, 0, 0},
    {"KC_LALT", 0x04, 0, 0}, {"KC_LGUI", 0x08, 0, 0},
    {"KC_RCTL", 0x10, 0, 0}, {"KC_RSFT", 0x20, 0, 0},
    {"KC_RALT", 0x40, 0, 0}, {"KC_RGUI", 0x80, 0, 0},
    {"KC_COPY", 0x01, 0x06, 0}, {"KC_PASTE", 0x01, 0x19, 0},
    {"KC_CUT", 0x01, 0x1B, 0}, {"KC_UNDO", 0x01, 0x1D, 0},
    {"KC_REDO", 0x01, 0x1C, 0}, {"KC_FIND", 0x01, 0x09, 0},
    {"KC_SAVE", 0x01, 0x16, 0}, {"KC_OPEN", 0x01, 0x12, 0},
    {"KC_PAUS", 0, 0x48, 0}, {"KC_SLCK", 0, 0x47, 0},
    {"KC_PSLS", 0, 0x54, 0}, {"KC_PAST", 0, 0x55, 0},
    {"KC_PMNS", 0, 0x56, 0}, {"KC_PPLS", 0, 0x57, 0},
    {"KC_PENT", 0, 0x58, 0}, {"KC_P1", 0, 0x59, 0},
    {"KC_P2", 0, 0x5A, 0}, {"KC_P3", 0, 0x5B, 0},
    {"KC_P4", 0, 0x5C, 0}, {"KC_P5", 0, 0x5D, 0},
    {"KC_P6", 0, 0x5E, 0}, {"KC_P7", 0, 0x5F, 0},
    {"KC_P8", 0, 0x60, 0}, {"KC_P9", 0, 0x61, 0},
    {"KC_P0", 0, 0x62, 0}, {"KC_PDOT", 0, 0x63, 0},
    {"KC_NUBS", 0, 0x64, 0}, {"KC_APP", 0, 0x65, 0},
    {"KC_POWER", 0, 0x66, 0}, {"KC_KB_POWER", 0, 0x66, 0},
    {"KC_PEQL", 0, 0x67, 0},
    {"KC_F13", 0, 0x68, 0}, {"KC_F14", 0, 0x69, 0},
    {"KC_F15", 0, 0x6A, 0}, {"KC_F16", 0, 0x6B, 0},
    {"KC_F17", 0, 0x6C, 0}, {"KC_F18", 0, 0x6D, 0},
    {"KC_F19", 0, 0x6E, 0}, {"KC_F20", 0, 0x6F, 0},
    {"KC_F21", 0, 0x70, 0}, {"KC_F22", 0, 0x71, 0},
    {"KC_F23", 0, 0x72, 0}, {"KC_F24", 0, 0x73, 0},
    {"KC_EXEC", 0, 0x74, 0}, {"KC_HELP", 0, 0x75, 0},
    {"KC_MENU", 0, 0x76, 0}, {"KC_SELECT", 0, 0x77, 0},
    {"KC_STOP", 0, 0x78, 0}, {"KC_AGAIN", 0, 0x79, 0},
    {"KC_LCAP", 0, 0x82, 0}, {"KC_NLCK", 0, 0x53, 0},
    {"KC_LSCR", 0, 0x84, 0}, {"KC_PCMM", 0, 0x85, 0},
    {"KC_KP_EQUAL_AS400", 0, 0x86, 0},
    {"KC_RO", 0, 0x87, 0}, {"KC_KANA", 0, 0x88, 0},
    {"KC_JYEN", 0, 0x89, 0}, {"KC_HENK", 0, 0x8A, 0},
    {"KC_MHEN", 0, 0x8B, 0}, {"KC_INT6", 0, 0x8C, 0},
    {"KC_INT7", 0, 0x8D, 0}, {"KC_INT8", 0, 0x8E, 0},
    {"KC_INT9", 0, 0x8F, 0}, {"KC_HAEN", 0, 0x90, 0},
    {"KC_HANJ", 0, 0x91, 0}, {"KC_LANG3", 0, 0x92, 0},
    {"KC_LANG4", 0, 0x93, 0}, {"KC_LANG5", 0, 0x94, 0},
    {"KC_LANG6", 0, 0x95, 0}, {"KC_LANG7", 0, 0x96, 0},
    {"KC_LANG8", 0, 0x97, 0}, {"KC_LANG9", 0, 0x98, 0},
    {"KC_ERAS", 0, 0x99, 0}, {"KC_SYSREQ", 0, 0x9A, 0},
    {"KC_CANCEL", 0, 0x9B, 0}, {"KC_CLEAR", 0, 0x9C, 0},
    {"KC_PRIOR", 0, 0x9D, 0}, {"KC_RETURN", 0, 0x9E, 0},
    {"KC_SEPAR", 0, 0x9F, 0}, {"KC_OUT", 0, 0xA0, 0},
    {"KC_OPER", 0, 0xA1, 0}, {"KC_CLEAR_AGAIN", 0, 0xA2, 0},
    {"KC_CRSEL", 0, 0xA3, 0}, {"KC_EXSEL", 0, 0xA4, 0},
    {"KC_ENT", 0, 0x28, 0}, {"KC_EQL", 0, 0x2E, 0},
    {"KC_MINS", 0, 0x2D, 0}, {"KC_SPC", 0, 0x2C, 0},
    {"KC_RGHT", 0, 0x4F, 0}, {"KC_NUHS", 0, 0x32, 0},
    {"KC_MPRV", 0, 0, 0x00B6}, {"KC_MPLY", 0, 0, 0x00CD},
    {"KC_MNXT", 0, 0, 0x00B5}, {"KC_MSTP", 0, 0, 0x00B7},
    {"KC_MSEL", 0, 0, 0x0183}, {"KC_EJCT", 0, 0, 0x00B8},
    {"KC_MFFD", 0, 0, 0x00B3}, {"KC_MRWD", 0, 0, 0x00B4},
    {"KC_BRIU", 0, 0, 0x006F}, {"KC_BRID", 0, 0, 0x0070},
    {"KC_VOLD", 0, 0, 0x00EA}, {"KC_VOLU", 0, 0, 0x00E9},
    {"KC_MUTE", 0, 0, 0x00E2},
    {"KC_PWR", 0, 0, 0, 0x81}, {"KC_SLEP", 0, 0, 0, 0x82},
    {"KC_WAKE", 0, 0, 0, 0x83},
};

const TokenOutput* findTokenOutput(const String& token) {
  for (const TokenOutput& output : kTokenOutputs) {
    if (token == output.token) return &output;
  }
  return nullptr;
}

bool parseLightingEffect(const char* name, uint8_t& effect) {
  if (name == nullptr) return false;
  if (strcmp(name, "off") == 0) effect = 0;
  else if (strcmp(name, "solid") == 0) effect = 1;
  else if (strcmp(name, "snake") == 0) effect = 2;
  else if (strcmp(name, "rainbow") == 0) effect = 3;
  else if (strcmp(name, "breath") == 0) effect = 4;
  else if (strcmp(name, "gradient") == 0) effect = 5;
  else return false;
  return true;
}

bool parseLightingConfig(JsonObjectConst lights, ThreadLight& backlightValue,
                         LightingSide& underglowValue) {
  if (lights.isNull()) return false;
  const JsonObjectConst backlight = lights["backlight"].as<JsonObjectConst>();
  const JsonObjectConst underglow = lights["underglow"].as<JsonObjectConst>();
  if (backlight.isNull() || underglow.isNull()) return false;

  ThreadLight backlightCandidate;
  LightingSide underglowCandidate;
  backlightCandidate.color = backlight["color"] | 0xFFFFFFu;
  backlightCandidate.brightness = backlight["brightness"] | 1.0f;
  backlightCandidate.speed = backlight["speed"] | 0.5f;
  backlightCandidate.magic = backlight["magic"] | 1.0f;
  underglowCandidate.color = underglow["color"] | 0xFFFFFFu;
  underglowCandidate.brightness = underglow["brightness"] | 1.0f;
  underglowCandidate.speed = underglow["speed"] | 0.5f;
  underglowCandidate.magic = underglow["magic"] | 1.0f;
  if (!parseLightingEffect(backlight["effect"].as<const char*>(),
                           backlightCandidate.effect) ||
      !parseLightingEffect(underglow["effect"].as<const char*>(),
                           underglowCandidate.effect) ||
      !std::isfinite(backlightCandidate.brightness) ||
      !std::isfinite(backlightCandidate.speed) ||
      !std::isfinite(backlightCandidate.magic) ||
      backlightCandidate.magic < 0.0f || backlightCandidate.magic > 1.0f ||
      !std::isfinite(underglowCandidate.brightness) ||
      !std::isfinite(underglowCandidate.speed) ||
      !std::isfinite(underglowCandidate.magic) ||
      backlightCandidate.brightness < 0.0f ||
      backlightCandidate.brightness > 1.0f ||
      backlightCandidate.speed < 0.0f || backlightCandidate.speed > 1.0f ||
      underglowCandidate.brightness < 0.0f ||
      underglowCandidate.brightness > 1.0f ||
      underglowCandidate.speed < 0.0f ||
      underglowCandidate.speed > 1.0f || underglowCandidate.magic < 0.0f ||
      underglowCandidate.magic > 1.0f) {
    return false;
  }
  backlightCandidate.color &= 0x00FFFFFFu;
  underglowCandidate.color &= 0x00FFFFFFu;
  backlightValue = backlightCandidate;
  underglowValue = underglowCandidate;
  return true;
}

bool parseTokenId(const String& token, const char* prefix, int32_t& id) {
  if (prefix == nullptr || !token.startsWith(prefix)) return false;
  const String suffix = token.substring(strlen(prefix));
  if (suffix.isEmpty()) return false;
  for (size_t index = 0; index < suffix.length(); ++index) {
    if (!isDigit(suffix[index])) return false;
  }
  id = suffix.toInt();
  return true;
}

bool isValidActionReferenceToken(const String& token) {
  int32_t id = -1;
  return parseTokenId(token, "KA_A", id) ||
         parseTokenId(token, "KA_M", id);
}

bool isValidSmartActionToken(const String& token) {
  if (!token.startsWith("SA_")) return false;
  const String suffix = token.substring(3);
  if (suffix.isEmpty() || (suffix.length() > 1 && suffix[0] == '0')) {
    return false;
  }
  for (size_t index = 0; index < suffix.length(); ++index) {
    if (!isDigit(suffix[index])) return false;
  }
  return true;
}

bool isValidInternalToken(const String& token) {
  // KI_FP belongs to devices that expose the optional fpLayer. Codex Micro is
  // explicitly advertised by Work axis without that layer, so accepting it
  // here would create a saved-but-inert key. Keep it rejected until a real
  // fpLayer is implemented end-to-end.
  if (token == "KI_FP") return false;
  if (token == "KI_BLDW" || token == "KI_BLUP" || token == "KI_CUSB" ||
      token == "KI_CS_SHOW" || token == "KI_CS_SHOW_TMP" ||
      token == "KI_CS_HIDE" || token == "KI_CS_TOGGLE" || token == "KI_X") {
    return true;
  }
  int32_t id = -1;
  if (parseTokenId(token, "KI_LS", id) || parseTokenId(token, "KI_LM", id) ||
      parseTokenId(token, "KI_PS", id) || parseTokenId(token, "KI_CBT", id) ||
      parseTokenId(token, "KI_CBTP", id)) {
    return id >= 1 && id <= 8 &&
           (!token.startsWith("KI_L") || id <= 6) &&
           (!token.startsWith("KI_PS") || id <= 6);
  }
  return false;
}

bool isValidCodexVendorToken(const String& token) {
  constexpr const char* kPrefix = "KV_OAI_";
  if (!token.startsWith(kPrefix)) return false;
  const String suffix = token.substring(strlen(kPrefix));
  if (suffix == "ENC_CC" || suffix == "ENC_CW" || suffix == "ENC_CLK") {
    return true;
  }
  if (!(suffix.startsWith("AG") || suffix.startsWith("ACT"))) {
    return false;
  }
  const bool isAgent = suffix.startsWith("AG");
  const size_t digitStart = isAgent ? 2 : 3;
  if (suffix.length() != digitStart + 2) return false;
  if (!isDigit(suffix[digitStart]) || !isDigit(suffix[digitStart + 1])) {
    return false;
  }
  const int value = (suffix[digitStart] - '0') * 10 +
                    (suffix[digitStart + 1] - '0');
  // The Codex Micro physical OAI map has six Agent inputs. ACT11 is retained
  // as a valid logical vendor event because it exists in Work axis layouts,
  // even though this board has no physical ACT11 contact.
  return isAgent ? value <= 5 : value >= 6 && value <= 12;
}

bool isSupportedConfigToken(const String& token) {
  return findTokenOutput(token) != nullptr ||
         isValidActionReferenceToken(token) ||
         isValidSmartActionToken(token) || isValidInternalToken(token) ||
         isValidCodexVendorToken(token);
}

constexpr uint8_t kMultiIdle = 0;
constexpr uint8_t kMultiFirstPressed = 1;
constexpr uint8_t kMultiWaitingSecond = 2;
constexpr uint8_t kMultiSecondPressed = 3;
constexpr uint8_t kMultiHoldActive = 4;
constexpr uint8_t kMultiTapHoldActive = 5;

// Composite keyboard, consumer-control and vendor protocol descriptor. The
// vendor collection remains Report 6 with the original 63-byte body.
const uint8_t kReportMap[] = {
    0x05, 0x01,              // Usage Page (Generic Desktop)
    0x09, 0x06,              // Usage (Keyboard)
    0xA1, 0x01,              // Collection (Application)
    0x85, 0x01,              // Report ID (1)
    0x05, 0x07,              // Usage Page (Keyboard/Keypad)
    0x19, 0xE0,              // Usage Minimum (Keyboard LeftControl)
    0x29, 0xE7,              // Usage Maximum (Keyboard Right GUI)
    0x15, 0x00,              // Logical Minimum (0)
    0x25, 0x01,              // Logical Maximum (1)
    0x75, 0x01,              // Report Size (1)
    0x95, 0x08,              // Report Count (8)
    0x81, 0x02,              // Input (Data, Variable, Absolute)
    0x75, 0x08,              // Report Size (8)
    0x95, 0x01,              // Report Count (1)
    0x81, 0x01,              // Input (Constant)
    0x19, 0x00,              // Usage Minimum (0)
    // Work axis exposes the full USB HID Keyboard/Keypad page, including
    // international, language and extended keypad usages above F24.
    0x29, 0xA4,              // Usage Maximum (Keyboard/Keypad Exsel)
    0x15, 0x00,              // Logical Minimum (0)
    0x25, 0xA4,              // Logical Maximum (Keyboard/Keypad Exsel)
    0x75, 0x08,              // Report Size (8)
    0x95, 0x06,              // Report Count (6)
    0x81, 0x00,              // Input (Data, Array, Absolute)
    0xC0,                    // End Collection

    0x05, 0x0C,              // Usage Page (Consumer)
    0x09, 0x01,              // Usage (Consumer Control)
    0xA1, 0x01,              // Collection (Application)
    0x85, 0x02,              // Report ID (2)
    0x15, 0x00,              // Logical Minimum (0)
    0x26, 0xFF, 0x03,        // Logical Maximum (1023)
    0x19, 0x00,              // Usage Minimum (0)
    0x2A, 0xFF, 0x03,        // Usage Maximum (1023)
    0x75, 0x10,              // Report Size (16)
    0x95, 0x01,              // Report Count (1)
    0x81, 0x00,              // Input (Data, Array, Absolute)
    0xC0,                    // End Collection

    0x05, 0x01,              // Usage Page (Generic Desktop)
    0x09, 0x80,              // Usage (System Control)
    0xA1, 0x01,              // Collection (Application)
    0x85, 0x03,              // Report ID (3)
    0x15, 0x00,              // Logical Minimum (0)
    0x26, 0xFF, 0x00,        // Logical Maximum (255)
    0x19, 0x00,              // Usage Minimum (Undefined)
    0x29, 0xFF,              // Usage Maximum (255)
    0x75, 0x08,              // Report Size (8)
    0x95, 0x01,              // Report Count (1)
    0x81, 0x00,              // Input (Data, Array, Absolute)
    0xC0,                    // End Collection

    0x06, 0x00, 0xFF,        // Usage Page (Vendor Defined 0xFF00)
    0x09, 0x01,              // Usage (1)
    0xA1, 0x01,              // Collection (Application)
    0x85, 0x06,              // Report ID (6)
    0x15, 0x00,              // Logical Minimum (0)
    0x26, 0xFF, 0x00,        // Logical Maximum (255)
    0x75, 0x08,              // Report Size (8)
    0x95, 0x3F,              // Report Count (63)
    0x09, 0x01,              // Usage (1)
    0x81, 0x02,              // Input (Data, Variable, Absolute)
    0x95, 0x3F,              // Report Count (63)
    0x09, 0x02,              // Usage (2)
    0x91, 0x02,              // Output (Data, Variable, Absolute)
    0xC0                     // End Collection
};

class SecurityCallbacks final : public BLESecurityCallbacks {
 public:
  bool onSecurityRequest() override { return true; }
  uint32_t onPassKeyRequest() override { return 0; }
  void onPassKeyNotify(uint32_t) override {}
  bool onConfirmPIN(uint32_t) override { return true; }
  void onAuthenticationComplete(esp_ble_auth_cmpl_t result) override {
    Serial.printf("BLE pairing %s\n", result.success ? "complete" : "failed");
  }
};

class SemaphoreGuard {
 public:
  explicit SemaphoreGuard(SemaphoreHandle_t semaphore)
      : semaphore_(semaphore) {
    if (semaphore_ != nullptr) {
      xSemaphoreTake(semaphore_, portMAX_DELAY);
    }
  }

  ~SemaphoreGuard() {
    if (semaphore_ != nullptr) {
      xSemaphoreGive(semaphore_);
    }
  }

  SemaphoreGuard(const SemaphoreGuard&) = delete;
  SemaphoreGuard& operator=(const SemaphoreGuard&) = delete;

 private:
  SemaphoreHandle_t semaphore_;
};

bool clearStoredBleBonds() {
  const int bondCount = esp_ble_get_bond_device_num();
  if (bondCount < 0) {
    Serial.println("ERR BLE bond count unavailable");
    return false;
  }
  if (bondCount == 0) {
    Serial.println("BLE bond reset requested; no stored bonds found");
    return true;
  }

  std::vector<esp_ble_bond_dev_t> bonds(
      static_cast<size_t>(bondCount));
  int listedCount = bondCount;
  const esp_err_t listResult =
      esp_ble_get_bond_device_list(&listedCount, bonds.data());
  if (listResult != ESP_OK) {
    Serial.printf("ERR BLE bond list result=%d\n",
                  static_cast<int>(listResult));
    return false;
  }

  bool allRequestsSucceeded = true;
  for (int index = 0; index < listedCount; ++index) {
    const int countBeforeRemoval = esp_ble_get_bond_device_num();
    const esp_err_t removeResult =
        esp_ble_remove_bond_device(bonds[index].bd_addr);
    if (removeResult != ESP_OK) {
      allRequestsSucceeded = false;
      Serial.printf("ERR BLE bond remove index=%d result=%d\n", index,
                    static_cast<int>(removeResult));
      continue;
    }

    const uint32_t removalStartedMs = millis();
    while (millis() - removalStartedMs < 1000) {
      const int remaining = esp_ble_get_bond_device_num();
      if (remaining >= 0 && remaining < countBeforeRemoval) {
        break;
      }
      delay(10);
    }
  }

  const int remaining = esp_ble_get_bond_device_num();
  const bool cleared = allRequestsSucceeded && remaining == 0;
  Serial.printf("BLE bond reset %s removed=%d remaining=%d\n",
                cleared ? "complete" : "incomplete",
                bondCount - max(remaining, 0), remaining);
  return cleared;
}

}  // namespace

CodexMicroBle* CodexMicroBle::usbEventOwner_ = nullptr;

class CodexMicroBle::ServerCallbacks final : public BLEServerCallbacks {
 public:
  explicit ServerCallbacks(CodexMicroBle& owner) : owner_(owner) {}

  void onConnect(BLEServer*) override { owner_.onConnected(true); }

  void onDisconnect(BLEServer*) override {
    owner_.onConnected(false);
  }

  void onMtuChanged(BLEServer*,
                    esp_ble_gatts_cb_param_t* param) override {
    owner_.onMtuChanged(param->mtu.mtu);
  }

 private:
  CodexMicroBle& owner_;
};

class CodexMicroBle::InputCallbacks final
    : public BLECharacteristicCallbacks {
 public:
  explicit InputCallbacks(CodexMicroBle& owner) : owner_(owner) {}

  void onStatus(BLECharacteristic*,
                BLECharacteristicCallbacks::Status status,
                uint32_t code) override {
    owner_.onNotifyStatus(status, code);
  }

 private:
  CodexMicroBle& owner_;
};

class CodexMicroBle::OutputCallbacks final
    : public BLECharacteristicCallbacks {
 public:
  explicit OutputCallbacks(CodexMicroBle& owner) : owner_(owner) {}

  void onWrite(BLECharacteristic* characteristic) override {
    const std::string value = characteristic->getValue();
    owner_.onOutput(reinterpret_cast<const uint8_t*>(value.data()),
                    value.size(), false);
  }

 private:
  CodexMicroBle& owner_;
};

class CodexMicroBle::UsbHidDevice final : public USBHIDDevice {
 public:
  explicit UsbHidDevice(CodexMicroBle& owner) : owner_(owner) {
    registered_ = USBHID::addDevice(this, sizeof(kReportMap));
  }

  uint16_t _onGetDescriptor(uint8_t* buffer) override {
    memcpy(buffer, kReportMap, sizeof(kReportMap));
    return sizeof(kReportMap);
  }

  void _onOutput(uint8_t reportId, const uint8_t* buffer,
                 uint16_t length) override {
    owner_.onUsbOutput(reportId, buffer, length);
  }

  bool registered() const { return registered_; }

 private:
  CodexMicroBle& owner_;
  bool registered_ = false;
};

void CodexMicroBle::begin(bool clearBleBonds) {
  stateMutex_ = xSemaphoreCreateMutex();
  transmitMutex_ = xSemaphoreCreateMutex();
  receiveMutex_ = xSemaphoreCreateMutex();
  mountFilesystem();
  loadDeviceConfiguration("/keymap.json");
  hidOta_.begin();

#if CODEX_BLE_ENABLED
  BLEDevice::init(kDeviceName);
  if (clearBleBonds) {
    clearStoredBleBonds();
  }
  const esp_err_t mtuResult = BLEDevice::setMTU(kLocalMtu);
  BLEDevice::setSecurityCallbacks(new SecurityCallbacks());

  auto* security = new BLESecurity();
  security->setCapability(ESP_IO_CAP_NONE);
  security->setAuthenticationMode(ESP_LE_AUTH_BOND);

  server_ = BLEDevice::createServer();
  server_->setCallbacks(new ServerCallbacks(*this));

  hid_ = new BLEHIDDevice(server_);
  hid_->manufacturer()->setValue(kManufacturer);
  // Low release bits mark the transport as wireless in the desktop bridge.
  // Arduino-ESP32 2.x serializes these fields big-endian, while the BLE PnP
  // characteristic is little-endian. Pre-swap so macOS enumerates 303A:8360.
  hid_->pnp(0x02, swapBytes(kVendorId), swapBytes(kProductId),
            swapBytes(0x0101));
  hid_->hidInfo(0x00, 0x01);
  hid_->reportMap(const_cast<uint8_t*>(kReportMap), sizeof(kReportMap));

  keyboardInput_ = hid_->inputReport(kKeyboardReportId);
  consumerInput_ = hid_->inputReport(kConsumerReportId);
  systemInput_ = hid_->inputReport(kSystemReportId);
  vendorInput_ = hid_->inputReport(kReportId);
  vendorOutput_ = hid_->outputReport(kReportId);
  vendorInput_->setCallbacks(new InputCallbacks(*this));
  vendorOutput_->setCallbacks(new OutputCallbacks(*this));
  hid_->startServices();
  hid_->setBatteryLevel(batteryPercentage_);

  BLEAdvertising* advertising = BLEDevice::getAdvertising();
  advertising->setAppearance(GENERIC_HID);
  advertising->addServiceUUID(hid_->hidService()->getUUID());
  advertising->setScanResponse(true);
  advertising->setMinPreferred(0x06);
  advertising->setMaxPreferred(0x12);
  BLEDevice::startAdvertising();
#else
  (void)clearBleBonds;
#endif

  usbEventOwner_ = this;
  usbHid_ = new USBHID();
  usbDevice_ = new UsbHidDevice(*this);
  USB.VID(kVendorId);
  USB.PID(kProductId);
  USB.firmwareVersion(kUsbFirmwareVersion);
  USB.manufacturerName(kManufacturer);
  USB.productName(kDeviceName);
  USB.onEvent(usbEventCallback);
  usbHid_->begin();
  const bool usbStarted =
      usbDevice_->registered() && USB.begin();

#if CODEX_BLE_ENABLED
  Serial.printf(
      "BLE composite HID ready VID=%04X PID=%04X reports=%u,%u,%u,%u\n",
      kVendorId, kProductId, kKeyboardReportId, kConsumerReportId,
      kSystemReportId, kReportId);
  Serial.printf("BLE local MTU=%u result=%d required=%u\n",
                kLocalMtu, static_cast<int>(mtuResult), kRequiredMtu);
#else
  Serial.println(
      "BLE disabled at compile time; controller and radio not initialized");
#endif
  Serial.printf(
      "USB composite HID stack %s VID=%04X PID=%04X vendor_report=%u\n",
      usbStarted ? "ready" : "failed", kVendorId, kProductId,
      kReportId);
  Serial.println(
      "USB HID OTA ready (inactive OTA slot, CRC32 + SHA-256)");
}

void CodexMicroBle::poll() {
  if (transmitMutex_ == nullptr) {
    return;
  }

  reconcileUsbMountedState();
  applyPendingTransportChange();
  processMultiActions();
  processActionQueue();
  processHidOta();
  // Handle parked JSON-RPC requests here on the main-loop task (full stack)
  // instead of inside the HID output callback (see onOutput). Skipped while
  // an OTA transfer owns the vendor report.
  if (!hidOta_.active()) {
    processDeferredRpc();
  }
  if (rebootAtMs_ != 0 &&
      static_cast<int32_t>(millis() - rebootAtMs_) >= 0) {
    rebootAtMs_ = 0;
    Serial.println("RPC requested ESP32-S3 ROM bootloader");
    Serial.flush();
    // Arduino-ESP32's USB persistence hook sets FORCE_DOWNLOAD_BOOT and
    // switches the S3 native USB controller to CDC/JTAG download mode. This
    // is the same mode used by esptool and lets the local serial flasher take
    // over without requiring a slow HID OTA transfer.
    usb_persist_restart(RESTART_BOOTLOADER);
    return;
  }
  if (otaRestartAtMs_ != 0 &&
      static_cast<int32_t>(millis() - otaRestartAtMs_) >= 0) {
    Serial.println("HID OTA restarting into verified firmware");
    Serial.flush();
    ESP.restart();
  }
  if (hidOta_.active()) {
    return;
  }

  String json;
  bool jsonUsb = false;
  String receivedJson;
  uint16_t mtu = 23;
  bool printMtu = false;
  BLECharacteristicCallbacks::Status notifyStatus =
      BLECharacteristicCallbacks::Status::ERROR_NO_CLIENT;
  uint32_t notifyCode = 0;
  bool printNotifyStatus = false;
  xSemaphoreTake(transmitMutex_, portMAX_DELAY);
  if (!pendingReceiveLog_.isEmpty()) {
    receivedJson = pendingReceiveLog_;
    pendingReceiveLog_.clear();
  }
  if (transmitCount_ > 0) {
    json = transmitQueue_[transmitTail_];
    jsonUsb = transmitQueueUsb_[transmitTail_];
    transmitQueue_[transmitTail_].clear();
    transmitTail_ = (transmitTail_ + 1) % kTransmitQueueDepth;
    --transmitCount_;
  }
  mtu = peerMtu_;
  printMtu = pendingMtuLog_;
  pendingMtuLog_ = false;
  if (pendingNotifyStatusValid_) {
    notifyStatus = pendingNotifyStatus_;
    notifyCode = pendingNotifyCode_;
    pendingNotifyStatusValid_ = false;
    printNotifyStatus = true;
  }
  xSemaphoreGive(transmitMutex_);

  if (printMtu) {
    Serial.printf("BLE peer MTU=%u (report requires >=%u)\n",
                  mtu, kRequiredMtu);
  }
  if (printNotifyStatus) {
    const char* name = "UNKNOWN";
    switch (notifyStatus) {
      case BLECharacteristicCallbacks::Status::SUCCESS_NOTIFY:
        name = "SUCCESS_NOTIFY";
        break;
      case BLECharacteristicCallbacks::Status::ERROR_NOTIFY_DISABLED:
        name = "ERROR_NOTIFY_DISABLED";
        break;
      case BLECharacteristicCallbacks::Status::ERROR_GATT:
        name = "ERROR_GATT";
        break;
      case BLECharacteristicCallbacks::Status::ERROR_NO_CLIENT:
        name = "ERROR_NO_CLIENT";
        break;
      default:
        break;
    }
    Serial.printf("BLE input notify status=%s code=%lu peer_mtu=%u\n",
                  name, static_cast<unsigned long>(notifyCode), mtu);
  }
  if (!receivedJson.isEmpty()) {
#if CODEX_VERBOSE_DIAGNOSTICS
    Serial.print("HID RX JSON ");
    Serial.println(receivedJson);
#endif
  }
  if (!json.isEmpty()) {
    if (!transportAvailable(jsonUsb)) {
      Serial.printf("HID TX dropped after transport switch (%s)\n",
                    jsonUsb ? "USB" : "BLE");
      return;
    }
#if CODEX_VERBOSE_DIAGNOSTICS
    Serial.print("HID TX JSON ");
    Serial.println(json);
#endif
    transmitJson(json, jsonUsb);
  }
}

void CodexMicroBle::setBattery(uint8_t percentage, bool charging) {
  batteryPercentage_ = constrain(percentage, 0, 100);
  charging_ = charging;
  bool updateBleBattery = false;
  if (stateMutex_ != nullptr) {
    SemaphoreGuard guard(stateMutex_);
    updateBleBattery = bleConnected_;
  }
  if (hid_ != nullptr && updateBleBattery) {
    hid_->setBatteryLevel(batteryPercentage_);
  }
}

void CodexMicroBle::sendCheatSheetEvent(const char* method) {
  if (method == nullptr) return;
  StaticJsonDocument<192> message;
  message["method"] = method;
  JsonObject params = message.createNestedObject("params");
  // The official app indexes both profile and layer from zero in this
  // notification, unlike device.status' one-based layer_index field.
  params["layer"] = activeLayerIndex_;
  params["profile"] = activeProfileIndex_;
  String json;
  serializeJson(message, json);
  sendJson(json);
}

void CodexMicroBle::sendRadialCloseEvent() {
  StaticJsonDocument<192> message;
  message["method"] = "kb.radial";
  JsonObject params = message.createNestedObject("params");
  params["a"] = 0.0f;
  params["d"] = 0.0f;
  params["l"] = activeLayerIndex_;
  params["p"] = activeProfileIndex_;
  params["o"] = 0;
  String json;
  serializeJson(message, json);
  sendJson(json);
  radialMenuVisible_ = false;
}

bool CodexMicroBle::sendCodexVendorToken(const String& token,
                                         uint8_t action) {
  // Saved general macros/joystick sectors may contain legacy Codex tokens.
  // They must not bypass the physical mode switch.
  if (!nativeCodexMode_ || !isValidCodexVendorToken(token) || action > 2) return false;
  const String raw = token.substring(strlen("KV_OAI_"));
  StaticJsonDocument<192> message;
  message["method"] = "v.oai.hid";
  JsonObject params = message.createNestedObject("params");
  params["k"] = raw;
  params["act"] = action;
  if (raw.startsWith("AG") && raw.length() == 4 && isDigit(raw[2]) &&
      isDigit(raw[3])) {
    params["ag"] = (raw[2] - '0') * 10 + (raw[3] - '0');
  }
  String json;
  serializeJson(message, json);
  sendJson(json);
  return true;
}

bool CodexMicroBle::switchToLayerIndex(uint16_t layerIndex) {
  if (!fsReady || !SPIFFS.exists("/keymap.json")) return false;
  File file = SPIFFS.open("/keymap.json", FILE_READ);
  if (!file) return false;
  const size_t fileSize = file.size();
  if (fileSize == 0 || fileSize > (SIZE_MAX - 4096) / 2) {
    file.close();
    return false;
  }
  DynamicJsonDocument document(fileSize * 2 + 4096);
  const DeserializationError error = deserializeJson(document, file, DeserializationOption::NestingLimit(kJsonNestingLimit));
  file.close();
  if (error) return false;
  const JsonArrayConst profiles = document["profiles"].as<JsonArrayConst>();
  JsonObjectConst selectedProfile;
  for (JsonObjectConst profile : profiles) {
    if ((profile["id"] | -1) == activeProfileId_) {
      selectedProfile = profile;
      break;
    }
  }
  if (selectedProfile.isNull() && activeProfileIndex_ < profiles.size()) {
    selectedProfile = profiles[activeProfileIndex_];
  }
  if (selectedProfile.isNull()) return false;
  const JsonArrayConst layers = selectedProfile["layers"].as<JsonArrayConst>();
  if (layerIndex >= layers.size()) return false;
  const int32_t profileId = selectedProfile["id"] | -1;
  const int32_t layerId = layers[layerIndex]["id"] | -1;
  if (profileId < 0 || layerId < 0) return false;
  return loadDeviceConfiguration("/keymap.json", profileId, layerId);
}

bool CodexMicroBle::switchToProfileIndex(uint16_t profileIndex) {
  if (!fsReady || !SPIFFS.exists("/keymap.json")) return false;
  File file = SPIFFS.open("/keymap.json", FILE_READ);
  if (!file) return false;
  const size_t fileSize = file.size();
  if (fileSize == 0 || fileSize > (SIZE_MAX - 4096) / 2) {
    file.close();
    return false;
  }
  DynamicJsonDocument document(fileSize * 2 + 4096);
  const DeserializationError error = deserializeJson(document, file, DeserializationOption::NestingLimit(kJsonNestingLimit));
  file.close();
  if (error) return false;
  const JsonArrayConst profiles = document["profiles"].as<JsonArrayConst>();
  if (profileIndex >= profiles.size()) return false;
  const JsonObjectConst profile = profiles[profileIndex];
  const JsonArrayConst layers = profile["layers"].as<JsonArrayConst>();
  if (layers.isNull() || layers.size() == 0) return false;
  const int32_t profileId = profile["id"] | -1;
  const int32_t layerId = profile["activeLayerId"] | layers[0]["id"].as<int32_t>();
  if (profileId < 0 || layerId < 0) return false;
  return loadDeviceConfiguration("/keymap.json", profileId, layerId);
}

bool CodexMicroBle::handleInternalToken(const String& token, uint8_t action,
                                        int physicalIndex) {
  if (!isValidInternalToken(token)) return false;
  const bool pressed = action == 1 || action == 2;
  if (token == "KI_X") {
    if (pressed) sendRadialCloseEvent();
    return true;
  }
  if (token == "KI_BLDW" || token == "KI_BLUP") {
    if (pressed) {
      const int delta = token == "KI_BLUP" ? 10 : -10;
      codexMicroAdjustGlobalBrightness(delta);
      Serial.printf("BRIGHTNESS adjusted by %d%% (%s)\n", delta,
                    token.c_str());
    }
    return true;
  }
  if (token == "KI_CS_SHOW" || token == "KI_CS_HIDE" ||
      token == "KI_CS_TOGGLE" || token == "KI_CS_SHOW_TMP") {
    if (token == "KI_CS_SHOW_TMP") {
      if (action == 1) sendCheatSheetEvent("kb.cs.show");
      if (action == 0) sendCheatSheetEvent("kb.cs.hide");
      if (action == 2) {
        // A click has no separate release edge. Pair the transient overlay
        // explicitly instead of leaving the cheat sheet stuck on screen.
        sendCheatSheetEvent("kb.cs.show");
        sendCheatSheetEvent("kb.cs.hide");
      }
    } else if (pressed) {
      sendCheatSheetEvent(token == "KI_CS_SHOW"
                              ? "kb.cs.show"
                              : token == "KI_CS_HIDE" ? "kb.cs.hide"
                                                       : "kb.cs.toggle");
    }
    return true;
  }
  if (token == "KI_CUSB") {
    if (!pressed) return true;
    // transportAvailable(true) also considers the current preference. That
    // makes a BLE-preferred device report USB as unavailable even after the
    // cable is inserted. Check the physical mount state directly first.
    bool usbMounted = false;
    if (stateMutex_ != nullptr) {
      xSemaphoreTake(stateMutex_, portMAX_DELAY);
      usbMounted = usbMounted_;
      xSemaphoreGive(stateMutex_);
    } else {
      usbMounted = usbMounted_;
    }
    if (!usbMounted) {
      Serial.println("KI_CUSB requested but USB is not mounted");
      return false;
    }
    preferredTransport_ = 0;
    pendingTransportChange_ = true;
    return true;
  }
  int32_t id = -1;
  if (parseTokenId(token, "KI_CBT", id) ||
      parseTokenId(token, "KI_CBTP", id)) {
    if (!pressed) return true;
    const bool pairing = token.startsWith("KI_CBTP");
    return configureBleChannel(static_cast<uint8_t>(id), pairing);
  }
  if (parseTokenId(token, "KI_LS", id)) {
    if (!pressed) return true;
    const bool switched = id >= 1 && id <= 6 && switchToLayerIndex(id - 1);
    if (switched) {
      manualLayerSelected_ = true;
      const bool codexLayer = std::any_of(configuredTokens_.begin(), configuredTokens_.end(),
        [](const String& value) { return isValidCodexVendorToken(value); });
      codexMicroShowLayerIndicator(codexLayer ? 0x0066FF : 0xFF9900);
    }
    if (!switched) Serial.printf("KI_LS unavailable layer=%ld\n", static_cast<long>(id));
    return switched;
  }
  if (parseTokenId(token, "KI_PS", id)) {
    if (!pressed) return true;
    const bool switched = id >= 1 && id <= 6 && switchToProfileIndex(id - 1);
    if (!switched) Serial.printf("KI_PS unavailable profile=%ld\n", static_cast<long>(id));
    return switched;
  }
  if (parseTokenId(token, "KI_LM", id)) {
    const bool validTarget = id >= 1 && id <= 6;
    if (!validTarget) return false;
    if (physicalIndex >= 0 &&
        static_cast<size_t>(physicalIndex) < momentaryLayerActive_.size()) {
      const size_t index = static_cast<size_t>(physicalIndex);
      if (action == 1 && !momentaryLayerActive_[index]) {
        const int32_t previousProfileId = activeProfileId_;
        const int32_t previousLayerId = activeLayerId_;
        preserveMomentaryState_ = true;
        const bool switched =
            switchToLayerIndex(static_cast<uint16_t>(id - 1));
        preserveMomentaryState_ = false;
        if (!switched) return false;
        // loadDeviceConfiguration intentionally clears stale momentary state;
        // restore this key's saved origin after the target layer is loaded.
        momentaryProfileIds_[index] = previousProfileId;
        momentaryLayerIds_[index] = previousLayerId;
        momentaryLayerActive_[index] = true;
      } else if (action == 0 && momentaryLayerActive_[index]) {
        preserveMomentaryState_ = true;
        const bool restored = loadDeviceConfiguration(
            "/keymap.json", momentaryProfileIds_[index],
            momentaryLayerIds_[index]);
        preserveMomentaryState_ = false;
        if (restored) momentaryLayerActive_[index] = false;
        return restored;
      }
      return true;
    }
    if (action == 1 && !momentaryLayerFallbackActive_) {
      const int32_t previousProfileId = activeProfileId_;
      const int32_t previousLayerId = activeLayerId_;
      preserveMomentaryState_ = true;
      const bool switched =
          switchToLayerIndex(static_cast<uint16_t>(id - 1));
      preserveMomentaryState_ = false;
      if (!switched) return false;
      momentaryProfileId_ = previousProfileId;
      momentaryLayerId_ = previousLayerId;
      momentaryLayerFallbackActive_ = true;
    } else if (action == 0 && momentaryLayerFallbackActive_) {
      preserveMomentaryState_ = true;
      const bool restored = loadDeviceConfiguration(
          "/keymap.json", momentaryProfileId_, momentaryLayerId_);
      preserveMomentaryState_ = false;
      if (restored) momentaryLayerFallbackActive_ = false;
      return restored;
    }
    return true;
  }
  return false;
}

bool CodexMicroBle::configureBleChannel(uint8_t channel, bool pairing) {
#if CODEX_BLE_ENABLED
  if (channel == 0 || channel > 8 || server_ == nullptr || hid_ == nullptr) {
    return false;
  }
  const int bondCount = esp_ble_get_bond_device_num();
  if (bondCount < 0) return false;
  std::vector<esp_ble_bond_dev_t> bonds(static_cast<size_t>(bondCount));
  int listedCount = bondCount;
  if (listedCount > 0 &&
      esp_ble_get_bond_device_list(&listedCount, bonds.data()) != ESP_OK) {
    return false;
  }

  // BLE channel numbers are stable user-facing slots over the controller's
  // bond list. Remove every prior whitelist entry before selecting a slot so
  // a stale filter can never strand the keyboard after a re-pair.
  for (int index = 0; index < listedCount; ++index) {
    BLEDevice::whiteListRemove(BLEAddress(bonds[index].bd_addr));
  }

  if (pairing) {
    const int selected = static_cast<int>(channel - 1);
    if (selected < listedCount &&
        esp_ble_remove_bond_device(bonds[selected].bd_addr) != ESP_OK) {
      Serial.printf("BLE pair slot %u could not remove old bond\n",
                    static_cast<unsigned>(channel));
      return false;
    }
    BLEDevice::getAdvertising()->setScanFilter(false, false);
    bleChannel_ = channel;
    preferredTransport_ = 1;
    if (bleConnected_) server_->disconnect(server_->getConnId());
    BLEDevice::startAdvertising();
    Serial.printf("BLE pairing slot %u ready\n", static_cast<unsigned>(channel));
    return true;
  }

  const int selected = static_cast<int>(channel - 1);
  if (selected >= listedCount) {
    Serial.printf("BLE channel %u has no stored bond\n",
                  static_cast<unsigned>(channel));
    return false;
  }
  BLEDevice::whiteListAdd(BLEAddress(bonds[selected].bd_addr));
  BLEDevice::getAdvertising()->setScanFilter(false, true);
  bleChannel_ = channel;
  preferredTransport_ = 1;
  if (bleConnected_) server_->disconnect(server_->getConnId());
  BLEDevice::startAdvertising();
  Serial.printf("BLE channel %u selected\n", static_cast<unsigned>(channel));
  return true;
#else
  (void)channel;
  (void)pairing;
  return false;
#endif
}

void CodexMicroBle::releaseLayerInputs() {
  // A stick held across the switch belongs to the previous mode until neutral.
  modeSuppressedJoystick_ = joystickHeld_;
  // Finish every held input on its current owner before changing mode.
  for (size_t i = 0; i < 12; ++i) {
    const String& token = pressedRouteTokens_[i];
    if (!token.isEmpty()) {
      if (isValidCodexVendorToken(token)) sendCodexVendorToken(token, 0);
      modeSuppressedReleases_[i] = true;
    }
    pressedRouteTokens_[i].clear();
  }
  configuredKeysPressed_.fill(false);
  macroHeldTokens_.clear();
  resolvedHeldTokens_.fill(String());
  actionQueue_.clear();
  multiActionStates_.fill(MultiActionState());
  activeJoystickSector_ = -1;
  emitCombinedReports();
  if (radialMenuVisible_) { sendRadialCloseEvent(); radialMenuVisible_ = false; }
  if (nativeCodexMode_) {
    StaticJsonDocument<128> neutral;
    neutral["method"] = "v.oai.rad";
    neutral["params"]["a"] = 0;
    neutral["params"]["d"] = 0;
    String json; serializeJson(neutral, json); sendJson(json);
  }
}

void CodexMicroBle::toggleCodexMode() {
  releaseLayerInputs();
  // Use the saved profile order, including the real vendor layer. Never
  // manufacture a Codex layer when a profile does not contain one.
  if (cycleLayerCount_ == 0) return;
  const uint16_t next = (activeLayerIndex_ + 1) % cycleLayerCount_;
  if (!switchToLayerIndex(next)) return;
  const bool vendor = std::any_of(configuredTokens_.begin(), configuredTokens_.end(),
      [](const String& token) { return token.startsWith("KV_OAI_"); });
  switchLightingMode(vendor);
  manualLayerSelected_ = true;
  codexMicroShowLayerIndicator(nativeCodexMode_ ? 0x0066FF : 0xFF9900);
}

void CodexMicroBle::switchLightingMode(bool codexMode) {
  SemaphoreGuard guard(stateMutex_);
  if (codexMode == nativeCodexMode_) return;
  auto copyLighting = [](CodexMicroState& target, const CodexMicroState& source) {
    target.threads = source.threads;
    target.ambient = source.ambient;
    target.keys = source.keys;
    target.ambientSyncThread = source.ambientSyncThread;
  };
  copyLighting(nativeCodexMode_ ? codexLighting_ : generalLighting_, state_);
  copyLighting(state_, codexMode ? codexLighting_ : generalLighting_);
  nativeCodexMode_ = codexMode;
  ++state_.threadsRevision;
  ++state_.ambientRevision;
  state_.dirty = true;
}

void CodexMicroBle::sendKey(const char* key, uint8_t action, int8_t agent) {
  String configuredToken;
  int routeIndex = -1;
  for (size_t index = 0; index < 12; ++index) {
    if (strcmp(key, kPhysicalInputNames[index]) == 0 && configLoaded_) {
      configuredToken = configuredTokens_[index];
      routeIndex = static_cast<int>(index);
      break;
    }
  }
  if (configuredToken.isEmpty() && configLoaded_) {
    int encoderIndex = -1;
    if (strcmp(key, "ENC_CC") == 0) encoderIndex = 0;
    if (strcmp(key, "ENC_CW") == 0) encoderIndex = 1;
    if (strcmp(key, "ENC") == 0) encoderIndex = 2;
    if (encoderIndex >= 0) configuredToken = configuredEncoderTokens_[encoderIndex];
    if (encoderIndex == 2) routeIndex = 12;
  }

  if (routeIndex >= 0 && routeIndex < 12 && modeSuppressedReleases_[routeIndex]) {
    if (action == 0) modeSuppressedReleases_[routeIndex] = false;
    return;
  }
  if (nativeCodexMode_) configuredToken = String("KV_OAI_") +
      (strcmp(key, "ENC") == 0 ? "ENC_CLK" : key);

  // The release belongs to the token pressed, even if the knob changed layers
  // while another key was held. Otherwise Codex can miss its release edge.
  if (configLoaded_ && routeIndex >= 0) {
    String& owner = pressedRouteTokens_[static_cast<size_t>(routeIndex)];
    if (action == 1) owner = configuredToken;
    else if (action == 0 && !owner.isEmpty()) {
      configuredToken = owner;
      owner.clear();
    }
  }

  // Resolve the active layer before doing any work, then give the physical
  // edge to exactly one owner. The official Codex layer is made of KV_OAI_*
  // tokens and goes to the vendor channel. KC_*, actions, multi-actions,
  // smart-actions and internal layer/profile controls stay on the device/Input
  // path. Classifying first prevents a future token handler from accidentally
  // executing locally and then being forwarded to Codex as a second action.
  if (!nativeCodexMode_) {
    // No vendor fallback when a general configuration is missing, or when an
    // old preset still contains KV_OAI_* mappings.
    if (!configLoaded_ || isValidCodexVendorToken(configuredToken)) return;
    const bool executed = sendConfiguredKey(key, action);
    Serial.printf("HID key=%s action=%u route=device result=%s\n", key, action,
                  executed ? "ok" : "failed");
    return;
  }

  String vendorKey = strcmp(key, "ENC") == 0 ? "ENC_CLK" : key;
  int8_t vendorAgent = agent;
  if (configLoaded_) {
    vendorKey = configuredToken.substring(strlen("KV_OAI_"));
    vendorAgent = -1;
    if (vendorKey.startsWith("AG") && vendorKey.length() == 4 &&
        isDigit(vendorKey[2]) && isDigit(vendorKey[3])) {
      vendorAgent = static_cast<int8_t>((vendorKey[2] - '0') * 10 +
                                        (vendorKey[3] - '0'));
    }
  }

  // Match the official @worklouder/device-kit-oai HIDKey wire contract. The
  // current Codex app derives task slots from k=AG00..AG05; ag remains optional
  // metadata and must describe the resolved token rather than the PCB switch.
  StaticJsonDocument<192> message;
  message["method"] = "v.oai.hid";
  JsonObject params = message.createNestedObject("params");
  params["k"] = vendorKey;
  params["act"] = action;
  if (vendorAgent >= 0) {
    params["ag"] = vendorAgent;
  }

  String json;
  serializeJson(message, json);
  sendJson(json);
  Serial.printf("HID key=%s action=%u route=codex\n", key, action);
}

bool CodexMicroBle::sendConfiguredKey(const char* key, uint8_t action) {
  if (!configLoaded_ || key == nullptr) return false;

  int physicalIndex = -1;
  for (size_t index = 0; index < 12; ++index) {
    if (strcmp(key, kPhysicalInputNames[index]) == 0) {
      physicalIndex = static_cast<int>(index);
      break;
    }
  }

  if (physicalIndex < 0) {
    int encoderIndex = -1;
    if (strcmp(key, "ENC_CC") == 0) encoderIndex = 0;
    if (strcmp(key, "ENC_CW") == 0) encoderIndex = 1;
    if (strcmp(key, "ENC") == 0) encoderIndex = 2;
    if (encoderIndex < 0) return false;
    const String& token = configuredEncoderTokens_[encoderIndex];
    if (encoderIndex == 2 && (token.startsWith("KI_LS") ||
                             !encoderLayerPressToken_.isEmpty())) {
      if (action == 1) {
        encoderLayerPressAt_ = millis();
        encoderLayerPressToken_ = token;
      } else if (action == 0) {
        const String target = encoderLayerPressToken_;
        encoderLayerPressToken_.clear();
        if (!target.isEmpty() && millis() - encoderLayerPressAt_ <= 500)
          return handleInternalToken(target, 1);
      }
      return true;
    }
    if (token.startsWith("KA_A")) {
      return action == 0 || enqueueTokenAction(token, 2, 0);
    }
    if (token.startsWith("KA_M")) {
      return action == 0 || dispatchResolvedToken(token, 2);
    }
    if (token.startsWith("SA_")) {
      if (action == 1 || action == 2) sendSmartAction(token);
      return false;
    }
    if (isValidInternalToken(token)) return handleInternalToken(token, action);
    // sendKey() emits the single host-facing vendor event after this helper;
    // do not call sendCodexVendorToken() here or encoder events would double.
    if (isValidCodexVendorToken(token)) return action <= 2;
    const TokenOutput* output = findTokenOutput(token);
    if (output == nullptr) return false;
    if (action == 2) {
      uint8_t report[8] = {output->modifiers, 0, output->keyboardUsage};
      if (output->systemUsage != 0) {
        sendSystemReport(output->systemUsage);
        sendSystemReport(0);
      } else if (output->consumerUsage != 0) {
        sendConsumerReport(output->consumerUsage);
        sendConsumerReport(0);
      } else if (output->modifiers != 0 || output->keyboardUsage != 0) {
        sendKeyboardReport(report, sizeof(report));
        memset(report, 0, sizeof(report));
        sendKeyboardReport(report, sizeof(report));
      }
    } else if (output->systemUsage != 0) {
      sendSystemReport(action == 1 ? output->systemUsage : 0);
    } else if (output->consumerUsage != 0) {
      sendConsumerReport(action == 1 ? output->consumerUsage : 0);
    } else if (output->modifiers != 0 || output->keyboardUsage != 0) {
      uint8_t report[8] = {};
      if (action == 1) {
        report[0] = output->modifiers;
        report[2] = output->keyboardUsage;
      }
      sendKeyboardReport(report, sizeof(report));
    }
    return true;
  }

  if (action != 0 && action != 1) return false;
  const String& token = configuredTokens_[static_cast<size_t>(physicalIndex)];
  if (token.startsWith("KA_A")) {
    return action == 0 || enqueueTokenAction(token, 2, 0);
  }
  if (token.startsWith("KA_M")) {
    handleMultiAction(static_cast<size_t>(physicalIndex), token, action);
    return canExecuteDeviceToken(token);
  }
  if (token.startsWith("SA_")) {
    if (action == 1) sendSmartAction(token);
    return false;
  }
  if (isValidInternalToken(token)) {
    return handleInternalToken(token, action, physicalIndex);
  }
  if (isValidCodexVendorToken(token)) {
    // sendKey() owns the host-facing vendor event for physical inputs.
    return action <= 2;
  }
  if (findTokenOutput(token) == nullptr) return false;
  configuredKeysPressed_[static_cast<size_t>(physicalIndex)] = action == 1;
  emitCombinedReports();
  return true;
}

bool CodexMicroBle::canExecuteDeviceToken(const String& token) const {
  if (token.startsWith("SA_")) return false;
  if (isValidInternalToken(token) || isValidCodexVendorToken(token)) return true;
  if (findTokenOutput(token) != nullptr) return true;
  int32_t macroId = -1;
  if (parseTokenId(token, "KA_A", macroId)) {
    for (size_t index = 0; index < configuredMacroCount_; ++index) {
      if (configuredMacros_[index].id == macroId) return true;
    }
    return false;
  }
  int32_t multiActionId = -1;
  if (!parseTokenId(token, "KA_M", multiActionId)) return false;
  for (size_t index = 0; index < configuredMultiActionCount_; ++index) {
    if (configuredMultiActions_[index].id == multiActionId) return true;
  }
  return false;
}

bool CodexMicroBle::enqueueTokenAction(const String& token, uint8_t action,
                                       uint32_t delayMs, size_t depth) {
  int32_t macroId = -1;
  if (parseTokenId(token, "KA_A", macroId)) {
    if (action == 0) return true;
    return enqueueMacro(macroId, delayMs, depth + 1);
  }
  if (findTokenOutput(token) == nullptr && !token.startsWith("SA_") &&
      !token.startsWith("KA_M") && !isValidInternalToken(token) &&
      !isValidCodexVendorToken(token)) {
    return false;
  }
  if (action > 2) return false;
  QueuedAction queued;
  queued.token = token;
  queued.action = action;
  queued.delayMs = delayMs;
  actionQueue_.push_back(std::move(queued));
  return true;
}

bool CodexMicroBle::enqueueMacro(int32_t macroId, uint32_t trailingDelayMs,
                                 size_t depth) {
  if (depth > configuredMacroCount_ + configuredMultiActionCount_ + 1) {
    return false;
  }
  const MacroDefinition* macro = nullptr;
  for (size_t index = 0; index < configuredMacroCount_; ++index) {
    if (configuredMacros_[index].id == macroId) {
      macro = &configuredMacros_[index];
      break;
    }
  }
  if (macro == nullptr) return false;

  const size_t originalCount = actionQueue_.size();
  for (size_t index = 0; index < macro->stepCount; ++index) {
    const MacroStep& step = macro->steps[index];
    const uint32_t delayMs =
        index + 1 == macro->stepCount ? step.delayMs + trailingDelayMs
                                      : step.delayMs;
    if (!enqueueTokenAction(step.token, step.action, delayMs, depth)) {
      while (actionQueue_.size() > originalCount) actionQueue_.pop_back();
      Serial.printf("MACRO enqueue rejected id=%ld\n",
                    static_cast<long>(macroId));
      return false;
    }
  }
  return true;
}

void CodexMicroBle::processActionQueue() {
  if (actionQueue_.empty()) return;
  const uint32_t now = millis();
  if (nextActionAtMs_ != 0 && static_cast<int32_t>(now - nextActionAtMs_) < 0) {
    return;
  }
  QueuedAction action = std::move(actionQueue_.front());
  actionQueue_.pop_front();
  applyQueuedAction(action.token, action.action);
  nextActionAtMs_ = action.delayMs == 0 ? 0 : now + action.delayMs;
}

void CodexMicroBle::applyQueuedAction(const String& token, uint8_t action) {
  if (token.startsWith("KA_M")) {
    dispatchResolvedToken(token, action, 1);
    return;
  }
  if (token.startsWith("SA_")) {
    if (action != 0) sendSmartAction(token);
    return;
  }
  if (handleInternalToken(token, action)) return;
  if (isValidCodexVendorToken(token)) {
    if (action != 0) sendCodexVendorToken(token, action);
    return;
  }

  const TokenOutput* output = findTokenOutput(token);
  if (output == nullptr) return;
  auto press = [&]() {
    for (const String& held : macroHeldTokens_) {
      if (held == token) return;
    }
    macroHeldTokens_.push_back(token);
    emitCombinedReports();
  };
  auto release = [&]() {
    for (auto iterator = macroHeldTokens_.begin();
         iterator != macroHeldTokens_.end(); ++iterator) {
      if (*iterator == token) {
        macroHeldTokens_.erase(iterator);
        emitCombinedReports();
        return;
      }
    }
  };
  if (action == 1) press();
  if (action == 0) release();
  if (action == 2) {
    press();
    release();
  }
}

bool CodexMicroBle::dispatchResolvedToken(const String& token, uint8_t action,
                                          size_t depth, int sourceIndex) {
  if (depth > configuredMacroCount_ + configuredMultiActionCount_ + 1) {
    return false;
  }
  int32_t macroId = -1;
  if (parseTokenId(token, "KA_A", macroId)) {
    return action == 0 || enqueueMacro(macroId, 0, depth + 1);
  }
  int32_t multiActionId = -1;
  if (parseTokenId(token, "KA_M", multiActionId)) {
    const MultiActionDefinition* definition = nullptr;
    for (size_t index = 0; index < configuredMultiActionCount_; ++index) {
      if (configuredMultiActions_[index].id == multiActionId) {
        definition = &configuredMultiActions_[index];
        break;
      }
    }
    if (definition == nullptr) return false;
    const String& target = action == 2 ? definition->tapToken
                                      : definition->holdToken;
    return dispatchResolvedToken(target, action, depth + 1, sourceIndex);
  }
  if (handleInternalToken(token, action)) return true;
  if (isValidCodexVendorToken(token)) {
    return sendCodexVendorToken(token, action);
  }
  if (findTokenOutput(token) == nullptr && !token.startsWith("SA_")) {
    return false;
  }
  if (sourceIndex >= 0 &&
      sourceIndex < static_cast<int>(resolvedHeldTokens_.size()) &&
      findTokenOutput(token) != nullptr) {
    String& held = resolvedHeldTokens_[static_cast<size_t>(sourceIndex)];
    if (action == 1) {
      held = token;
      emitCombinedReports();
    } else if (action == 0) {
      if (held == token) {
        held.clear();
        emitCombinedReports();
      }
    } else if (action == 2) {
      const String previous = held;
      held = token;
      emitCombinedReports();
      held = previous;
      emitCombinedReports();
    } else {
      return false;
    }
    return true;
  }
  applyQueuedAction(token, action);
  return true;
}

void CodexMicroBle::handleMultiAction(size_t physicalIndex,
                                      const String& token, uint8_t action) {
  if (physicalIndex >= multiActionStates_.size() ||
      (action != 0 && action != 1)) {
    return;
  }
  int32_t multiActionId = -1;
  if (!parseTokenId(token, "KA_M", multiActionId)) return;
  const MultiActionDefinition* definition = nullptr;
  for (size_t index = 0; index < configuredMultiActionCount_; ++index) {
    if (configuredMultiActions_[index].id == multiActionId) {
      definition = &configuredMultiActions_[index];
      break;
    }
  }
  if (definition == nullptr) return;

  const uint32_t now = millis();
  MultiActionState& state = multiActionStates_[physicalIndex];
  if (action == 1) {
    if (state.phase == kMultiWaitingSecond && state.id == multiActionId &&
        now - state.releasedAtMs < definition->tappingTermMs) {
      state.phase = kMultiSecondPressed;
      state.pressedAtMs = now;
      return;
    }
    if (state.phase == kMultiWaitingSecond) {
      dispatchResolvedToken(definition->tapToken, 2, 0, static_cast<int>(physicalIndex));
    }
    state = {};
    state.id = multiActionId;
    state.phase = kMultiFirstPressed;
    state.pressedAtMs = now;
    return;
  }

  if (state.id != multiActionId) return;
  if (state.phase == kMultiFirstPressed) {
    if (now - state.pressedAtMs >= definition->tappingTermMs) {
      // If the poll loop did not observe the hold threshold before release,
      // emit the hold target as a complete click so the selected branch is
      // still the same one the Work axis UI defines.
      dispatchResolvedToken(definition->holdToken, 2, 0, static_cast<int>(physicalIndex));
      state = {};
    } else {
      state.phase = kMultiWaitingSecond;
      state.releasedAtMs = now;
    }
    return;
  }
  if (state.phase == kMultiSecondPressed) {
    if (now - state.pressedAtMs >= definition->tappingTermMs) {
      // As above, preserve tap-hold selection even if release races the poll.
      dispatchResolvedToken(definition->tapHoldToken, 2, 0, static_cast<int>(physicalIndex));
    } else {
      dispatchResolvedToken(definition->doubleTapToken, 2, 0, static_cast<int>(physicalIndex));
    }
    state = {};
    return;
  }
  if (state.phase == kMultiHoldActive) {
    dispatchResolvedToken(definition->holdToken, 0, 0, static_cast<int>(physicalIndex));
    state = {};
    return;
  }
  if (state.phase == kMultiTapHoldActive) {
    dispatchResolvedToken(definition->tapHoldToken, 0, 0, static_cast<int>(physicalIndex));
    state = {};
  }
}

void CodexMicroBle::processMultiActions() {
  const uint32_t now = millis();
  for (size_t physicalIndex = 0; physicalIndex < multiActionStates_.size(); ++physicalIndex) {
    MultiActionState& state = multiActionStates_[physicalIndex];
    if (state.phase == kMultiIdle) continue;
    const MultiActionDefinition* definition = nullptr;
    for (size_t index = 0; index < configuredMultiActionCount_; ++index) {
      if (configuredMultiActions_[index].id == state.id) {
        definition = &configuredMultiActions_[index];
        break;
      }
    }
    if (definition == nullptr) {
      state = {};
      continue;
    }
    if (state.phase == kMultiWaitingSecond &&
        now - state.releasedAtMs >= definition->tappingTermMs) {
      dispatchResolvedToken(definition->tapToken, 2, 0, static_cast<int>(physicalIndex));
      state = {};
    } else if (state.phase == kMultiFirstPressed &&
               now - state.pressedAtMs >= definition->tappingTermMs) {
      dispatchResolvedToken(definition->holdToken, 1, 0, static_cast<int>(physicalIndex));
      state.phase = kMultiHoldActive;
    } else if (state.phase == kMultiSecondPressed &&
               now - state.pressedAtMs >= definition->tappingTermMs) {
      dispatchResolvedToken(definition->tapHoldToken, 1, 0, static_cast<int>(physicalIndex));
      state.phase = kMultiTapHoldActive;
    }
  }
}

void CodexMicroBle::emitCombinedReports() {

  uint8_t keyboardReport[8] = {};
  uint16_t consumerUsage = 0;
  uint8_t systemUsage = 0;
  size_t keyboardUsageCount = 0;
  auto addOutput = [&](const TokenOutput* output) {
    if (output == nullptr) return;
    keyboardReport[0] |= output->modifiers;
    if (output->consumerUsage != 0 && consumerUsage == 0) {
      consumerUsage = output->consumerUsage;
    }
    if (output->systemUsage != 0 && systemUsage == 0) {
      systemUsage = output->systemUsage;
    }
    if (output->keyboardUsage != 0) {
      bool alreadyPresent = false;
      for (size_t slot = 0; slot < keyboardUsageCount && slot < 6; ++slot) {
        alreadyPresent = alreadyPresent ||
                         keyboardReport[2 + slot] == output->keyboardUsage;
      }
      if (!alreadyPresent) {
        if (keyboardUsageCount < 6) {
          keyboardReport[2 + keyboardUsageCount] = output->keyboardUsage;
        }
        ++keyboardUsageCount;
      }
    }
  };
  for (size_t index = 0; index < configuredKeysPressed_.size(); ++index) {
    if (!configuredKeysPressed_[index]) continue;
    addOutput(findTokenOutput(configuredTokens_[index]));
  }
  for (const String& token : macroHeldTokens_) {
    if (!token.isEmpty()) addOutput(findTokenOutput(token));
  }
  for (const String& token : resolvedHeldTokens_) {
    if (!token.isEmpty()) addOutput(findTokenOutput(token));
  }
  if (keyboardUsageCount > 6) memset(keyboardReport + 2, 0x01, 6);
  sendKeyboardReport(keyboardReport, sizeof(keyboardReport));
  sendConsumerReport(consumerUsage);
  sendSystemReport(systemUsage);
}

void CodexMicroBle::sendKeyboardReport(const uint8_t* report,
                                       size_t length) {
  if (report == nullptr || length != 8) return;
  if (transportAvailable(true)) {
    if (usbHid_ != nullptr) {
      usbHid_->SendReport(kKeyboardReportId, report, length,
                          kUsbReportTimeoutMs);
    }
    return;
  }
  if (!transportAvailable(false) || keyboardInput_ == nullptr) return;
  keyboardInput_->setValue(const_cast<uint8_t*>(report), length);
  keyboardInput_->notify();
  delay(4);
}

void CodexMicroBle::sendConsumerReport(uint16_t usage) {
  const uint8_t report[2] = {
      static_cast<uint8_t>(usage & 0xFF),
      static_cast<uint8_t>((usage >> 8) & 0xFF),
  };
  if (transportAvailable(true)) {
    if (usbHid_ != nullptr) {
      usbHid_->SendReport(kConsumerReportId, report, sizeof(report),
                          kUsbReportTimeoutMs);
    }
    return;
  }
  if (!transportAvailable(false) || consumerInput_ == nullptr) return;
  consumerInput_->setValue(const_cast<uint8_t*>(report), sizeof(report));
  consumerInput_->notify();
  delay(4);
}

void CodexMicroBle::sendSystemReport(uint8_t usage) {
  if (transportAvailable(true)) {
    if (usbHid_ != nullptr) {
      usbHid_->SendReport(kSystemReportId, &usage, sizeof(usage),
                          kUsbReportTimeoutMs);
    }
    return;
  }
  if (!transportAvailable(false) || systemInput_ == nullptr) return;
  systemInput_->setValue(&usage, sizeof(usage));
  systemInput_->notify();
  delay(4);
}

void CodexMicroBle::sendJoystick(float angle, float distance) {
  joystickHeld_ = distance > 0.0f;
  if (modeSuppressedJoystick_) {
    if (!joystickHeld_) modeSuppressedJoystick_ = false;
    return;
  }
  if (!nativeCodexMode_) {
  applyConfiguredJoystick(angle, distance);
  if (joystickType_ == "RADIAL") {
    const bool close = distance <= 0.0f ||
                       (activeJoystickSector_ >= 0 &&
                        static_cast<size_t>(activeJoystickSector_) <
                            joystickSectors_.size() &&
                        joystickSectors_[static_cast<size_t>(activeJoystickSector_)]
                                .token == "KI_X");
    StaticJsonDocument<224> radial;
    radial["method"] = "kb.radial";
    JsonObject radialParams = radial.createNestedObject("params");
    radialParams["a"] = angle;
    radialParams["d"] = distance;
    radialParams["l"] = activeLayerIndex_;
    radialParams["p"] = activeProfileIndex_;
    radialParams["o"] = close ? 0 : (radialMenuVisible_ ? 2 : 1);
    if (activeJoystickSector_ >= 0) {
      radialParams["s"] = activeJoystickSector_;
    }
    String radialJson;
    serializeJson(radial, radialJson);
    sendJson(radialJson);
    radialMenuVisible_ = !close;
  }
  return;
  }
  StaticJsonDocument<160> message;
  message["method"] = "v.oai.rad";
  JsonObject params = message.createNestedObject("params");
  params["a"] = angle;
  params["d"] = distance;

  String json;
  serializeJson(message, json);
  sendJson(json);
}

bool CodexMicroBle::joystickSectorContains(float angle, float start,
                                           float end) const {
  if (start <= end) return angle >= start && angle < end;
  return angle >= start || angle < end;
}

void CodexMicroBle::applyConfiguredJoystick(float angle, float distance) {
  int32_t nextIndex = -1;
  if (configLoaded_ && distance > 0.0f) {
    for (size_t index = 0; index < joystickSectorCount_; ++index) {
      if (joystickSectorContains(angle, joystickSectors_[index].start,
                                 joystickSectors_[index].end)) {
        nextIndex = static_cast<int32_t>(index);
        break;
      }
    }
  }
  if (nextIndex == activeJoystickSector_) return;

  if (activeJoystickSector_ >= 0) {
    const JoystickSector& previous =
        joystickSectors_[static_cast<size_t>(activeJoystickSector_)];
    if (previous.token != "KI_X" && previous.token != "KC_NONE") {
      dispatchResolvedToken(previous.token, 0, 0, 12);
    }
  }
  activeJoystickSector_ = -1;
  if (nextIndex >= 0) {
    const JoystickSector& next =
        joystickSectors_[static_cast<size_t>(nextIndex)];
    if (next.token != "KI_X" && next.token != "KC_NONE") {
      dispatchResolvedToken(next.token, 1, 0, 12);
    }
    // A radial sector is meaningful even when it is intentionally blank or
    // closes the overlay. Keep its index for kb.radial's `s` field and let the
    // host decide how to render that sector.
    activeJoystickSector_ = nextIndex;
  }
}

bool CodexMicroBle::connected() {
  if (stateMutex_ == nullptr) {
    return false;
  }
  xSemaphoreTake(stateMutex_, portMAX_DELAY);
  const bool result = state_.connected;
  xSemaphoreGive(stateMutex_);
  return result;
}

CodexMicroState CodexMicroBle::snapshot() {
  CodexMicroState copy;
  if (stateMutex_ == nullptr) {
    return copy;
  }
  xSemaphoreTake(stateMutex_, portMAX_DELAY);
  copy = state_;
  state_.dirty = false;
  xSemaphoreGive(stateMutex_);
  return copy;
}

void CodexMicroBle::onConnected(bool connected) {
  bool clearQueue = false;
  {
    SemaphoreGuard guard(stateMutex_);
    bleConnected_ = connected;
    state_.connected = usbMounted_ || bleConnected_;
    state_.dirty = true;
    pendingTransportChange_ =
        (connected && usbMounted_) || (!connected && !usbMounted_);
    clearQueue = !connected && !usbMounted_;
  }
  if (!connected && receiveMutex_ != nullptr) {
    SemaphoreGuard guard(receiveMutex_);
    if (!rpcBuffer_.isEmpty() && !rpcBufferUsb_) {
      rpcBuffer_.clear();
    }
  }
  if (clearQueue) {
    clearTransmitQueue();
  }
  Serial.printf("BLE host %s\n",
                connected ? "connected" : "disconnected");
}

void CodexMicroBle::onUsbOutput(uint8_t reportId, const uint8_t* data,
                                size_t length) {
  if (reportId != kReportId) {
    return;
  }
  const size_t offset =
      (length > 0 && data[0] == kReportId) ? 1 : 0;
  if (length > offset && data[offset] == HidOtaUpdate::kRequestType) {
    hidOta_.enqueue(data + offset, length - offset);
    return;
  }
  onOutput(data, length, true);
}

void CodexMicroBle::usbEventCallback(void*, esp_event_base_t eventBase,
                                     int32_t eventId, void*) {
  if (usbEventOwner_ != nullptr &&
      eventBase == ARDUINO_USB_EVENTS) {
    usbEventOwner_->onUsbEvent(eventId);
  }
}

void CodexMicroBle::onUsbEvent(int32_t eventId) {
  const char* eventName = nullptr;
  {
    SemaphoreGuard guard(stateMutex_);
    switch (eventId) {
      case ARDUINO_USB_STARTED_EVENT:
        usbMounted_ = true;
        usbSuspended_ = false;
        pendingTransportChange_ = true;
        eventName = "mounted";
        break;
      case ARDUINO_USB_STOPPED_EVENT:
        usbMounted_ = false;
        usbSuspended_ = false;
        pendingTransportChange_ = true;
        eventName = "unmounted";
        break;
      case ARDUINO_USB_SUSPEND_EVENT:
        // A suspended bus is still an enumerated computer connection. Keeping
        // USB selected prevents duplicate input from suddenly moving to BLE.
        usbSuspended_ = true;
        eventName = "suspended";
        break;
      case ARDUINO_USB_RESUME_EVENT:
        usbSuspended_ = false;
        eventName = "resumed";
        break;
      default:
        return;
    }
    state_.connected = usbMounted_ || bleConnected_;
    state_.dirty = true;
  }
  Serial.printf("USB host %s\n", eventName);
}

void CodexMicroBle::reconcileUsbMountedState() {
  // ESPUSB::operator bool() is true only after TinyUSB has been mounted
  // (configured) by a USB host. VBUS, battery charging, and a charge-only
  // cable do not satisfy this condition.
  const bool mounted = static_cast<bool>(USB);
  {
    SemaphoreGuard guard(stateMutex_);
    if (usbMounted_ == mounted) {
      return;
    }

    // Recover if an asynchronous Arduino USB mount/unmount event was missed
    // or reached the application out of order.
    usbMounted_ = mounted;
    if (!mounted) {
      usbSuspended_ = false;
    }
    pendingTransportChange_ = true;
    state_.connected = usbMounted_ || bleConnected_;
    state_.dirty = true;
  }
  Serial.printf("USB host state reconciled to %s from TinyUSB configured state\n",
                mounted ? "mounted" : "unmounted");
}

void CodexMicroBle::applyPendingTransportChange() {
  bool pending = false;
  bool useUsb = false;
  bool bleConnected = false;
  {
    SemaphoreGuard guard(stateMutex_);
    pending = pendingTransportChange_;
    if (pending) {
      pendingTransportChange_ = false;
      // A requested BLE channel may coexist with a mounted USB cable. If BLE
      // is not connected, fall back to USB so a transport preference cannot
      // disable all input/output while the selected peer is unavailable.
      useUsb = usbMounted_ &&
               (preferredTransport_ != 1 || !bleConnected_);
      bleConnected = bleConnected_;
    }
  }
  if (!pending) {
    return;
  }

  if (!useUsb) {
    hidOta_.onUsbDisconnected();
  }

  if (receiveMutex_ != nullptr) {
    SemaphoreGuard guard(receiveMutex_);
    if (!rpcBuffer_.isEmpty() && rpcBufferUsb_ != useUsb) {
      rpcBuffer_.clear();
    }
  }

#if CODEX_BLE_ENABLED
  if (useUsb) {
    BLEDevice::getAdvertising()->stop();
    if (bleConnected && preferredTransport_ != 1 && server_ != nullptr) {
      server_->disconnect(server_->getConnId());
    }
    Serial.println(
        "TRANSPORT USB selected (host enumeration succeeded)");
  } else {
    if (!bleConnected) {
      BLEDevice::startAdvertising();
    }
    Serial.println(
        "TRANSPORT BLE selected (USB host not enumerated)");
  }
#else
  if (useUsb) {
    Serial.println(
        "TRANSPORT USB selected (BLE disabled at compile time)");
  } else {
    Serial.println(
        "TRANSPORT unavailable (USB not enumerated; BLE disabled)");
  }
#endif
}

bool CodexMicroBle::transportAvailable(bool usb) {
  if (stateMutex_ == nullptr) {
    return false;
  }
  SemaphoreGuard guard(stateMutex_);
  if (preferredTransport_ == 0 && usbMounted_) return usb;
  if (preferredTransport_ == 1 && bleConnected_) return !usb;
  return usb ? usbMounted_ : (!usbMounted_ && bleConnected_);
}

void CodexMicroBle::onMtuChanged(uint16_t mtu) {
  if (transmitMutex_ == nullptr) {
    return;
  }
  xSemaphoreTake(transmitMutex_, portMAX_DELAY);
  peerMtu_ = mtu;
  pendingMtuLog_ = true;
  xSemaphoreGive(transmitMutex_);
}

void CodexMicroBle::onNotifyStatus(
    BLECharacteristicCallbacks::Status status, uint32_t code) {
  if (transmitMutex_ == nullptr) {
    return;
  }
  xSemaphoreTake(transmitMutex_, portMAX_DELAY);
  pendingNotifyStatus_ = status;
  pendingNotifyCode_ = code;
  pendingNotifyStatusValid_ = true;
  xSemaphoreGive(transmitMutex_);
}

void CodexMicroBle::onOutput(const uint8_t* data, size_t length,
                             bool usb) {
  if (data == nullptr || length < 2) {
    return;
  }
  if (!transportAvailable(usb)) {
    return;
  }
  if (hidOta_.active()) {
    return;
  }
  SemaphoreGuard receiveGuard(receiveMutex_);
  if (!rpcBuffer_.isEmpty() && rpcBufferUsb_ != usb) {
    rpcBuffer_.clear();
  }
  if (rpcBuffer_.isEmpty()) {
    rpcBufferUsb_ = usb;
  }

  // HOGP normally strips the report ID. Accept an included ID as well so the
  // transport remains compatible with hosts that forward the raw report.
  size_t offset = (length >= 3 && data[0] == kReportId) ? 1 : 0;
  if (length < offset + 2 || data[offset] != 2) {
    return;
  }

  const size_t payloadLength =
      min<size_t>(data[offset + 1], kPayloadSize);
  if (length < offset + 2 + payloadLength) {
    return;
  }
  const char* payload =
      reinterpret_cast<const char*>(data + offset + 2);
  constexpr char kTopLevelPrefix[] = "{\"method\"";
  const bool startsTopLevel =
      payloadLength >= sizeof(kTopLevelPrefix) - 1 &&
      memcmp(payload, kTopLevelPrefix,
             sizeof(kTopLevelPrefix) - 1) == 0;
  if (startsTopLevel && !rpcBuffer_.isEmpty()) {
    // A new top-level object means a previous fragmented write was dropped.
    // Resynchronize immediately instead of poisoning the next request.
    rpcBuffer_.clear();
  }
  if (rpcBuffer_.isEmpty()) {
    size_t jsonStart = 0;
    while (jsonStart < payloadLength && payload[jsonStart] != '{') {
      ++jsonStart;
    }
    if (jsonStart == payloadLength) {
      return;
    }
    rpcBuffer_.concat(payload + jsonStart, payloadLength - jsonStart);
  } else {
    rpcBuffer_.concat(payload, payloadLength);
  }

  DynamicJsonDocument request(kRpcDocumentCapacity);
  const DeserializationError error =
      deserializeJson(request, rpcBuffer_, DeserializationOption::NestingLimit(kJsonNestingLimit));
  if (error == DeserializationError::IncompleteInput) {
    return;
  }
  if (error) {
    Serial.printf("RPC parse error: %s\n", error.c_str());
    rpcBuffer_.clear();
    return;
  }

  if (transmitMutex_ != nullptr) {
    xSemaphoreTake(transmitMutex_, portMAX_DELAY);
    pendingReceiveLog_ = rpcBuffer_;
    xSemaphoreGive(transmitMutex_);
  }
  // Park the completed request for poll() to process. Running handleRpc()
  // here means heavy handlers (e.g. fs.write -> loadDeviceConfiguration:
  // SPIFFS + JSON parse + config swap) execute inside the HID callback,
  // whose small stack overflows and watchdog-resets the device. poll() runs
  // on the main-loop task with a full stack (OTA already uses this pattern).
  if (deferredRpcRequests_.size() < kDeferredRpcLimit) {
    deferredRpcRequests_.push_back(rpcBuffer_);
  } else if (!request["id"].isNull()) {
    // sendError only queues a response; filesystem work stays in poll().
    sendError(request["id"], -32005, "device_busy_retry");
  }
  rpcBuffer_.clear();
}

bool CodexMicroBle::validateSmartActionsFile(const char* path) {
  if (!fsReady || path == nullptr || !SPIFFS.exists(path)) return false;

  File file = SPIFFS.open(path, FILE_READ);
  if (!file) return false;
  const size_t fileSize = file.size();
  if (fileSize == 0 || fileSize > (SIZE_MAX - 4096) / 2) {
    file.close();
    return false;
  }

  DynamicJsonDocument document(fileSize * 2 + 4096);
  const DeserializationError error = deserializeJson(document, file, DeserializationOption::NestingLimit(kJsonNestingLimit));
  file.close();
  if (error || (document["version"] | 0) != 1) return false;

  const JsonObjectConst actions =
      document["smartActions"].as<JsonObjectConst>();
  if (actions.isNull()) return false;
  for (JsonPairConst entry : actions) {
    const String token = entry.key().c_str();
    if (!isValidSmartActionToken(token) ||
        !entry.value().is<JsonObjectConst>()) {
      return false;
    }
    const JsonObjectConst action = entry.value().as<JsonObjectConst>();
    const char* type = action["type"].as<const char*>();
    const JsonObjectConst payload = action["payload"].as<JsonObjectConst>();
    if (type == nullptr || payload.isNull()) return false;
    if (strcmp(type, "TEXT_STEP") == 0) {
      if (!payload["text"].is<const char*>()) return false;
    } else if (strcmp(type, "CMD_STEP") == 0) {
      if (!payload["cmd"].is<const char*>()) return false;
    } else if (strcmp(type, "URL_STEP") == 0) {
      if (!payload["url"].is<const char*>()) return false;
    } else if (strcmp(type, "APP_STEP") == 0) {
      if (!payload["name"].is<const char*>() ||
          !payload["path"].is<const char*>()) {
        return false;
      }
    } else {
      return false;
    }
  }
  return true;
}

bool CodexMicroBle::smartActionReferenceExists(const String& token) const {
  if (!isValidSmartActionToken(token) || !fsReady ||
      !SPIFFS.exists("/smart_actions.json")) {
    return false;
  }

  File file = SPIFFS.open("/smart_actions.json", FILE_READ);
  if (!file) return false;
  const size_t fileSize = file.size();
  if (fileSize == 0 || fileSize > (SIZE_MAX - 2048) / 2) {
    file.close();
    return false;
  }

  StaticJsonDocument<256> filter;
  filter["smartActions"][token] = true;
  DynamicJsonDocument document(fileSize * 2 + 1024);
  const DeserializationError error = deserializeJson(
      document, file, DeserializationOption::Filter(filter),
      DeserializationOption::NestingLimit(kJsonNestingLimit));
  file.close();
  if (error) return false;

  const JsonObjectConst action =
      document["smartActions"][token].as<JsonObjectConst>();
  return !action.isNull() && action["type"].is<const char*>() &&
         action["payload"].is<JsonObjectConst>();
}

bool CodexMicroBle::sendSmartAction(const String& token) {
  if (!isValidSmartActionToken(token) || !fsReady ||
      !SPIFFS.exists("/smart_actions.json")) {
    return false;
  }

  File file = SPIFFS.open("/smart_actions.json", FILE_READ);
  if (!file) return false;
  const size_t fileSize = file.size();
  if (fileSize == 0) {
    file.close();
    return false;
  }
  StaticJsonDocument<256> filter;
  filter["smartActions"][token] = true;
  // The Work axis UI does not impose a fixed Smart Action payload limit.
  // Size the filtered parser from the committed file instead of silently
  // rejecting long text, command, URL, or application payloads at runtime.
  DynamicJsonDocument document(fileSize * 2 + 1024);
  const DeserializationError error = deserializeJson(
      document, file, DeserializationOption::Filter(filter),
      DeserializationOption::NestingLimit(kJsonNestingLimit));
  file.close();
  if (error) return false;

  const JsonObjectConst action =
      document["smartActions"][token].as<JsonObjectConst>();
  const JsonObjectConst payload = action["payload"].as<JsonObjectConst>();
  const char* type = action["type"].as<const char*>();
  const char* method = nullptr;
  const char* parameter = nullptr;
  const char* value = nullptr;
  if (type != nullptr && !payload.isNull()) {
    if (strcmp(type, "TEXT_STEP") == 0) {
      method = "kb.sa.inserttext";
      parameter = "text";
      value = payload["text"].as<const char*>();
    } else if (strcmp(type, "CMD_STEP") == 0) {
      method = "kb.sa.exec";
      parameter = "cmd";
      value = payload["cmd"].as<const char*>();
    } else if (strcmp(type, "URL_STEP") == 0) {
      method = "kb.sa.openurl";
      parameter = "url";
      value = payload["url"].as<const char*>();
    } else if (strcmp(type, "APP_STEP") == 0) {
      method = "kb.sa.openapp";
      parameter = "path";
      value = payload["path"].as<const char*>();
    }
  }
  if (method == nullptr || parameter == nullptr || value == nullptr ||
      value[0] == '\0') {
    return false;
  }

  DynamicJsonDocument message(strlen(value) + 512);
  message["method"] = method;
  message["params"][parameter] = value;
  String json;
  serializeJson(message, json);
  sendJson(json);
  return true;
}

bool CodexMicroBle::loadDeviceConfiguration(const char* path,
                                            int32_t profileOverride,
                                            int32_t layerOverride) {
  if (!fsReady || path == nullptr || !SPIFFS.exists(path)) return false;

  File file = SPIFFS.open(path, FILE_READ);
  if (!file) return false;
  const size_t fileSize = file.size();
  if (fileSize == 0 || fileSize > (SIZE_MAX - 8192) / 2) {
    file.close();
    return false;
  }
  // Parse the complete Work axis document. Filtering to profiles[0]/layers[0]
  // silently discarded every additional profile and layer before the active
  // IDs were resolved, so a valid preset could load the wrong runtime map.
  // The document is sized from the actual committed file and can use the
  // board's external RAM; no product-level profile/layer count is imposed.
  DynamicJsonDocument document(fileSize * 2 + 8192);
  const DeserializationError error = deserializeJson(document, file, DeserializationOption::NestingLimit(kJsonNestingLimit));
  file.close();
  if (error) {
    Serial.printf("CONFIG load failed path=%s error=%s\n", path,
                  error.c_str());
    return false;
  }

  const JsonArrayConst profiles = document["profiles"].as<JsonArrayConst>();
  if (profiles.isNull() || profiles.size() == 0) return false;
  int32_t requestedProfileId =
      profileOverride >= 0
          ? profileOverride
          : document["activeProfileId"] | profiles[0]["id"].as<int32_t>();
  if (manualLayerSelected_ && profileOverride < 0) {
    for (JsonObjectConst profile : profiles)
      if ((profile["id"] | -1) == activeProfileId_) requestedProfileId = activeProfileId_;
  }
  JsonObjectConst selectedProfile;
  uint16_t selectedProfileIndex = 0;
  uint16_t profileIndex = 0;
  for (JsonObjectConst profile : profiles) {
    if ((profile["id"] | -1) == requestedProfileId) {
      selectedProfile = profile;
      selectedProfileIndex = profileIndex;
      break;
    }
    ++profileIndex;
  }
  if (selectedProfile.isNull() && profileOverride < 0) {
    selectedProfile = profiles[0]; selectedProfileIndex = 0;
    requestedProfileId = selectedProfile["id"].as<int32_t>();
  }
  if (selectedProfile.isNull()) return false;

  const JsonArrayConst layers = selectedProfile["layers"].as<JsonArrayConst>();
  if (layers.isNull() || layers.size() == 0) return false;

  int32_t requestedLayerId =
      layerOverride >= 0
          ? layerOverride
          : selectedProfile["activeLayerId"] | layers[0]["id"].as<int32_t>();
  if (manualLayerSelected_ && layerOverride < 0 && requestedProfileId == activeProfileId_) {
    for (JsonObjectConst layer : layers)
      if ((layer["id"] | -1) == activeLayerId_) requestedLayerId = activeLayerId_;
  }
  JsonObjectConst selectedLayer;
  uint16_t selectedLayerIndex = 0;
  uint16_t layerIndex = 0;
  for (JsonObjectConst layer : layers) {
    if ((layer["id"] | -1) == requestedLayerId) {
      selectedLayer = layer;
      selectedLayerIndex = layerIndex;
      break;
    }
    ++layerIndex;
  }
  if (selectedLayer.isNull() && layerOverride < 0) {
    selectedLayer = layers[0]; selectedLayerIndex = 0;
    requestedLayerId = selectedLayer["id"].as<int32_t>();
  }
  if (selectedLayer.isNull()) return false;
  // The desktop app stores the key matrix under "base"; the original adapted
  // firmware wrote it under "keymap". Accept base first (official protocol)
  // and fall back to keymap so an existing device file still loads through
  // the transition until the app rewrites it in the official shape.
  const JsonObjectConst layoutJson =
      selectedLayer["layout"].as<JsonObjectConst>();
  const JsonArrayConst keymap = layoutJson["base"].is<JsonArrayConst>()
                                    ? layoutJson["base"].as<JsonArrayConst>()
                                    : layoutJson["keymap"].as<JsonArrayConst>();
  std::array<String, 13> logicalTokens;
  size_t logicalTokenCount = 0;
  for (JsonArrayConst row : keymap) {
    for (JsonVariantConst value : row) {
      if (logicalTokenCount >= logicalTokens.size()) return false;
      const String token = value.as<const char*>();
      if (!isSupportedConfigToken(token)) {
        return false;
      }
      logicalTokens[logicalTokenCount++] = token;
    }
  }
  if (logicalTokenCount != logicalTokens.size()) return false;

  std::array<String, 12> candidateTokens;
  for (size_t index = 0; index < candidateTokens.size(); ++index) {
    candidateTokens[index] = logicalTokens[kPhysicalToLogicalToken[index]];
  }

  const JsonArrayConst encoders =
      selectedLayer["layout"]["encoders"].as<JsonArrayConst>();
  const JsonArrayConst encoder =
      encoders.size() > 0 ? encoders[0].as<JsonArrayConst>() : JsonArrayConst();
  if (encoders.isNull() || encoder.isNull() || encoder.size() < 3) {
    return false;
  }
  std::array<String, 3> candidateEncoderTokens;
  for (size_t index = 0; index < candidateEncoderTokens.size(); ++index) {
    const String token = encoder[index].as<const char*>();
    if (!isSupportedConfigToken(token)) {
      return false;
    }
    candidateEncoderTokens[index] = token;
  }

  std::vector<JoystickSector> candidateJoystickSectors;
  const JsonObjectConst joystick =
      selectedLayer["layout"]["joystick"].as<JsonObjectConst>();
  const String joystickType = joystick["type"] | "VENDOR";
  if (joystickType == "RADIAL") {
    const JsonArrayConst sectors = joystick["sectors"].as<JsonArrayConst>();
    if (sectors.isNull()) return false;
    candidateJoystickSectors.reserve(sectors.size());
    for (JsonObjectConst sourceSector : sectors) {
      if (!sourceSector.containsKey("k") || !sourceSector.containsKey("a1") ||
          !sourceSector.containsKey("a2")) {
        return false;
      }
      const String token = sourceSector["k"].as<const char*>();
      const float start = sourceSector["a1"].as<float>();
      const float end = sourceSector["a2"].as<float>();
      const bool tokenSupported =
          token == "KI_X" || isSupportedConfigToken(token);
      if (!tokenSupported || !std::isfinite(start) || !std::isfinite(end) ||
          start < 0.0f || start > 1.0f || end < 0.0f || end > 1.0f) {
        return false;
      }
      JoystickSector target;
      target.token = token;
      target.start = start;
      target.end = end;
      candidateJoystickSectors.push_back(std::move(target));
    }
  } else if (joystickType != "VENDOR") {
    return false;
  }

  bool candidateLightingConfigured = false;
  ThreadLight candidateBacklight;
  LightingSide candidateUnderglow;
  const JsonObjectConst lights = selectedLayer["lights"].as<JsonObjectConst>();
  if (!lights.isNull()) {
    candidateLightingConfigured =
        parseLightingConfig(lights, candidateBacklight, candidateUnderglow);
    if (!candidateLightingConfigured) return false;
  }

  std::vector<MacroDefinition> candidateMacros;
  const JsonArrayConst macros = document["macros"].as<JsonArrayConst>();
  if (!macros.isNull()) {
    candidateMacros.reserve(macros.size());
    for (JsonObjectConst sourceMacro : macros) {
      const JsonArrayConst steps = sourceMacro["actions"].as<JsonArrayConst>();
      if (steps.isNull()) return false;
      if ((sourceMacro["id"] | -1) < 0) return false;
      MacroDefinition candidateMacro;
      candidateMacro.id = sourceMacro["id"].as<int32_t>();
      candidateMacro.steps.reserve(steps.size());
      for (JsonObjectConst sourceStep : steps) {
        const String token = sourceStep["kc"].as<const char*>();
        const int action = sourceStep["act"] | 2;
        const uint32_t delayMs = sourceStep["delay"] | 0;
        if (!isSupportedConfigToken(token) ||
            action < 0 || action > 2) {
          return false;
        }
        MacroStep candidateStep;
        candidateStep.token = token;
        candidateStep.action = static_cast<uint8_t>(action);
        candidateStep.delayMs = delayMs;
        candidateMacro.steps.push_back(std::move(candidateStep));
      }
      candidateMacro.stepCount = candidateMacro.steps.size();
      candidateMacros.push_back(std::move(candidateMacro));
    }
  }

  std::vector<MultiActionDefinition> candidateMultiActions;
  const JsonArrayConst multiActions =
      document["multiActions"].as<JsonArrayConst>();
  if (!multiActions.isNull()) {
    candidateMultiActions.reserve(multiActions.size());
    for (JsonObjectConst sourceMulti : multiActions) {
      const int32_t id = sourceMulti["id"] | -1;
      const uint32_t tappingTermMs = sourceMulti["tt"] | 250;
      if (id < 0) return false;
      const std::array<String, 4> targets = {
          sourceMulti["kcOnTap"].as<const char*>(),
          sourceMulti["kcOnHold"].as<const char*>(),
          sourceMulti["kcOnDoubleTap"].as<const char*>(),
          sourceMulti["kcOnTapHold"].as<const char*>()};
      for (const String& token : targets) {
        if (!isSupportedConfigToken(token)) {
          return false;
        }
      }
      MultiActionDefinition candidateMultiAction;
      candidateMultiAction.id = id;
      candidateMultiAction.tapToken = targets[0];
      candidateMultiAction.holdToken = targets[1];
      candidateMultiAction.doubleTapToken = targets[2];
      candidateMultiAction.tapHoldToken = targets[3];
      candidateMultiAction.tappingTermMs = tappingTermMs;
      candidateMultiActions.push_back(std::move(candidateMultiAction));
    }
  }

  // The UI deliberately stores all profiles, macros and multi-actions in one
  // document. Reject dangling KA_A/KA_M references before swapping the
  // runtime state so a preset cannot appear saved while a gesture silently
  // becomes a no-op on the keyboard.
  const auto actionReferenceExists = [&](const String& token) {
    int32_t id = -1;
    if (parseTokenId(token, "KA_A", id)) {
      for (const MacroDefinition& macro : candidateMacros) {
        if (macro.id == id) return true;
      }
      return false;
    }
    if (parseTokenId(token, "KA_M", id)) {
      for (const MultiActionDefinition& multi : candidateMultiActions) {
        if (multi.id == id) return true;
      }
      return false;
    }
    if (isValidSmartActionToken(token)) {
      return smartActionReferenceExists(token);
    }
    return true;
  };
  for (const String& token : candidateTokens) {
    if (!actionReferenceExists(token)) return false;
  }
  for (const String& token : candidateEncoderTokens) {
    if (!actionReferenceExists(token)) return false;
  }
  for (const JoystickSector& sector : candidateJoystickSectors) {
    if (!actionReferenceExists(sector.token)) return false;
  }
  for (const MacroDefinition& macro : candidateMacros) {
    for (const MacroStep& step : macro.steps) {
      if (!actionReferenceExists(step.token)) return false;
    }
  }
  for (const MultiActionDefinition& multi : candidateMultiActions) {
    if (!actionReferenceExists(multi.tapToken) ||
        !actionReferenceExists(multi.holdToken) ||
        !actionReferenceExists(multi.doubleTapToken) ||
        !actionReferenceExists(multi.tapHoldToken)) {
      return false;
    }
  }

  activeProfileId_ = requestedProfileId;
  activeProfileIndex_ = selectedProfileIndex;
  activeLayerId_ = requestedLayerId;
  activeLayerIndex_ = selectedLayerIndex;
  cycleLayerCount_ = layers.size();
  cycleLayerName_ = selectedLayer["name"] | "";
  cycleNextLayerName_ = layers[(selectedLayerIndex + 1) % layers.size()]["name"] | "";
  const bool hadPressedKey =
      std::any_of(configuredKeysPressed_.begin(),
                  configuredKeysPressed_.end(), [](bool pressed) {
                    return pressed;
                  }) ||
      std::any_of(macroHeldTokens_.begin(), macroHeldTokens_.end(),
                  [](const String& token) { return !token.isEmpty(); }) ||
      std::any_of(resolvedHeldTokens_.begin(), resolvedHeldTokens_.end(),
                  [](const String& token) { return !token.isEmpty(); });
  configuredTokens_ = candidateTokens;
  configuredEncoderTokens_ = candidateEncoderTokens;
  encoderLayerMode_ = configuredEncoderTokens_[2].startsWith("KI_LS");
  joystickType_ = joystickType;
  joystickSectors_ = std::move(candidateJoystickSectors);
  joystickSectorCount_ = joystickSectors_.size();
  activeJoystickSector_ = -1;
  configuredMacros_ = std::move(candidateMacros);
  configuredMacroCount_ = configuredMacros_.size();
  configuredMultiActions_ = std::move(candidateMultiActions);
  configuredMultiActionCount_ = configuredMultiActions_.size();
  configuredKeysPressed_.fill(false);
  macroHeldTokens_.clear();
  resolvedHeldTokens_.fill(String());
  multiActionStates_.fill(MultiActionState());
  if (!preserveMomentaryState_) {
    momentaryLayerActive_.fill(false);
    momentaryProfileIds_.fill(-1);
    momentaryLayerIds_.fill(-1);
    momentaryLayerFallbackActive_ = false;
    momentaryProfileId_ = -1;
    momentaryLayerId_ = -1;
  }
  actionQueue_.clear();
  nextActionAtMs_ = 0;
  if (candidateLightingConfigured && stateMutex_ != nullptr) {
    xSemaphoreTake(stateMutex_, portMAX_DELAY);
    CodexMicroState& lighting = nativeCodexMode_ ? generalLighting_ : state_;
    for (ThreadLight& thread : lighting.threads) {
      thread = candidateBacklight;
    }
    lighting.ambient = candidateUnderglow;
    lighting.ambientSyncThread = -1;
    ++state_.threadsRevision;
    ++state_.ambientRevision;
    state_.dirty = true;
    xSemaphoreGive(stateMutex_);
  }
  // A saved layer owns its lighting. A stale preview from a different layer
  // must not overwrite it during save/reconnect. Legacy files without layer
  // lighting still use the persisted preview as a fallback.
  if (!candidateLightingConfigured) restoreAmbientFromFlash();
  if (hadPressedKey) {
    uint8_t releasedKeyboard[8] = {};
    sendKeyboardReport(releasedKeyboard, sizeof(releasedKeyboard));
    sendConsumerReport(0);
    sendSystemReport(0);
  }
  configLoaded_ = true;
  if (manualLayerSelected_) {
    switchLightingMode(std::any_of(configuredTokens_.begin(), configuredTokens_.end(),
        [](const String& token) { return token.startsWith("KV_OAI_"); }));
  }
  Serial.printf(
      "CONFIG loaded path=%s profile_id=%ld profile_index=%u layer_id=%ld\n",
      path, static_cast<long>(activeProfileId_),
      static_cast<unsigned>(activeProfileIndex_),
      static_cast<long>(activeLayerId_));
  return true;
}

void CodexMicroBle::processDeferredRpc() {
  String json;
  {
    SemaphoreGuard guard(receiveMutex_);
    if (deferredRpcRequests_.empty()) return;
    json = std::move(deferredRpcRequests_.front());
    deferredRpcRequests_.pop_front();
  }
  if (json.isEmpty()) {
    return;
  }
  DynamicJsonDocument request(kRpcDocumentCapacity);
  const DeserializationError error = deserializeJson(request, json, DeserializationOption::NestingLimit(kJsonNestingLimit));
  if (error) {
    Serial.printf("RPC deferred parse error: %s\n", error.c_str());
    return;
  }
  handleRpc(request);
}

void CodexMicroBle::handleRpc(const JsonDocument& request) {
  const char* method = request["method"] | "";
  JsonVariantConst id = request["id"];
  JsonVariantConst params = request["params"];
  Serial.printf("RPC method=%s\n", method);

  // Keep the system calls from the official Input contract available even on
  // this display-less ESP32-S3 build. Self-test is intentionally a no-op
  // diagnostic acknowledgement; bootloader requests restart cleanly so the
  // host can re-discover the device before starting a programmer session.
  if (strcmp(method, "sys.selftest") == 0 ||
      strcmp(method, "selftest") == 0) {
    StaticJsonDocument<96> resultDoc;
    resultDoc["ok"] = true;
    resultDoc["mode"] = "selftest";
    sendResult(id, resultDoc.as<JsonVariantConst>());
    return;
  }
  if (strcmp(method, "sys.bootloader") == 0 ||
      strcmp(method, "bootloader") == 0) {
    StaticJsonDocument<96> resultDoc;
    resultDoc["ok"] = true;
    resultDoc["restarting"] = true;
    sendResult(id, resultDoc.as<JsonVariantConst>());
    rebootAtMs_ = millis() + 120;
    return;
  }

  if (strcmp(method, "sys.version") == 0) {
    StaticJsonDocument<128> resultDoc;
    resultDoc["version"] = kFirmwareVersion;
    sendResult(id, resultDoc.as<JsonVariantConst>());
    return;
  }

  if (strcmp(method, "device.status") == 0) {
    StaticJsonDocument<320> resultDoc;
    resultDoc["version"] = kFirmwareVersion;
    resultDoc["input_mode"] = nativeCodexMode_ ? "codex" : "general";
    resultDoc["profile_id"] = activeProfileId_;
    resultDoc["profile_index"] = activeProfileIndex_;
    resultDoc["layer_id"] = activeLayerId_;
    // Work axis normalizes the official device status' 1-based layer number
    // back to a zero-based UI index. Keep the wire contract 1-based.
    resultDoc["layer_index"] = static_cast<uint16_t>(activeLayerIndex_ + 1);
    resultDoc["config_loaded"] = configLoaded_;
    resultDoc["battery"] = batteryPercentage_;
    resultDoc["is_charging"] = charging_;
    resultDoc["filesystem"] = fsReady;
    resultDoc["filesystem_read"] = fsReady;
    resultDoc["filesystem_write"] = fsReady;
    sendResult(id, resultDoc.as<JsonVariantConst>());
    return;
  }

  if (strcmp(method, "diag.lighting") == 0) {
    // Observe both owners without switching modes or generating input.
    DynamicJsonDocument resultDoc(4096);
    {
    SemaphoreGuard guard(stateMutex_);
    resultDoc["input_mode"] = nativeCodexMode_ ? "codex" : "general";
    auto appendLighting = [&](const char* name, const CodexMicroState& source) {
      JsonObject target = resultDoc.createNestedObject(name);
      JsonArray threads = target.createNestedArray("threads");
      for (const ThreadLight& light : source.threads) {
        JsonObject item = threads.createNestedObject();
        item["c"] = light.color;
        item["b"] = light.brightness;
        item["e"] = light.effect;
        item["s"] = light.speed;
        item["m"] = light.magic;
      }
      JsonObject ambient = target.createNestedObject("ambient");
      ambient["c"] = source.ambient.color;
      ambient["b"] = source.ambient.brightness;
      ambient["e"] = source.ambient.effect;
      ambient["s"] = source.ambient.speed;
      ambient["m"] = source.ambient.magic;
      target["ambient_sync_thread"] = source.ambientSyncThread;
    };
    appendLighting("general", nativeCodexMode_ ? generalLighting_ : state_);
    appendLighting("codex", nativeCodexMode_ ? state_ : codexLighting_);
    }
    sendResult(id, resultDoc.as<JsonVariantConst>());
    return;
  }

  if (strcmp(method, "diag.inputs") == 0) {
    StaticJsonDocument<2048> resultDoc;
    codexMicroInputSnapshot(resultDoc.to<JsonObject>());
    resultDoc["mode_gesture"] = "encoder_hold_3000";
    resultDoc["layer_cycle"] = true;
    resultDoc["layer_number"] = activeLayerIndex_ + 1;
    resultDoc["layer_count"] = cycleLayerCount_;
    resultDoc["layer_name"] = cycleLayerName_;
    resultDoc["next_layer_number"] = cycleLayerCount_ ? (activeLayerIndex_ + 1) % cycleLayerCount_ + 1 : 0;
    resultDoc["next_layer_name"] = cycleNextLayerName_;
    resultDoc["profile_id"] = activeProfileId_;
    resultDoc["layer_id"] = activeLayerId_;
    resultDoc["input_mode"] = nativeCodexMode_ ? "codex" : "general";
    JsonArray tokens = resultDoc.createNestedArray("tokens");
    for (const String& token : configuredTokens_) tokens.add(token);
    sendResult(id, resultDoc.as<JsonVariantConst>());
    return;
  }

  if (strcmp(method, "fs.list") == 0) {
    DynamicJsonDocument resultDoc(4096);
    JsonArray files = resultDoc.to<JsonArray>();
    const bool includeChecksum = params["checksum"] | false;
    const bool recursive = params["rec"] | false;
    const String requestedPath = normalizeFsPath(params["path"] | "");
    if (fsReady) {
      File root = SPIFFS.open("/");
      if (root) {
        File file = root.openNextFile();
        while (file) {
          if (!file.isDirectory()) {
            String name = file.name();
            while (name.startsWith("/")) name.remove(0, 1);
            if (isInternalFsArtifact(name)) {
              file = root.openNextFile();
              continue;
            }
            const String fullName = "/" + name;
            if (!requestedPath.isEmpty() &&
                !(fullName == requestedPath ||
                  (recursive && fullName.startsWith(requestedPath + "/")))) {
              file = root.openNextFile();
              continue;
            }
            JsonObject item = files.createNestedObject();
            item["name"] = name;
            item["size"] = static_cast<uint32_t>(file.size());
            if (includeChecksum) item["checksum"] = sha1OfOpen(file);
          }
          file = root.openNextFile();
        }
        root.close();
      }
    }
    sendResult(id, files);
    return;
  }

  if (strcmp(method, "fs.read") == 0) {
    const String path = normalizeFsPath(params["file"] | "");
    if (!fsReady || path.isEmpty()) {
      sendError(id, -32602, "invalid_file_path");
      return;
    }
    File file = SPIFFS.open(path, FILE_READ);
    if (!file) {
      sendError(id, -32001, "File does not exist");
      return;
    }
    const size_t fileSize = file.size();
    if (fileSize == 0 || fileSize > (SIZE_MAX - 4096) / 2) {
      file.close();
      sendError(id, -32002, "file_too_large");
      return;
    }
    DynamicJsonDocument document(fileSize * 2 + 4096);
    const DeserializationError error = deserializeJson(document, file, DeserializationOption::NestingLimit(kJsonNestingLimit));
    file.close();
    if (error) {
      sendError(id, -32003, "file_invalid_json");
      return;
    }
    sendResult(id, document.as<JsonVariantConst>());
    return;
  }

  if (strcmp(method, "fs.write") == 0) {
    const String path = normalizeFsPath(params["file"] | "");
    const JsonVariantConst data = params["data"];
    if (!fsReady || path.isEmpty() || data.isNull()) {
      sendError(id, -32602, "invalid_file_data");
      return;
    }
    String encoded;
    serializeJson(data, encoded);
    const String stagingPath = path + ".part";
    if (SPIFFS.exists(stagingPath)) SPIFFS.remove(stagingPath);
    File file = SPIFFS.open(stagingPath, FILE_WRITE);
    const size_t written = file ? file.print(encoded) : 0;
    if (file) {
      file.flush();
      file.close();
    }
    bool valid = written == encoded.length();
    if (valid && path == "/keymap.json") {
      valid = loadDeviceConfiguration(stagingPath.c_str());
      if (!valid) loadDeviceConfiguration("/keymap.json");
    } else if (valid && path == "/smart_actions.json") {
      valid = validateSmartActionsFile(stagingPath.c_str());
    }
    const bool committed = valid && replaceFsFile(stagingPath, path);
    if (path == "/keymap.json") loadDeviceConfiguration("/keymap.json");
    if (!committed) {
      if (SPIFFS.exists(stagingPath)) SPIFFS.remove(stagingPath);
      sendError(id, -32603, valid ? "write_failed" : "config_invalid");
      return;
    }
    StaticJsonDocument<96> resultDoc;
    resultDoc["ok"] = true;
    resultDoc["bytes"] = static_cast<uint32_t>(written);
    sendResult(id, resultDoc.as<JsonVariantConst>());
    return;
  }

  if (strcmp(method, "fs.readbin") == 0) {
    DynamicJsonDocument resultDoc(kRpcDocumentCapacity);
    const String path = normalizeFsPath(params["file"] | "");
    const size_t offset = params["offset"] | 0;
    size_t requested = params["len"] | kFsReadChunk;
    if (requested > kFsReadChunk) requested = kFsReadChunk;
    File file = fsReady && !path.isEmpty() ? SPIFFS.open(path, FILE_READ) : File();
    if (!file) {
      StaticJsonDocument<128> errorDoc;
      errorDoc["ok"] = false;
      errorDoc["error"] = "file_not_found";
      sendResult(id, errorDoc.as<JsonVariantConst>());
      return;
    }
    const size_t total = file.size();
    if (offset > total || !file.seek(offset)) {
      file.close();
      StaticJsonDocument<128> errorDoc;
      errorDoc["ok"] = false;
      errorDoc["error"] = "invalid_offset";
      sendResult(id, errorDoc.as<JsonVariantConst>());
      return;
    }
    std::vector<uint8_t> bytes(requested);
    const size_t count = file.read(bytes.data(), requested);
    file.close();
    resultDoc["total_size"] = static_cast<uint32_t>(total);
    resultDoc["data"] = encodeBase64(bytes.data(), count);
    sendResult(id, resultDoc.as<JsonVariantConst>());
    return;
  }

  if (strcmp(method, "fs.writebin") == 0) {
    const String path = normalizeFsPath(params["file"] | "");
    const String encoded = params["data"] | "";
    const size_t offset = params["offset"] | 0;
    const bool append = params["append"] | true;
    const char* transaction = params["tx"].as<const char*>();
    const bool hasTransaction = transaction != nullptr && transaction[0] != '\0';
    if (hasTransaction && filesystemTransactionId_ != transaction) {
      sendError(id, -32010, "transaction_not_found");
      return;
    }
    const bool chunked = !params["completed"].isNull();
    const bool completed = params["completed"] | false;
    // SPIFFS has a short filename limit; a single active transaction means a
    // fixed suffix is sufficient and keeps smart_actions.json within it.
    const String stagingPath = hasTransaction
                                   ? path + ".txpart"
                                   : (chunked && !path.isEmpty() ? path + ".part" : path);
    std::vector<uint8_t> bytes;
    const bool valid = fsReady && !path.isEmpty() && decodeBase64(encoded, bytes) &&
                       !bytes.empty() && bytes.size() <= kFsReadChunk;
    const bool written = valid && writeFsChunk(stagingPath, bytes, offset, append);
    bool committed = written;
    if (written && chunked && completed) {
      bool validConfiguration = true;
      if (path == "/keymap.json") {
        validConfiguration = loadDeviceConfiguration(stagingPath.c_str());
        if (!validConfiguration) loadDeviceConfiguration("/keymap.json");
      } else if (path == "/smart_actions.json") {
        validConfiguration =
            validateSmartActionsFile(stagingPath.c_str());
      }
      if (hasTransaction) {
        committed = validConfiguration;
        if (committed && std::find(filesystemTransactionFiles_.begin(),
                                   filesystemTransactionFiles_.end(), path) ==
                             filesystemTransactionFiles_.end()) {
          filesystemTransactionFiles_.push_back(path);
        }
      } else {
        committed = validConfiguration && replaceFsFile(stagingPath, path);
        if (path == "/keymap.json") loadDeviceConfiguration("/keymap.json");
      }
      if (!committed && SPIFFS.exists(stagingPath)) SPIFFS.remove(stagingPath);
    }
    if (!committed) {
      sendError(id, -32603, fsReady ? "write_failed" : "filesystem_unavailable");
      return;
    }
    StaticJsonDocument<128> resultDoc;
    resultDoc["ok"] = true;
    resultDoc["bytes"] = static_cast<uint32_t>(bytes.size());
    resultDoc["completed"] = chunked && completed;
    sendResult(id, resultDoc.as<JsonVariantConst>());
    return;
  }

  if (strcmp(method, "fs.delete") == 0) {
    const String path = normalizeFsPath(params["file"] | "");
    const bool deleted = fsReady && !path.isEmpty() && SPIFFS.exists(path) && SPIFFS.remove(path);
    StaticJsonDocument<128> resultDoc;
    resultDoc["ok"] = deleted;
    sendResult(id, resultDoc.as<JsonVariantConst>());
    return;
  }

  if (strcmp(method, "fs.txbegin") == 0) {
    if (!fsReady) {
      sendError(id, -32000, "filesystem_unavailable");
      return;
    }
    if (!filesystemTransactionId_.isEmpty()) {
      sendError(id, -32011, "transaction_busy");
      return;
    }
    filesystemTransactionId_ = String("tx-") + String(millis(), HEX);
    filesystemTransactionFiles_.clear();
    StaticJsonDocument<128> resultDoc;
    resultDoc["tx"] = filesystemTransactionId_;
    resultDoc["ok"] = true;
    sendResult(id, resultDoc.as<JsonVariantConst>());
    return;
  }

  if (strcmp(method, "fs.txcommit") == 0) {
    const char* transaction = params["tx"].as<const char*>();
    if (transaction == nullptr || filesystemTransactionId_ != transaction) {
      sendError(id, -32010, "transaction_not_found");
      return;
    }
    bool committed = true;
    for (const String& path : filesystemTransactionFiles_) {
      const String stagingPath = path + ".txpart";
      if (!replaceFsFile(stagingPath, path)) {
        committed = false;
        break;
      }
      if (path == "/keymap.json") loadDeviceConfiguration("/keymap.json");
    }
    if (!committed) {
      sendError(id, -32603, "transaction_commit_failed");
      return;
    }
    filesystemTransactionFiles_.clear();
    filesystemTransactionId_.clear();
    sendSuccess(id);
    return;
  }

  if (strcmp(method, "fs.rmdir") == 0) {
    String rawPath = params["path"] | "";
    while (rawPath.startsWith("/")) rawPath.remove(0, 1);
    const String prefix = rawPath.isEmpty() ? String() : "/" + rawPath;
    bool removed = fsReady;
    if (fsReady) {
      File root = SPIFFS.open("/");
      if (root) {
        File file = root.openNextFile();
        while (file) {
          const String name = file.name();
          if (!file.isDirectory() &&
              (prefix.isEmpty() || name.startsWith(prefix + "/") ||
               name == prefix)) {
            if (!SPIFFS.remove(name)) removed = false;
          }
          file = root.openNextFile();
        }
        root.close();
      }
    }
    StaticJsonDocument<96> resultDoc;
    resultDoc["ok"] = removed;
    sendResult(id, resultDoc.as<JsonVariantConst>());
    return;
  }

  if (strcmp(method, "appmgr.list_active") == 0 ||
      strcmp(method, "appmgr.list_installed") == 0) {
    DynamicJsonDocument resultDoc(256);
    resultDoc.to<JsonArray>();
    sendResult(id, resultDoc.as<JsonVariantConst>());
    return;
  }

  if (strcmp(method, "ui.active_screen") == 0) {
    StaticJsonDocument<128> resultDoc;
    resultDoc["screen_name"] = uiActiveScreen_;
    sendResult(id, resultDoc.as<JsonVariantConst>());
    return;
  }

  if (strcmp(method, "ui.home_accent_color") == 0) {
    const char* color = params["color"].as<const char*>();
    if (color != nullptr && color[0] != '\0') uiHomeAccentColor_ = color;
    StaticJsonDocument<128> resultDoc;
    resultDoc["ok"] = true;
    resultDoc["color"] = uiHomeAccentColor_;
    sendResult(id, resultDoc.as<JsonVariantConst>());
    return;
  }

  if (strcmp(method, "mp.write_info") == 0) {
    if (params.is<JsonObjectConst>()) {
      const JsonObjectConst media = params.as<JsonObjectConst>();
      if (!media["song_title"].isNull()) mediaSongTitle_ = media["song_title"].as<String>();
      if (!media["artist"].isNull()) mediaArtist_ = media["artist"].as<String>();
      if (!media["elapsed"].isNull()) mediaElapsedSeconds_ = media["elapsed"] | 0;
      if (!media["total_duration"].isNull()) mediaDurationSeconds_ = media["total_duration"] | 0;
      if (!media["is_playing"].isNull()) mediaPlaying_ = media["is_playing"] | false;
    }
    sendSuccess(id);
    return;
  }

  if (strcmp(method, "mp.write_artwork") == 0) {
    const String encoded = params["data"] | "";
    std::vector<uint8_t> artwork;
    if (encoded.isEmpty() || !decodeBase64(encoded, artwork)) {
      sendError(id, -32602, "invalid_artwork_chunk");
      return;
    }
    sendSuccess(id);
    return;
  }

  if (strcmp(method, "fs.replace") == 0) {
    const String source = normalizeFsPath(params["source"] | "");
    const String target = normalizeFsPath(params["target"] | "");
    bool validConfiguration = true;
    if (target == "/keymap.json" && fsReady && SPIFFS.exists(source)) {
      validConfiguration = loadDeviceConfiguration(source.c_str());
      if (!validConfiguration) loadDeviceConfiguration("/keymap.json");
    } else if (target == "/smart_actions.json" && fsReady &&
               SPIFFS.exists(source)) {
      validConfiguration = validateSmartActionsFile(source.c_str());
    }
    const bool replaced = validConfiguration && replaceFsFile(source, target);
    if (target == "/keymap.json" && replaced) {
      loadDeviceConfiguration("/keymap.json");
    }
    if (target == "/keymap.json" && !replaced) {
      loadDeviceConfiguration("/keymap.json");
    }
    StaticJsonDocument<192> resultDoc;
    resultDoc["ok"] = replaced;
    if (target == "/keymap.json") resultDoc["config_loaded"] = configLoaded_;
    if (!replaced) {
      if (!fsReady) {
        resultDoc["error"] = "filesystem_unavailable";
      } else if (source.isEmpty() || target.isEmpty() || source == target) {
        resultDoc["error"] = "invalid_path";
      } else if (!SPIFFS.exists(source)) {
        resultDoc["error"] = "source_not_found";
      } else if (!validConfiguration) {
        resultDoc["error"] = "config_invalid";
      } else {
        resultDoc["error"] = "replace_failed";
      }
    }
    sendResult(id, resultDoc.as<JsonVariantConst>());
    return;
  }

  if (strcmp(method, "config.reload") == 0) {
    const bool loaded = loadDeviceConfiguration("/keymap.json");
    StaticJsonDocument<192> resultDoc;
    resultDoc["ok"] = loaded;
    resultDoc["loaded"] = configLoaded_;
    resultDoc["profile_id"] = activeProfileId_;
    resultDoc["profile_index"] = activeProfileIndex_;
    resultDoc["layer_id"] = activeLayerId_;
    resultDoc["layer_index"] = static_cast<uint16_t>(activeLayerIndex_ + 1);
    if (!loaded) resultDoc["error"] = "config_invalid_or_missing";
    sendResult(id, resultDoc.as<JsonVariantConst>());
    return;
  }

  if (strcmp(method, "v.oai.thstatus") == 0) {
    if (!params.is<JsonArrayConst>()) {
      sendError(id, -32602, "invalid_thread_lighting");
      return;
    }
    updateThreadLighting(params.as<JsonArrayConst>());
    sendSuccess(id);
    return;
  }

  if (strcmp(method, "v.oai.rgbcfg") == 0) {
    if (!params.is<JsonObjectConst>()) {
      sendError(id, -32602, "invalid_codex_lighting");
      return;
    }
    updateAmbientLighting(params["ambient"].as<JsonObjectConst>());
    sendSuccess(id);
    return;
  }

  if (strcmp(method, "lights.preview") == 0) {
    ThreadLight previewBacklight;
    LightingSide previewUnderglow;
    if (!params.is<JsonObjectConst>() ||
        !parseLightingConfig(params.as<JsonObjectConst>(), previewBacklight,
                             previewUnderglow)) {
      sendError(id, -32602, "invalid_lighting_config");
      return;
    }
    if (stateMutex_ != nullptr) {
      xSemaphoreTake(stateMutex_, portMAX_DELAY);
      CodexMicroState& lighting = nativeCodexMode_ ? generalLighting_ : state_;
      for (ThreadLight& thread : lighting.threads) {
        thread = previewBacklight;
      }
      lighting.ambient = previewUnderglow;
      lighting.ambientSyncThread = -1;
      ++state_.threadsRevision;
      ++state_.ambientRevision;
      state_.dirty = true;
      // Persist live previews so the underglow survives a reboot.
      requestAmbientPersist(millis());
      xSemaphoreGive(stateMutex_);
    }
    sendSuccess(id);
    return;
  }

  if (strcmp(method, "host.select_layer") == 0) {
    if (!params["profile_id"].is<int32_t>() || !params["layer_id"].is<int32_t>() ||
        params["profile_id"].as<int32_t>() < 0 || params["layer_id"].as<int32_t>() < 0) {
      sendError(id, -32602, "invalid_layer_selection"); return;
    }
    releaseLayerInputs();
    if (!loadDeviceConfiguration("/keymap.json", params["profile_id"].as<int32_t>(),
                                 params["layer_id"].as<int32_t>())) {
      sendError(id, -32004, "layer_selection_failed"); return;
    }
    const bool vendor = std::any_of(configuredTokens_.begin(), configuredTokens_.end(),
        [](const String& token) { return token.startsWith("KV_OAI_"); });
    switchLightingMode(vendor);
    manualLayerSelected_ = true;
    codexMicroShowLayerIndicator(vendor ? 0x0066FF : 0xFF9900);
    StaticJsonDocument<128> resultDoc;
    resultDoc["profile_id"] = activeProfileId_;
    resultDoc["layer_id"] = activeLayerId_;
    resultDoc["input_mode"] = nativeCodexMode_ ? "codex" : "general";
    sendResult(id, resultDoc.as<JsonVariantConst>()); return;
  }

  if (strcmp(method, "host.focused_app") == 0) {
    if (nativeCodexMode_ || encoderLayerMode_ || manualLayerSelected_) {
      // A physical layer choice stays selected when desktop focus changes.
      sendSuccess(id);
      return;
    }
    if (!fsReady || !params.is<JsonObjectConst>()) {
      sendError(id, -32602, "invalid_focus_app_params");
      return;
    }
    const char* appName = params["appName"].as<const char*>();
    const char* process = params["process"].as<const char*>();
    const char* executablePath = params["path"].as<const char*>();
    if ((appName == nullptr || appName[0] == '\0') &&
        (process == nullptr || process[0] == '\0') &&
        (executablePath == nullptr || executablePath[0] == '\0')) {
      sendError(id, -32602, "empty_focus_app_params");
      return;
    }

    File file = SPIFFS.open("/keymap.json", FILE_READ);
    if (!file) {
      sendError(id, -32001, "keymap_not_found");
      return;
    }
    const size_t fileSize = file.size();
    if (fileSize == 0 || fileSize > (SIZE_MAX - 4096) / 2) {
      file.close();
      sendError(id, -32002, "keymap_too_large");
      return;
    }
    DynamicJsonDocument document(fileSize * 2 + 4096);
    const DeserializationError error = deserializeJson(document, file, DeserializationOption::NestingLimit(kJsonNestingLimit));
    file.close();
    if (error) {
      sendError(id, -32003, "keymap_invalid");
      return;
    }

    auto matchesField = [](const char* incoming, JsonVariantConst expected,
                           bool& constrained) {
      const char* value = expected.as<const char*>();
      if (value == nullptr || value[0] == '\0') return true;
      constrained = true;
      return incoming != nullptr && incoming[0] != '\0' &&
             String(incoming).equalsIgnoreCase(value);
    };
    int32_t linkedAppId = -1;
    const JsonArrayConst linkedApps = document["linkedApps"].as<JsonArrayConst>();
    for (JsonObjectConst linkedApp : linkedApps) {
      bool constrained = false;
      const bool nameMatch = matchesField(appName, linkedApp["name"], constrained);
      const bool processMatch =
          matchesField(process, linkedApp["process"], constrained);
      const bool pathMatch =
          matchesField(executablePath, linkedApp["path"], constrained);
      if (constrained && nameMatch && processMatch && pathMatch) {
        linkedAppId = linkedApp["id"] | -1;
        break;
      }
    }

    int32_t matchedProfileId = -1;
    int32_t matchedLayerId = -1;
    if (linkedAppId >= 0) {
      const JsonArrayConst profiles = document["profiles"].as<JsonArrayConst>();
      for (JsonObjectConst profile : profiles) {
        const JsonArrayConst layers = profile["layers"].as<JsonArrayConst>();
        for (JsonObjectConst layer : layers) {
          if ((layer["linkedAppId"] | -1) == linkedAppId) {
            matchedProfileId = profile["id"] | -1;
            matchedLayerId = layer["id"] | -1;
            break;
          }
        }
        if (matchedLayerId >= 0) break;
      }
    }

    bool switched = false;
    if (matchedProfileId >= 0 && matchedLayerId >= 0) {
      switched = loadDeviceConfiguration("/keymap.json", matchedProfileId,
                                         matchedLayerId);
      if (!switched) {
        sendError(id, -32004, "linked_layer_invalid");
        return;
      }
    }
    StaticJsonDocument<224> resultDoc;
    resultDoc["ok"] = true;
    resultDoc["matched"] = switched;
    resultDoc["profile_id"] = activeProfileId_;
    resultDoc["profile_index"] = activeProfileIndex_;
    resultDoc["layer_id"] = activeLayerId_;
    resultDoc["layer_index"] = static_cast<uint16_t>(activeLayerIndex_ + 1);
    sendResult(id, resultDoc.as<JsonVariantConst>());
    return;
  }

  sendError(id, -32601, "method_not_found");
}

void CodexMicroBle::sendResult(JsonVariantConst id,
                               JsonVariantConst result) {
  DynamicJsonDocument response(8192);
  response["id"] = id;
  response["result"] = result;
  String json;
  serializeJson(response, json);
  sendJson(json);
}

void CodexMicroBle::sendSuccess(JsonVariantConst id) {
  StaticJsonDocument<96> resultDoc;
  resultDoc["ok"] = true;
  sendResult(id, resultDoc.as<JsonVariantConst>());
}

void CodexMicroBle::sendError(JsonVariantConst id, int code,
                              const char* message) {
  StaticJsonDocument<192> response;
  response["id"] = id;
  JsonObject error = response.createNestedObject("error");
  error["code"] = code;
  error["message"] = message;
  String json;
  serializeJson(response, json);
  sendJson(json);
}

void CodexMicroBle::sendJson(const String& json) {
  if (transmitMutex_ == nullptr || hidOta_.active()) {
    return;
  }
  const bool useUsb = transportAvailable(true);
  const bool useBle = !useUsb && transportAvailable(false);
  if (!useUsb && !useBle) {
    return;
  }

  xSemaphoreTake(transmitMutex_, portMAX_DELAY);
  if (transmitCount_ == kTransmitQueueDepth) {
    // Preserve forward progress if a host floods requests: discard the oldest
    // unsent response and retain the most recent request acknowledgements.
    transmitQueue_[transmitTail_].clear();
    transmitTail_ = (transmitTail_ + 1) % kTransmitQueueDepth;
    --transmitCount_;
  }
  transmitQueue_[transmitHead_] = json;
  transmitQueueUsb_[transmitHead_] = useUsb;
  transmitHead_ = (transmitHead_ + 1) % kTransmitQueueDepth;
  ++transmitCount_;
  xSemaphoreGive(transmitMutex_);
}

void CodexMicroBle::processHidOta() {
  uint8_t response[HidOtaUpdate::kReportBodySize] = {};
  if (!hidOta_.process(response)) {
    return;
  }

  const bool rebootAfterResponse =
      response[2] ==
          static_cast<uint8_t>(HidOtaUpdate::Command::kEnd) &&
      response[3] == static_cast<uint8_t>(HidOtaUpdate::Status::kOk) &&
      hidOta_.takeRebootRequest();
  const bool sent =
      transportAvailable(true) && usbHid_ != nullptr &&
      usbHid_->SendReport(kReportId, response, sizeof(response),
                          kUsbReportTimeoutMs);
  if (!sent) {
    Serial.println("HID OTA response send failed; host may retry");
    if (rebootAfterResponse) {
      // The boot partition has already been switched after full verification.
      // Reboot even if the final acknowledgement was lost.
      otaRestartAtMs_ = millis() + 1000;
    }
    return;
  }

  if (response[2] ==
          static_cast<uint8_t>(HidOtaUpdate::Command::kBegin) &&
      response[3] == static_cast<uint8_t>(HidOtaUpdate::Status::kOk) &&
      hidOta_.active()) {
    clearTransmitQueue();
  }

  if (response[2] ==
          static_cast<uint8_t>(HidOtaUpdate::Command::kData) &&
      response[3] == static_cast<uint8_t>(HidOtaUpdate::Status::kOk)) {
    const uint8_t progress = hidOta_.progressPercentage();
    static uint8_t lastLoggedProgress = 255;
    if (lastLoggedProgress == 255 || progress >= lastLoggedProgress + 10) {
      Serial.printf("HID OTA progress=%u%%\n", progress);
      lastLoggedProgress = progress;
    }
  }
  if (rebootAfterResponse) {
    otaRestartAtMs_ = millis() + 250;
  }
}

void CodexMicroBle::transmitJson(const String& json, bool usb) {
  if (!transportAvailable(usb)) {
    return;
  }

  if (!usb) {
    if (vendorInput_ == nullptr) {
      return;
    }
    // Windows persists the CCCD subscription for a bonded HID device and may
    // not rewrite it after the ESP32 reboots. Arduino-ESP32 does not persist
    // that descriptor, so its local guard otherwise drops every input report.
    // Receiving an RPC output report proves the bonded HID host is actively
    // using this channel; restore the cached notification state before reply.
    auto* notificationDescriptor = static_cast<BLE2902*>(
        vendorInput_->getDescriptorByUUID(
            BLEUUID(static_cast<uint16_t>(0x2902))));
    if (notificationDescriptor != nullptr &&
        !notificationDescriptor->getNotifications()) {
      notificationDescriptor->setNotifications(true);
      Serial.println("BLE input notification state restored");
    }
  }

  String framed = json;
  framed += '\n';
  size_t offset = 0;
  while (offset < framed.length()) {
    const size_t chunk =
        min<size_t>(kPayloadSize, framed.length() - offset);
    uint8_t report[kReportBodySize] = {};
    report[0] = 2;
    report[1] = chunk;
    memcpy(report + 2, framed.c_str() + offset, chunk);
    if (usb) {
      const bool sent =
          usbHid_ != nullptr &&
          usbHid_->SendReport(kReportId, report, sizeof(report),
                              kUsbReportTimeoutMs);
      Serial.printf("USB input report bytes=%u result=%s\n",
                    static_cast<unsigned>(sizeof(report)),
                    sent ? "ok" : "failed");
      if (!sent) {
        return;
      }
    } else {
      Serial.printf("BLE input report bytes=%u peer_mtu=%u\n",
                    static_cast<unsigned>(sizeof(report)), peerMtu_);
      vendorInput_->setValue(report, sizeof(report));
      vendorInput_->notify();
      delay(4);
    }
    offset += chunk;
  }
}

void CodexMicroBle::clearTransmitQueue() {
  if (transmitMutex_ == nullptr) {
    return;
  }
  xSemaphoreTake(transmitMutex_, portMAX_DELAY);
  for (String& queued : transmitQueue_) {
    queued.clear();
  }
  transmitQueueUsb_.fill(false);
  transmitHead_ = 0;
  transmitTail_ = 0;
  transmitCount_ = 0;
  pendingReceiveLog_.clear();
  xSemaphoreGive(transmitMutex_);
}

void CodexMicroBle::updateThreadLighting(JsonArrayConst values) {
  xSemaphoreTake(stateMutex_, portMAX_DELAY);
  CodexMicroState& lighting = nativeCodexMode_ ? state_ : codexLighting_;
  bool changed = false;
  for (JsonObjectConst value : values) {
    const int id = value["id"] | -1;
    if (id < 0 ||
        id >= static_cast<int>(lighting.threads.size())) {
      continue;
    }
    ThreadLight& light = lighting.threads[id];
    if (value.containsKey("c")) {
      light.color = value["c"].as<uint32_t>() & 0x00FFFFFFu;
    }
    if (value.containsKey("b")) {
      light.brightness = clampUnit(value["b"].as<float>());
    }
    if (value.containsKey("e")) {
      const int effect = value["e"].as<int>();
      if (effect >= 0 && effect <= 6) {
        light.effect = static_cast<uint8_t>(effect);
      }
    }
    if (value.containsKey("s")) {
      light.speed = clampUnit(value["s"].as<float>());
    }
    if (value.containsKey("sk")) {
      light.syncKeysLighting = value["sk"].as<int>() != 0;
    }
    if (value.containsKey("sa")) {
      const bool followAmbient = value["sa"].as<int>() != 0;
      light.syncAmbientLighting = followAmbient;
      if (followAmbient) {
        for (size_t other = 0; other < lighting.threads.size(); ++other) {
          if (other != static_cast<size_t>(id)) {
            lighting.threads[other].syncAmbientLighting = false;
          }
        }
        lighting.ambientSyncThread = static_cast<int8_t>(id);
      } else if (lighting.ambientSyncThread == id) {
        lighting.ambientSyncThread = -1;
      }
    }
    changed = true;
  }
  if (changed && nativeCodexMode_) {
    ++lighting.threadsRevision;
    state_.dirty = true;
  }
  xSemaphoreGive(stateMutex_);
}

void CodexMicroBle::updateAmbientLighting(JsonObjectConst value) {
  if (value.isNull()) {
    return;
  }

  xSemaphoreTake(stateMutex_, portMAX_DELAY);
  CodexMicroState& lighting = nativeCodexMode_ ? state_ : codexLighting_;
  bool changed = false;
  if (value.containsKey("c")) {
    lighting.ambient.color =
        value["c"].as<uint32_t>() & 0x00FFFFFFu;
    changed = true;
  }
  if (value.containsKey("b")) {
    lighting.ambient.brightness =
        clampUnit(value["b"].as<float>());
    changed = true;
  }
  if (value.containsKey("e")) {
    const int effect = value["e"].as<int>();
    if (effect >= 0 && effect <= 6) {
      lighting.ambient.effect = static_cast<uint8_t>(effect);
      changed = true;
    }
  }
  if (value.containsKey("s")) {
    lighting.ambient.speed = clampUnit(value["s"].as<float>());
    changed = true;
  }
  if (value.containsKey("m")) {
    lighting.ambient.magic = value["m"].as<float>();
    changed = true;
  }
  if (changed && nativeCodexMode_) {
    ++lighting.ambientRevision;
    state_.dirty = true;
  }
  xSemaphoreGive(stateMutex_);
}

void CodexMicroBle::requestAmbientPersist(uint32_t nowMs) {
  ambientPersistPending_ = true;
  // Coalesce a slider drag (many RPCs) into a single debounced SPIFFS write.
  constexpr uint32_t kDebounceMs = 400;
  const uint32_t target = nowMs + kDebounceMs;
  if (target > ambientPersistAtMs_) ambientPersistAtMs_ = target;
}

void CodexMicroBle::flushPendingAmbientPersist() {
  if (!ambientPersistPending_) return;
  if (millis() < ambientPersistAtMs_) return;
  ambientPersistPending_ = false;
  persistAmbientToFlash();
}

void CodexMicroBle::persistAmbientToFlash() {
  if (!fsReady) return;
  StaticJsonDocument<192> doc;
  {
    SemaphoreGuard guard(stateMutex_);
    CodexMicroState& lighting = nativeCodexMode_ ? generalLighting_ : state_;
    doc["c"] = lighting.ambient.color;
    doc["b"] = lighting.ambient.brightness;
    doc["e"] = lighting.ambient.effect;
    doc["s"] = lighting.ambient.speed;
    doc["m"] = lighting.ambient.magic;
  }
  File f = SPIFFS.open("/ambient.json", FILE_WRITE);
  if (!f) {
    Serial.println("AMBIENT persist: open failed");
    return;
  }
  if (serializeJson(doc, f) == 0) {
    Serial.println("AMBIENT persist: write failed");
  }
  f.close();
}

void CodexMicroBle::restoreAmbientFromFlash() {
  if (!fsReady) return;
  if (!SPIFFS.exists("/ambient.json")) return;
  File f = SPIFFS.open("/ambient.json", FILE_READ);
  if (!f) return;
  const size_t size = f.size();
  DynamicJsonDocument doc(size + 64);
  const DeserializationError err = deserializeJson(
      doc, f, DeserializationOption::NestingLimit(kJsonNestingLimit));
  f.close();
  if (err) {
    Serial.printf("AMBIENT restore: parse %s\n", err.c_str());
    return;
  }
  const JsonObjectConst o = doc.as<JsonObjectConst>();
  SemaphoreGuard guard(stateMutex_);
  CodexMicroState& lighting = nativeCodexMode_ ? generalLighting_ : state_;
  bool changed = false;
  if (o.containsKey("c")) {
    lighting.ambient.color = o["c"].as<uint32_t>() & 0x00FFFFFFu;
    changed = true;
  }
  if (o.containsKey("b")) {
    lighting.ambient.brightness = clampUnit(o["b"].as<float>());
    changed = true;
  }
  if (o.containsKey("e")) {
    const int e = o["e"].as<int>();
    if (e >= 0 && e <= 6) {
      lighting.ambient.effect = static_cast<uint8_t>(e);
      changed = true;
    }
  }
  if (o.containsKey("s")) {
    lighting.ambient.speed = clampUnit(o["s"].as<float>());
    changed = true;
  }
  if (o.containsKey("m")) {
    lighting.ambient.magic = o["m"].as<float>();
    changed = true;
  }
  if (changed) {
    ++lighting.ambientRevision;
    state_.dirty = true;
  }
}
