// SPDX-License-Identifier: MIT
// Copyright (c) 2026 imliubo

#pragma once

#ifndef CODEX_BLE_ENABLED
#define CODEX_BLE_ENABLED 1
#endif

// Full JSON UART traces are opt-in: synchronous 115200-baud writes can block
// the input-scanning loop for tens or hundreds of milliseconds per request.
#ifndef CODEX_VERBOSE_DIAGNOSTICS
#define CODEX_VERBOSE_DIAGNOSTICS 0
#endif

#include <Arduino.h>
#include <ArduinoJson.h>
#include <BLECharacteristic.h>
#include <BLEHIDDevice.h>
#include <BLEServer.h>
#include <USB.h>
#include <USBHID.h>

#include <array>
#include <deque>
#include <vector>

#include "HidOtaUpdate.h"

struct ThreadLight {
  uint32_t color = 0;
  float brightness = 0.0f;
  uint8_t effect = 0;
  float speed = 0.0f;
  bool syncKeysLighting = false;
  bool syncAmbientLighting = false;
  float magic = 1.0f;
};

struct LightingSide {
  uint32_t color = 0;
  float brightness = 0.0f;
  uint8_t effect = 0;
  float speed = 0.0f;
  float magic = 0.0f;
};

struct CodexMicroState {
  std::array<ThreadLight, 6> threads;
  LightingSide ambient;
  LightingSide keys;
  int8_t ambientSyncThread = -1;
  bool connected = false;
  bool dirty = true;
  uint32_t threadsRevision = 0;
  uint32_t ambientRevision = 0;
};

class CodexMicroBle {
 public:
  static constexpr uint16_t kVendorId = 0x303A;
  static constexpr uint16_t kProductId = 0x8360;
  static constexpr uint8_t kKeyboardReportId = 1;
  static constexpr uint8_t kConsumerReportId = 2;
  static constexpr uint8_t kSystemReportId = 3;
  static constexpr uint8_t kReportId = 6;

  void begin(bool clearBleBonds = false);
  void poll();
  // Flushes a debounced underglow snapshot to SPIFFS. Called from the main
  // loop so a lighting slider drag persists once, not on every RPC tick.
  void flushPendingAmbientPersist();
  void setBattery(uint8_t percentage, bool charging);
  void sendKey(const char* key, uint8_t action, int8_t agent = -1);
  void sendJoystick(float angle, float distance);
  void toggleCodexMode();
  bool connected();
  CodexMicroState snapshot();

 private:
  void releaseLayerInputs();
  class ServerCallbacks;
  class InputCallbacks;
  class OutputCallbacks;
  class UsbHidDevice;

  void onConnected(bool connected);
  void onUsbOutput(uint8_t reportId, const uint8_t* data, size_t length);
  void onUsbEvent(int32_t eventId);
  void reconcileUsbMountedState();
  static void usbEventCallback(void* arg, esp_event_base_t eventBase,
                               int32_t eventId, void* eventData);
  void applyPendingTransportChange();
  bool transportAvailable(bool usb);
  void onMtuChanged(uint16_t mtu);
  void onNotifyStatus(BLECharacteristicCallbacks::Status status,
                      uint32_t code);
  void onOutput(const uint8_t* data, size_t length, bool usb);
  // profileOverride/layerOverride are transient runtime selections used by
  // Work axis app-focus switching. A value below zero keeps the persisted
  // activeProfileId/activeLayerId selection from the document.
  bool loadDeviceConfiguration(const char* path, int32_t profileOverride = -1,
                               int32_t layerOverride = -1);
  bool validateSmartActionsFile(const char* path);
  bool smartActionReferenceExists(const String& token) const;
  bool sendSmartAction(const String& token);
  void handleRpc(const JsonDocument& request);
  void processDeferredRpc();
  void sendResult(JsonVariantConst id, JsonVariantConst result);
  void sendSuccess(JsonVariantConst id);
  void sendError(JsonVariantConst id, int code, const char* message);
  void sendJson(const String& json);
  void transmitJson(const String& json, bool usb);
  // Returns true only when the configured token was actually executed on the
  // device. Host-only smart actions deliberately return false.
  bool sendConfiguredKey(const char* key, uint8_t action);
  bool handleInternalToken(const String& token, uint8_t action,
                           int physicalIndex = -1);
  bool sendCodexVendorToken(const String& token, uint8_t action);
  bool switchToLayerIndex(uint16_t layerIndex);
  bool switchToProfileIndex(uint16_t profileIndex);
  bool configureBleChannel(uint8_t channel, bool pairing);
  void sendCheatSheetEvent(const char* method);
  void sendRadialCloseEvent();
  void switchLightingMode(bool codexMode);
  bool canExecuteDeviceToken(const String& token) const;
  bool enqueueTokenAction(const String& token, uint8_t action,
                          uint32_t delayMs, size_t depth = 0);
  bool enqueueMacro(int32_t macroId, uint32_t trailingDelayMs = 0,
                    size_t depth = 0);
  void processActionQueue();
  void applyQueuedAction(const String& token, uint8_t action);
  void handleMultiAction(size_t physicalIndex, const String& token,
                         uint8_t action);
  void processMultiActions();
  bool dispatchResolvedToken(const String& token, uint8_t action,
                             size_t depth = 0, int sourceIndex = -1);
  bool joystickSectorContains(float angle, float start, float end) const;
  void applyConfiguredJoystick(float angle, float distance);
  void emitCombinedReports();
  void sendKeyboardReport(const uint8_t* report, size_t length);
  void sendConsumerReport(uint16_t usage);
  void sendSystemReport(uint8_t usage);
  void processHidOta();
  void clearTransmitQueue();
  void updateThreadLighting(JsonArrayConst values);
  void updateAmbientLighting(JsonObjectConst value);
  // Undeglow persistence: the desktop app only sends a live preview
  // (lights.preview / v.oai.rgbcfg) for most lighting edits, so the runtime
  // state_.ambient must be Snapshotted to /ambient.json to survive a reboot.
  void persistAmbientToFlash();
  void restoreAmbientFromFlash();
  void requestAmbientPersist(uint32_t nowMs);

  BLEHIDDevice* hid_ = nullptr;
  BLEServer* server_ = nullptr;
  BLECharacteristic* keyboardInput_ = nullptr;
  BLECharacteristic* consumerInput_ = nullptr;
  BLECharacteristic* systemInput_ = nullptr;
  BLECharacteristic* vendorInput_ = nullptr;
  BLECharacteristic* vendorOutput_ = nullptr;
  USBHID* usbHid_ = nullptr;
  UsbHidDevice* usbDevice_ = nullptr;
  SemaphoreHandle_t stateMutex_ = nullptr;
  SemaphoreHandle_t transmitMutex_ = nullptr;
  SemaphoreHandle_t receiveMutex_ = nullptr;
  CodexMicroState state_;
  // Lighting only is copied on mode changes; transport state stays in state_.
  CodexMicroState generalLighting_;
  CodexMicroState codexLighting_;
  String rpcBuffer_;
  bool rpcBufferUsb_ = false;
  // A fully received JSON-RPC request is parked here and processed from poll()
  // instead of the HID output callback. Heavy handlers (keymap config reload
  // with file I/O + JSON parsing) overflow the USB/BLE callback stack and
  // watchdog-reset the device when executed synchronously in onOutput().
  // Keep complete requests in order; a second client must not overwrite an
  // unprocessed filesystem write while the main loop is busy.
  std::deque<String> deferredRpcRequests_;
  static constexpr size_t kDeferredRpcLimit = 8;
  uint8_t batteryPercentage_ = 100;
  bool charging_ = false;
  bool bleConnected_ = false;
  bool usbMounted_ = false;
  bool usbSuspended_ = false;
  bool pendingTransportChange_ = false;
  bool configLoaded_ = false;
  bool manualLayerSelected_ = false;
  bool encoderLayerMode_ = false;
  bool nativeCodexMode_ = false;
  bool joystickHeld_ = false;
  bool modeSuppressedJoystick_ = false;
  std::array<bool, 12> modeSuppressedReleases_{};
  uint32_t encoderLayerPressAt_ = 0;
  String encoderLayerPressToken_;
  std::array<String, 13> pressedRouteTokens_;
  int32_t activeProfileId_ = 0;
  uint16_t activeProfileIndex_ = 0;
  int32_t activeLayerId_ = 0;
  uint16_t activeLayerIndex_ = 0;
  uint16_t cycleLayerCount_ = 0;
  String cycleLayerName_;
  String cycleNextLayerName_;
  String joystickType_ = "VENDOR";
  bool radialMenuVisible_ = false;
  int8_t preferredTransport_ = -1;  // -1=automatic, 0=USB, 1=BLE
  uint8_t bleChannel_ = 0;
  std::array<String, 12> configuredTokens_{};
  std::array<String, 3> configuredEncoderTokens_{};
  std::array<bool, 12> configuredKeysPressed_{};
  std::array<int32_t, 12> momentaryProfileIds_{};
  std::array<int32_t, 12> momentaryLayerIds_{};
  std::array<bool, 12> momentaryLayerActive_{};
  int32_t momentaryProfileId_ = -1;
  int32_t momentaryLayerId_ = -1;
  bool momentaryLayerFallbackActive_ = false;
  bool preserveMomentaryState_ = false;

  // Official Input uses filesystem transactions when it deploys a preset or
  // app bundle. Keep the transaction state in RAM and commit each staged file
  // atomically into SPIFFS; the payload itself remains persistent across reset.
  String filesystemTransactionId_;
  std::vector<String> filesystemTransactionFiles_;
  String mediaSongTitle_;
  String mediaArtist_;
  uint32_t mediaElapsedSeconds_ = 0;
  uint32_t mediaDurationSeconds_ = 0;
  bool mediaPlaying_ = false;
  String uiActiveScreen_ = "home";
  String uiHomeAccentColor_ = "#FFFFFF";

  struct MacroStep {
    String token;
    uint8_t action = 2;
    uint32_t delayMs = 0;
  };
  struct MacroDefinition {
    int32_t id = -1;
    size_t stepCount = 0;
    std::vector<MacroStep> steps;
  };
  struct QueuedAction {
    String token;
    uint8_t action = 2;
    uint32_t delayMs = 0;
  };
  std::vector<MacroDefinition> configuredMacros_;
  size_t configuredMacroCount_ = 0;
  std::deque<QueuedAction> actionQueue_;
  uint32_t nextActionAtMs_ = 0;
  std::vector<String> macroHeldTokens_;

  struct MultiActionDefinition {
    int32_t id = -1;
    String tapToken;
    String holdToken;
    String doubleTapToken;
    String tapHoldToken;
    uint32_t tappingTermMs = 250;
  };
  struct MultiActionState {
    int32_t id = -1;
    uint8_t phase = 0;
    uint32_t pressedAtMs = 0;
    uint32_t releasedAtMs = 0;
  };
  std::vector<MultiActionDefinition> configuredMultiActions_;
  size_t configuredMultiActionCount_ = 0;
  std::array<MultiActionState, 12> multiActionStates_{};
  // Independent owners: physical multi-action keys 0..11 and joystick 12.
  // Macros retain their own explicit press/release state in macroHeldTokens_.
  std::array<String, 13> resolvedHeldTokens_{};

  struct JoystickSector {
    String token;
    float start = 0.0f;
    float end = 0.0f;
  };
  std::vector<JoystickSector> joystickSectors_;
  size_t joystickSectorCount_ = 0;
  int32_t activeJoystickSector_ = -1;
  HidOtaUpdate hidOta_;
  uint32_t otaRestartAtMs_ = 0;
  uint32_t rebootAtMs_ = 0;
  bool ambientPersistPending_ = false;
  uint32_t ambientPersistAtMs_ = 0;

  static constexpr size_t kTransmitQueueDepth = 32;
  std::array<String, kTransmitQueueDepth> transmitQueue_;
  std::array<bool, kTransmitQueueDepth> transmitQueueUsb_;
  size_t transmitHead_ = 0;
  size_t transmitTail_ = 0;
  size_t transmitCount_ = 0;
  String pendingReceiveLog_;
  uint16_t peerMtu_ = 23;
  bool pendingMtuLog_ = false;
  BLECharacteristicCallbacks::Status pendingNotifyStatus_ =
      BLECharacteristicCallbacks::Status::ERROR_NO_CLIENT;
  uint32_t pendingNotifyCode_ = 0;
  bool pendingNotifyStatusValid_ = false;

  static CodexMicroBle* usbEventOwner_;
};
