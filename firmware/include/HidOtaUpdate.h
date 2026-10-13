// SPDX-License-Identifier: MIT
// Copyright (c) 2026 imliubo

#pragma once

#include <Arduino.h>

#include <array>

#include <mbedtls/sha256.h>

class HidOtaUpdate {
 public:
  static constexpr size_t kReportBodySize = 63;
  static constexpr size_t kHeaderSize = 22;
  static constexpr size_t kMaxPayloadSize =
      kReportBodySize - kHeaderSize;
  static constexpr uint8_t kRequestType = 0xF0;
  static constexpr uint8_t kResponseType = 0xF1;
  static constexpr uint8_t kProtocolVersion = 1;

  enum class Command : uint8_t {
    kInfo = 0x01,
    kBegin = 0x02,
    kData = 0x03,
    kEnd = 0x04,
    kAbort = 0x05,
    kStatus = 0x06,
  };

  enum class Status : uint8_t {
    kOk = 0,
    kBadFrame = 1,
    kBadVersion = 2,
    kBadCommand = 3,
    kBusy = 4,
    kBadSession = 5,
    kBadSequence = 6,
    kBadOffset = 7,
    kBadLength = 8,
    kCrcError = 9,
    kSizeError = 10,
    kBeginFailed = 11,
    kWriteFailed = 12,
    kVerifyFailed = 13,
    kNotActive = 14,
    kBadImage = 15,
    kInternalError = 16,
  };

  void begin();
  bool enqueue(const uint8_t* body, size_t length);
  bool process(uint8_t response[kReportBodySize]);
  void onUsbDisconnected();
  bool active() const { return active_; }
  uint8_t progressPercentage() const;
  bool takeRebootRequest();

 private:
  struct Frame {
    uint8_t type = 0;
    uint8_t version = 0;
    Command command = Command::kInfo;
    uint8_t flagsOrStatus = 0;
    uint32_t session = 0;
    uint32_t sequence = 0;
    uint32_t offset = 0;
    uint8_t payloadLength = 0;
    uint32_t crc = 0;
    const uint8_t* payload = nullptr;
  };

  Status parseRequest(const uint8_t body[kReportBodySize], Frame& frame);
  Status handleRequest(const Frame& request, uint8_t* payload,
                       uint8_t& payloadLength, uint32_t& responseOffset);
  Status handleInfo(uint8_t* payload, uint8_t& payloadLength,
                    uint32_t& responseOffset);
  Status handleBegin(const Frame& request, uint32_t& responseOffset);
  Status handleData(const Frame& request, uint32_t& responseOffset);
  Status handleEnd(const Frame& request, uint32_t& responseOffset);
  Status handleAbort(const Frame& request, uint32_t& responseOffset);
  void buildResponse(const Frame& request, Status status,
                     uint32_t responseOffset, const uint8_t* payload,
                     uint8_t payloadLength,
                     uint8_t response[kReportBodySize]);
  void resetSession(bool abortUpdate);
  void finishSha();

  static uint16_t loadLe16(const uint8_t* data);
  static uint32_t loadLe32(const uint8_t* data);
  static void storeLe16(uint8_t* data, uint16_t value);
  static void storeLe32(uint8_t* data, uint32_t value);
  static uint32_t frameCrc(const uint8_t* body, uint8_t payloadLength);

  SemaphoreHandle_t queueMutex_ = nullptr;
  std::array<uint8_t, kReportBodySize> pendingFrame_{};
  bool pendingFrameValid_ = false;

  bool active_ = false;
  uint32_t session_ = 0;
  uint32_t nextSequence_ = 0;
  uint32_t imageSize_ = 0;
  uint32_t bytesWritten_ = 0;
  std::array<uint8_t, 32> expectedSha_{};
  mbedtls_sha256_context shaContext_{};
  bool shaInitialized_ = false;

  uint32_t lastAcceptedSequence_ = 0;
  uint32_t lastAcceptedOffset_ = 0;
  uint32_t lastAcceptedFrameCrc_ = 0;
  bool lastAcceptedValid_ = false;

  uint32_t completedSession_ = 0;
  uint32_t completedSequence_ = 0;
  uint32_t completedImageSize_ = 0;
  bool completed_ = false;
  bool rebootRequested_ = false;
};
