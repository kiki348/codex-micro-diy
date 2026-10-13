// SPDX-License-Identifier: MIT
// Copyright (c) 2026 imliubo

#include "HidOtaUpdate.h"

#include <Update.h>
#include <esp_ota_ops.h>

#include <algorithm>
#include <cstring>

namespace {

constexpr uint8_t kEspImageMagic = 0xE9;

bool isKnownCommand(HidOtaUpdate::Command command) {
  using Command = HidOtaUpdate::Command;
  switch (command) {
    case Command::kInfo:
    case Command::kBegin:
    case Command::kData:
    case Command::kEnd:
    case Command::kAbort:
    case Command::kStatus:
      return true;
    default:
      return false;
  }
}

}  // namespace

void HidOtaUpdate::begin() {
  if (queueMutex_ == nullptr) {
    queueMutex_ = xSemaphoreCreateMutex();
  }
  mbedtls_sha256_init(&shaContext_);
}

bool HidOtaUpdate::enqueue(const uint8_t* body, size_t length) {
  if (queueMutex_ == nullptr || body == nullptr || length == 0 ||
      body[0] != kRequestType) {
    return false;
  }

  if (xSemaphoreTake(queueMutex_, 0) != pdTRUE) {
    return false;
  }
  if (pendingFrameValid_) {
    xSemaphoreGive(queueMutex_);
    return false;
  }

  pendingFrame_.fill(0);
  memcpy(pendingFrame_.data(), body,
         std::min(length, pendingFrame_.size()));
  pendingFrameValid_ = true;
  xSemaphoreGive(queueMutex_);
  return true;
}

bool HidOtaUpdate::process(uint8_t response[kReportBodySize]) {
  if (queueMutex_ == nullptr || response == nullptr) {
    return false;
  }

  std::array<uint8_t, kReportBodySize> requestBody{};
  xSemaphoreTake(queueMutex_, portMAX_DELAY);
  if (!pendingFrameValid_) {
    xSemaphoreGive(queueMutex_);
    return false;
  }
  requestBody = pendingFrame_;
  pendingFrameValid_ = false;
  xSemaphoreGive(queueMutex_);

  Frame request;
  Status status = parseRequest(requestBody.data(), request);
  uint32_t responseOffset = active_ ? bytesWritten_ : 0;
  uint8_t payload[kMaxPayloadSize] = {};
  uint8_t payloadLength = 0;
  if (status == Status::kOk) {
    status = handleRequest(request, payload, payloadLength,
                           responseOffset);
  }
  buildResponse(request, status, responseOffset, payload, payloadLength,
                response);
  return true;
}

void HidOtaUpdate::onUsbDisconnected() {
  if (active_) {
    Serial.println("HID OTA aborted: USB host disconnected");
    resetSession(true);
  }
  if (queueMutex_ != nullptr) {
    xSemaphoreTake(queueMutex_, portMAX_DELAY);
    pendingFrameValid_ = false;
    xSemaphoreGive(queueMutex_);
  }
}

uint8_t HidOtaUpdate::progressPercentage() const {
  if (!active_ || imageSize_ == 0) {
    return completed_ ? 100 : 0;
  }
  return static_cast<uint8_t>(
      std::min<uint32_t>(100, (bytesWritten_ * 100ULL) / imageSize_));
}

bool HidOtaUpdate::takeRebootRequest() {
  if (!rebootRequested_) {
    return false;
  }
  rebootRequested_ = false;
  return true;
}

HidOtaUpdate::Status HidOtaUpdate::parseRequest(
    const uint8_t body[kReportBodySize], Frame& frame) {
  frame.type = body[0];
  frame.version = body[1];
  frame.command = static_cast<Command>(body[2]);
  frame.flagsOrStatus = body[3];
  frame.session = loadLe32(body + 4);
  frame.sequence = loadLe32(body + 8);
  frame.offset = loadLe32(body + 12);
  frame.payloadLength = body[16];
  frame.crc = loadLe32(body + 18);
  frame.payload = body + kHeaderSize;

  if (frame.type != kRequestType) {
    return Status::kBadFrame;
  }
  if (frame.version != kProtocolVersion) {
    return Status::kBadVersion;
  }
  if (!isKnownCommand(frame.command)) {
    return Status::kBadCommand;
  }
  if (frame.payloadLength > kMaxPayloadSize) {
    return Status::kBadLength;
  }
  if (frame.crc != frameCrc(body, frame.payloadLength)) {
    return Status::kCrcError;
  }
  return Status::kOk;
}

HidOtaUpdate::Status HidOtaUpdate::handleRequest(
    const Frame& request, uint8_t* payload, uint8_t& payloadLength,
    uint32_t& responseOffset) {
  switch (request.command) {
    case Command::kInfo:
    case Command::kStatus:
      return handleInfo(payload, payloadLength, responseOffset);
    case Command::kBegin:
      return handleBegin(request, responseOffset);
    case Command::kData:
      return handleData(request, responseOffset);
    case Command::kEnd:
      return handleEnd(request, responseOffset);
    case Command::kAbort:
      return handleAbort(request, responseOffset);
    default:
      return Status::kBadCommand;
  }
}

HidOtaUpdate::Status HidOtaUpdate::handleInfo(
    uint8_t* payload, uint8_t& payloadLength, uint32_t& responseOffset) {
  const esp_partition_t* partition =
      esp_ota_get_next_update_partition(nullptr);
  if (partition == nullptr) {
    return Status::kInternalError;
  }

  payload[0] = static_cast<uint8_t>(kMaxPayloadSize);
  payload[1] = active_ ? 1 : (completed_ ? 2 : 0);
  storeLe32(payload + 2, partition->size);
  storeLe32(payload + 6, bytesWritten_);
  payloadLength = 10;
  responseOffset = bytesWritten_;
  return Status::kOk;
}

HidOtaUpdate::Status HidOtaUpdate::handleBegin(
    const Frame& request, uint32_t& responseOffset) {
  if (request.payloadLength != expectedSha_.size() ||
      request.offset == 0) {
    return Status::kBadLength;
  }
  const esp_partition_t* partition =
      esp_ota_get_next_update_partition(nullptr);
  if (partition == nullptr || request.offset > partition->size) {
    return Status::kSizeError;
  }

  if (active_) {
    if (request.session == session_ && request.offset == imageSize_ &&
        memcmp(request.payload, expectedSha_.data(),
               expectedSha_.size()) == 0) {
      responseOffset = bytesWritten_;
      return Status::kOk;
    }
    return Status::kBusy;
  }

  completed_ = false;
  rebootRequested_ = false;
  if (!Update.begin(request.offset, U_FLASH)) {
    Serial.printf("HID OTA begin failed: %s\n", Update.errorString());
    return Status::kBeginFailed;
  }

  session_ = request.session;
  imageSize_ = request.offset;
  bytesWritten_ = 0;
  nextSequence_ = request.sequence + 1;
  memcpy(expectedSha_.data(), request.payload, expectedSha_.size());
  lastAcceptedValid_ = false;
  mbedtls_sha256_free(&shaContext_);
  mbedtls_sha256_init(&shaContext_);
  if (mbedtls_sha256_starts_ret(&shaContext_, 0) != 0) {
    resetSession(true);
    return Status::kInternalError;
  }
  shaInitialized_ = true;
  active_ = true;
  responseOffset = 0;
  Serial.printf("HID OTA started session=%08lX bytes=%lu\n",
                static_cast<unsigned long>(session_),
                static_cast<unsigned long>(imageSize_));
  return Status::kOk;
}

HidOtaUpdate::Status HidOtaUpdate::handleData(
    const Frame& request, uint32_t& responseOffset) {
  responseOffset = bytesWritten_;
  if (!active_) {
    return Status::kNotActive;
  }
  if (request.session != session_) {
    return Status::kBadSession;
  }
  if (request.payloadLength == 0) {
    return Status::kBadLength;
  }

  if (lastAcceptedValid_ &&
      request.sequence == lastAcceptedSequence_ &&
      request.offset == lastAcceptedOffset_ &&
      request.crc == lastAcceptedFrameCrc_) {
    return Status::kOk;
  }
  if (request.sequence != nextSequence_) {
    return Status::kBadSequence;
  }
  if (request.offset != bytesWritten_) {
    return Status::kBadOffset;
  }
  if (request.offset + request.payloadLength > imageSize_) {
    return Status::kSizeError;
  }
  if (request.offset == 0 && request.payload[0] != kEspImageMagic) {
    return Status::kBadImage;
  }

  const size_t written =
      Update.write(const_cast<uint8_t*>(request.payload),
                   request.payloadLength);
  if (written != request.payloadLength) {
    Serial.printf("HID OTA write failed at %lu: %s\n",
                  static_cast<unsigned long>(request.offset),
                  Update.errorString());
    resetSession(true);
    return Status::kWriteFailed;
  }
  if (mbedtls_sha256_update_ret(&shaContext_, request.payload,
                                request.payloadLength) != 0) {
    resetSession(true);
    return Status::kInternalError;
  }

  lastAcceptedSequence_ = request.sequence;
  lastAcceptedOffset_ = request.offset;
  lastAcceptedFrameCrc_ = request.crc;
  lastAcceptedValid_ = true;
  bytesWritten_ += request.payloadLength;
  ++nextSequence_;
  responseOffset = bytesWritten_;
  return Status::kOk;
}

HidOtaUpdate::Status HidOtaUpdate::handleEnd(
    const Frame& request, uint32_t& responseOffset) {
  responseOffset = bytesWritten_;
  if (!active_) {
    if (completed_ && request.session == completedSession_ &&
        request.sequence == completedSequence_ &&
        request.offset == completedImageSize_) {
      rebootRequested_ = true;
      responseOffset = completedImageSize_;
      return Status::kOk;
    }
    return Status::kNotActive;
  }
  if (request.session != session_) {
    return Status::kBadSession;
  }
  if (request.sequence != nextSequence_) {
    return Status::kBadSequence;
  }
  if (request.payloadLength != 0 || request.offset != imageSize_ ||
      bytesWritten_ != imageSize_) {
    return Status::kBadOffset;
  }

  std::array<uint8_t, 32> actualSha{};
  if (!shaInitialized_ ||
      mbedtls_sha256_finish_ret(&shaContext_, actualSha.data()) != 0) {
    resetSession(true);
    return Status::kInternalError;
  }
  mbedtls_sha256_free(&shaContext_);
  mbedtls_sha256_init(&shaContext_);
  shaInitialized_ = false;
  if (actualSha != expectedSha_) {
    Serial.println("HID OTA SHA-256 verification failed");
    resetSession(true);
    return Status::kVerifyFailed;
  }

  if (!Update.end(false)) {
    Serial.printf("HID OTA finalize failed: %s\n", Update.errorString());
    resetSession(true);
    return Status::kVerifyFailed;
  }

  completedSession_ = session_;
  completedSequence_ = request.sequence;
  completedImageSize_ = imageSize_;
  completed_ = true;
  rebootRequested_ = true;
  active_ = false;
  responseOffset = completedImageSize_;
  Serial.printf("HID OTA verified; boot partition switched, bytes=%lu\n",
                static_cast<unsigned long>(completedImageSize_));
  return Status::kOk;
}

HidOtaUpdate::Status HidOtaUpdate::handleAbort(
    const Frame& request, uint32_t& responseOffset) {
  if (active_ && request.session != session_) {
    return Status::kBadSession;
  }
  resetSession(active_);
  completed_ = false;
  rebootRequested_ = false;
  responseOffset = 0;
  Serial.println("HID OTA aborted by host");
  return Status::kOk;
}

void HidOtaUpdate::buildResponse(
    const Frame& request, Status status, uint32_t responseOffset,
    const uint8_t* payload, uint8_t payloadLength,
    uint8_t response[kReportBodySize]) {
  memset(response, 0, kReportBodySize);
  response[0] = kResponseType;
  response[1] = kProtocolVersion;
  response[2] = static_cast<uint8_t>(request.command);
  response[3] = static_cast<uint8_t>(status);
  storeLe32(response + 4, request.session);
  storeLe32(response + 8, request.sequence);
  storeLe32(response + 12, responseOffset);
  response[16] = std::min<uint8_t>(payloadLength, kMaxPayloadSize);
  response[17] = 0;
  if (payload != nullptr && response[16] > 0) {
    memcpy(response + kHeaderSize, payload, response[16]);
  }
  storeLe32(response + 18, frameCrc(response, response[16]));
}

void HidOtaUpdate::resetSession(bool abortUpdate) {
  if (abortUpdate && Update.isRunning()) {
    Update.abort();
  }
  finishSha();
  active_ = false;
  session_ = 0;
  nextSequence_ = 0;
  imageSize_ = 0;
  bytesWritten_ = 0;
  expectedSha_.fill(0);
  lastAcceptedValid_ = false;
}

void HidOtaUpdate::finishSha() {
  if (shaInitialized_) {
    mbedtls_sha256_free(&shaContext_);
    mbedtls_sha256_init(&shaContext_);
    shaInitialized_ = false;
  }
}

uint16_t HidOtaUpdate::loadLe16(const uint8_t* data) {
  return static_cast<uint16_t>(data[0]) |
         (static_cast<uint16_t>(data[1]) << 8);
}

uint32_t HidOtaUpdate::loadLe32(const uint8_t* data) {
  return static_cast<uint32_t>(data[0]) |
         (static_cast<uint32_t>(data[1]) << 8) |
         (static_cast<uint32_t>(data[2]) << 16) |
         (static_cast<uint32_t>(data[3]) << 24);
}

void HidOtaUpdate::storeLe16(uint8_t* data, uint16_t value) {
  data[0] = static_cast<uint8_t>(value);
  data[1] = static_cast<uint8_t>(value >> 8);
}

void HidOtaUpdate::storeLe32(uint8_t* data, uint32_t value) {
  data[0] = static_cast<uint8_t>(value);
  data[1] = static_cast<uint8_t>(value >> 8);
  data[2] = static_cast<uint8_t>(value >> 16);
  data[3] = static_cast<uint8_t>(value >> 24);
}

uint32_t HidOtaUpdate::frameCrc(const uint8_t* body,
                                uint8_t payloadLength) {
  uint32_t crc = 0xFFFFFFFFu;
  auto update = [&crc](uint8_t value) {
    crc ^= value;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc >> 1) ^
            (0xEDB88320u & (0u - (crc & 1u)));
    }
  };
  for (size_t i = 0; i < 18; ++i) {
    update(body[i]);
  }
  for (size_t i = 0; i < payloadLength; ++i) {
    update(body[kHeaderSize + i]);
  }
  return crc ^ 0xFFFFFFFFu;
}
