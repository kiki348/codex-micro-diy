#pragma once
#include <stdint.h>

// Power/wake gestures own their edges before this recognizer runs.
class EncoderModeGesture {
 public:
  enum class Event { None, Click, Toggle };
  void cancel() { pending_ = false; }
  Event poll(uint32_t now) {
    if (pending_ && uint32_t(now - started_) >= 3000) {
      pending_ = false;
      return Event::Toggle;
    }
    return Event::None;
  }
  Event edge(bool pressed, uint32_t now, bool consumed) {
    if (consumed) { cancel(); return Event::None; }
    if (pressed) { pending_ = true; started_ = now; return Event::None; }
    if (!pending_) return Event::None;
    const uint32_t duration = uint32_t(now - started_);
    // A cancelled hold must not turn into a confirmation click.
    const Event result = duration >= 3000 ? Event::Toggle :
        (duration < 500 ? Event::Click : Event::None);
    cancel();
    return result;
  }
 private:
  bool pending_ = false;
  uint32_t started_ = 0;
};
