#pragma once

#include <cmath>
#include <cstdint>

// Spatial behavior traced from the local 0.6.3-rc.12 reference image.
// See scripts/audit_official_lighting_tables.py for the independent LUT oracle.
namespace spatialLighting {
struct Point { float x; float y; };

inline float unit(float value) {
  return !std::isfinite(value) || value < 0.0f ? 0.0f
       : value > 1.0f ? 1.0f : value;
}
inline float wrap(float value) {
  value = std::fmod(value, 1.0f);
  return value < 0.0f ? value + 1.0f : value;
}
inline uint8_t magicByte(float magic) {
  return static_cast<uint8_t>(unit(magic) * 126.0f);
}
inline float angularTurns(float rowCoordinate, float columnCoordinate) {
  // Match the original 256x256 LUT coordinate quantization and four decimals.
  const int row = static_cast<int>((unit((rowCoordinate + 1.0f) * 0.5f)) * 255.0f);
  const int column = static_cast<int>((unit((columnCoordinate + 1.0f) * 0.5f)) * 255.0f);
  const double angle = std::atan2(row - 127.5, column - 127.5);
  const float rounded = static_cast<float>(std::round(angle * 10000.0) / 10000.0);
  return static_cast<float>(rounded / 6.28318530717958647692);
}
inline float gradientTurns(Point point, float magic) {
  // Gradient indexes row=x, column=y; angular rainbow uses the opposite order.
  return wrap(angularTurns(point.x, point.y)) * magicByte(magic) / 127.0f;
}
inline float rainbowTurns(Point point, float magic, float phase) {
  float position;
  switch (magicByte(magic) / 32) {
    case 0: position = point.x + point.y; break;
    case 1: position = point.y; break;
    case 2: position = std::sqrt(point.x * point.x + point.y * point.y); break;
    default: position = angularTurns(point.y, point.x); break;
  }
  return wrap(position + phase);
}
}  // namespace spatialLighting
