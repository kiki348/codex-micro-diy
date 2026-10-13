#include "SpatialLighting.h"
#include "BoardLightingGeometry.h"
#include <cstdio>
#include <fstream>
#include <stdexcept>
#include <limits>

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
void close(float actual, float expected, const char* message) {
  require(std::fabs(actual - expected) < 0.0000002f, message);
}
int main(int argc, char** argv) {
  try {
    require(argc == 2, "Pass the hash-verified original merged image");
    std::ifstream image(argv[1], std::ios::binary);
    require(image.good(), "Reference image unavailable");
    image.seekg(0x77170);  // 0x3c157170 in the hash-pinned image's rodata segment.
    for (int row = 0; row < 256; ++row) {
      for (int col = 0; col < 256; ++col) {
        float original;
        image.read(reinterpret_cast<char*>(&original), sizeof(original));
        require(image.good(), "Truncated reference table");
        // Interior of each quantization cell, including the endpoint cell.
        const float y = row == 255 ? 1.0f : (row + 0.25f) / 127.5f - 1.0f;
        const float x = col == 255 ? 1.0f : (col + 0.25f) / 127.5f - 1.0f;
        close(spatialLighting::angularTurns(y, x),
              static_cast<float>(original / 6.28318530717958647692),
              "Angular coordinate or table mismatch");
      }
    }
    using namespace spatialLighting;
    const Point p{0.3f, 0.4f};
    close(rainbowTurns(p, 0.0f, 0.1f), 0.8f, "Diagonal mode");
    close(rainbowTurns(p, 0.3f, 0.1f), 0.5f, "Vertical mode");
    close(rainbowTurns(p, 0.6f, 0.1f), 0.6f, "Radial mode");
    close(rainbowTurns(p, 1.0f, 0.1f), wrap(angularTurns(p.y,p.x)+0.1f), "Angular mode");
    close(rainbowTurns({-0.8f,-0.8f},0,0),0.4f,"Negative hue wraps");
    close(gradientTurns(p,0),0,"Zero magic preserves base hue");
    close(gradientTurns(p,1),wrap(angularTurns(p.x,p.y))*126/127,"Gradient angle order");
    require(magicByte(-1)==0 && magicByte(2)==126, "Clamp out-of-range magic");
    require(magicByte(0.25f)/32==0 && magicByte(0.26f)/32==1 &&
            magicByte(0.50f)/32==1 && magicByte(0.51f)/32==2 &&
            magicByte(0.76f)/32==2 && magicByte(0.77f)/32==3,
            "UI percent mode boundaries");
    require(magicByte(std::numeric_limits<float>::quiet_NaN())==0,"NaN guard");
    require(sizeof(keyPoints)/sizeof(Point)==6 && sizeof(ambientPoints)/sizeof(Point)==8,"Board geometry counts");
    std::puts("PASS: 65536 original angular samples, four rainbow modes, gradient and boundary checks");
    return 0;
  } catch (const std::exception& error) {
    std::fprintf(stderr,"FAIL: %s\n",error.what());
    return 1;
  }
}
