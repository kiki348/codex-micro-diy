#include "EncoderModeGesture.h"
#include <cassert>
int main() {
  using E = EncoderModeGesture::Event;
  EncoderModeGesture g;
  assert(g.edge(true, 10, false)==E::None);
  assert(g.edge(false, 100, false)==E::Click);
  g.edge(true, 100, false);
  assert(g.edge(false, 599, false)==E::Click);
  g.edge(true, 100, false);
  assert(g.edge(false, 600, false)==E::None);
  g.edge(true, 100, false);
  assert(g.edge(false, 3099, false)==E::None);
  assert(g.edge(true, 1000, false)==E::None);
  assert(g.poll(3999)==E::None);
  assert(g.poll(4000)==E::Toggle);
  assert(g.poll(5000)==E::None);
  assert(g.edge(false, 5001, false)==E::None);
  g.edge(true, 100, false);
  assert(g.edge(false, 3100, false)==E::Toggle);
  g.edge(true, 0xfffffff0u, false);
  assert(g.poll(uint32_t(0xfffffff0u+3000u))==E::Toggle);
  g.edge(true, 1, true); // shutdown hold consumes its press
  assert(g.poll(9000)==E::None);
  assert(g.edge(false, 9001, true)==E::None);
  g.edge(true, 10, false);
  g.cancel(); // disconnect / wake suppression
  assert(g.edge(false, 20, false)==E::None);
}
