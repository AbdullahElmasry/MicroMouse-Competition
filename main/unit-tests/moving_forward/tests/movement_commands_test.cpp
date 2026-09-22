#include "../MovementCommands.h"
#include <assert.h>
#include <stdio.h>

MovementCommand send(MovementCommands &parser, const char *text, uint32_t now) {
  MovementCommand result = MovementCommand::None;
  while (*text) result = mergeCommands(result, parser.feed(*text++, now));
  return result;
}

int main() {
  MovementCommands usb, wifi;
  assert(usb.poll(3000) == MovementCommand::None); // No automatic start.
  assert(send(usb,"s\n",0)==MovementCommand::Start);
  assert(send(wifi,"S",0)==MovementCommand::None);
  assert(wifi.poll(99)==MovementCommand::None);
  assert(wifi.poll(100)==MovementCommand::Start);
  assert(send(wifi,"sd",0)==MovementCommand::Stop);
  assert(send(usb, "sta", 1) == MovementCommand::None);
  assert(usb.poll(150) == MovementCommand::None); // Human typing / delayed packet.
  assert(send(usb, "rt\r\n", 200) == MovementCommand::Start);
  assert(send(usb, "d", 30) == MovementCommand::Stop);
  assert(send(wifi, "START\n", 40) == MovementCommand::Start);
  assert(send(wifi, "start", 50) == MovementCommand::None);
  assert(wifi.poll(149) == MovementCommand::None);
  assert(wifi.poll(150) == MovementCommand::Start);
  assert(wifi.poll(151) == MovementCommand::None);
  assert(send(wifi, "starter\n", 160) == MovementCommand::None);
  assert(send(wifi, "start\nd", 170) == MovementCommand::Stop);
  assert(send(wifi, "dstart\n", 180) == MovementCommand::Stop);
  send(usb, "sta", 190);
  assert(send(wifi, "rt\n", 191) == MovementCommand::None); // Separate transports.
  usb.reset();
  assert(send(usb, "rt\n", 200) == MovementCommand::None);
  assert(send(usb, "staD", 210) == MovementCommand::Stop);
  assert(send(usb, "start\n", 220) == MovementCommand::Start); // New run allowed.
  assert(mergeCommands(MovementCommand::Start, MovementCommand::Stop) == MovementCommand::Stop);
  puts("PASS: explicit start, immediate d, fragmented input, transport isolation, stop priority, reset");
}
