#pragma once
#include <stdint.h>
#include <string.h>

enum class MovementCommand { None, Start, Stop };

// One parser per transport: TCP packets and USB bytes may arrive in fragments.
class MovementCommands {
 public:
  void reset() { length_ = 0; invalid_ = false; lastByteMs_ = 0; }

  MovementCommand feed(char value, uint32_t nowMs) {
    lastByteMs_ = nowMs;
    if (value >= 'A' && value <= 'Z') value += 'a' - 'A';
    // Stop does not require Enter, even during an unfinished command.
    if (value == 'd') { reset(); return MovementCommand::Stop; }
    if (value == '\r' || value == '\n' || value == ' ' || value == '\t') return finish();
    if (length_ < 5 && !invalid_) buffer_[length_++] = value;
    else invalid_ = true;
    return MovementCommand::None;
  }

  MovementCommand poll(uint32_t nowMs) {
    // Also accept a terminal configured to send without a line ending.
    if ((length_ == 1 || length_ == 5 || invalid_) && uint32_t(nowMs - lastByteMs_) >= 100) return finish();
    return MovementCommand::None;
  }

 private:
  MovementCommand finish() {
    bool start = !invalid_ && ((length_ == 1 && buffer_[0] == 's') ||
                              (length_ == 5 && memcmp(buffer_, "start", 5) == 0));
    reset();
    return start ? MovementCommand::Start : MovementCommand::None;
  }
  char buffer_[5] = {};
  uint8_t length_ = 0;
  bool invalid_ = false;
  uint32_t lastByteMs_ = 0;
};

inline MovementCommand mergeCommands(MovementCommand first, MovementCommand next) {
  if (first == MovementCommand::Stop || next == MovementCommand::Stop) return MovementCommand::Stop;
  if (first == MovementCommand::Start || next == MovementCommand::Start) return MovementCommand::Start;
  return MovementCommand::None;
}
