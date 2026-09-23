#pragma once

#include <stdint.h>

constexpr int MAZE_SIZE = 16;
constexpr uint16_t FLOOD_UNREACHABLE = 0xFFFF;

enum class Direction : uint8_t { North = 0, East = 1, South = 2, West = 3 };

struct MazeCell {
  uint8_t walls = 0;
  uint8_t known = 0;
  uint8_t traversed = 0;
  bool visited = false;
  uint16_t distance = FLOOD_UNREACHABLE;
};

class MazeMap {
 public:
  void reset();
  bool inBounds(int x, int y) const;
  bool isGoal(int x, int y) const;
  int observe(int x, int y, Direction heading,
              bool frontWall, bool leftWall, bool rightWall);
  void markTraversed(int x, int y, Direction direction);
  void floodFill();
  bool chooseNext(int x, int y, Direction heading, Direction &next) const;
  const MazeCell &cell(int x, int y) const;

  static Direction leftOf(Direction direction);
  static Direction rightOf(Direction direction);
  static Direction opposite(Direction direction);
  static int dx(Direction direction);
  static int dy(Direction direction);
  static const char *directionName(Direction direction);

 private:
  bool setWall(int x, int y, Direction direction, bool present);
  bool canTravel(int x, int y, Direction direction) const;
  static uint8_t wallBit(Direction direction);

  MazeCell cells_[MAZE_SIZE][MAZE_SIZE];
};
