#include "MazeMap.h"

#include <Preferences.h>
#include <string.h>

namespace {
constexpr uint32_t SAVED_MAZE_MAGIC = 0x4D4D0101;
struct SavedMaze {
  uint32_t magic;
  uint8_t cells[MAZE_SIZE * MAZE_SIZE * 4];
};
}

uint8_t MazeMap::wallBit(Direction direction) {
  return (uint8_t)(1U << (uint8_t)direction);
}

Direction MazeMap::leftOf(Direction direction) {
  return (Direction)(((uint8_t)direction + 3U) % 4U);
}

Direction MazeMap::rightOf(Direction direction) {
  return (Direction)(((uint8_t)direction + 1U) % 4U);
}

Direction MazeMap::opposite(Direction direction) {
  return (Direction)(((uint8_t)direction + 2U) % 4U);
}

int MazeMap::dx(Direction direction) {
  if (direction == Direction::East) return 1;
  if (direction == Direction::West) return -1;
  return 0;
}

int MazeMap::dy(Direction direction) {
  if (direction == Direction::North) return 1;
  if (direction == Direction::South) return -1;
  return 0;
}

const char *MazeMap::directionName(Direction direction) {
  static const char *names[] = {"NORTH", "EAST", "SOUTH", "WEST"};
  return names[(uint8_t)direction];
}

bool MazeMap::inBounds(int x, int y) const {
  return x >= 0 && x < MAZE_SIZE && y >= 0 && y < MAZE_SIZE;
}

bool MazeMap::isGoal(int x, int y) const {
  return (x == 7 || x == 8) && (y == 7 || y == 8);
}

void MazeMap::reset() {
  memset(cells_, 0, sizeof(cells_));
  for (int x = 0; x < MAZE_SIZE; ++x) {
    setWall(x, 0, Direction::South, true);
    setWall(x, MAZE_SIZE - 1, Direction::North, true);
  }
  for (int y = 0; y < MAZE_SIZE; ++y) {
    setWall(0, y, Direction::West, true);
    setWall(MAZE_SIZE - 1, y, Direction::East, true);
  }
  floodFill();
}

bool MazeMap::setWall(int x, int y, Direction direction, bool present) {
  if (!inBounds(x, y)) return false;
  const int nx = x + dx(direction);
  const int ny = y + dy(direction);
  const bool boundaryConflict = !inBounds(nx, ny) && !present;
  if (!inBounds(nx, ny)) present = true;
  MazeCell &here = cells_[y][x];
  const uint8_t mask = wallBit(direction);
  // A sensor reading cannot close an edge the robot physically crossed.
  if (present && (here.traversed & mask)) return true;
  const bool conflict = boundaryConflict ||
      ((here.known & mask) && (((here.walls & mask) != 0) != present));
  here.known |= mask;
  if (present) here.walls |= mask;
  else here.walls &= (uint8_t)~mask;

  if (inBounds(nx, ny)) {
    MazeCell &there = cells_[ny][nx];
    const uint8_t oppositeMask = wallBit(opposite(direction));
    there.known |= oppositeMask;
    if (present) there.walls |= oppositeMask;
    else there.walls &= (uint8_t)~oppositeMask;
  }
  return conflict;
}

int MazeMap::observe(int x, int y, Direction heading,
                     bool frontWall, bool leftWall, bool rightWall) {
  if (!inBounds(x, y)) return 0;
  cells_[y][x].visited = true;
  int conflicts = 0;
  conflicts += setWall(x, y, heading, frontWall) ? 1 : 0;
  conflicts += setWall(x, y, leftOf(heading), leftWall) ? 1 : 0;
  conflicts += setWall(x, y, rightOf(heading), rightWall) ? 1 : 0;
  return conflicts;
}

void MazeMap::markTraversed(int x, int y, Direction direction) {
  if (!inBounds(x, y)) return;
  const int nx = x + dx(direction);
  const int ny = y + dy(direction);
  if (!inBounds(nx, ny)) return;
  setWall(x, y, direction, false);
  cells_[y][x].traversed |= wallBit(direction);
  cells_[ny][nx].traversed |= wallBit(opposite(direction));
}

bool MazeMap::canTravel(int x, int y, Direction direction) const {
  if (!inBounds(x, y)) return false;
  const int nx = x + dx(direction);
  const int ny = y + dy(direction);
  if (!inBounds(nx, ny)) return false;
  return (cells_[y][x].walls & wallBit(direction)) == 0;
}

void MazeMap::floodFill() {
  struct Position { uint8_t x, y; };
  Position queue[MAZE_SIZE * MAZE_SIZE];
  int head = 0, tail = 0;

  for (int y = 0; y < MAZE_SIZE; ++y)
    for (int x = 0; x < MAZE_SIZE; ++x)
      cells_[y][x].distance = FLOOD_UNREACHABLE;

  for (int y = 7; y <= 8; ++y) {
    for (int x = 7; x <= 8; ++x) {
      cells_[y][x].distance = 0;
      queue[tail++] = {(uint8_t)x, (uint8_t)y};
    }
  }

  while (head < tail) {
    const Position current = queue[head++];
    const uint16_t nextDistance = cells_[current.y][current.x].distance + 1;
    for (uint8_t raw = 0; raw < 4; ++raw) {
      const Direction direction = (Direction)raw;
      if (!canTravel(current.x, current.y, direction)) continue;
      const int nx = current.x + dx(direction);
      const int ny = current.y + dy(direction);
      if (cells_[ny][nx].distance <= nextDistance) continue;
      cells_[ny][nx].distance = nextDistance;
      queue[tail++] = {(uint8_t)nx, (uint8_t)ny};
    }
  }
}

bool MazeMap::chooseNext(int x, int y, Direction heading, Direction &next) const {
  if (!inBounds(x, y)) return false;
  const Direction preference[] = {
      heading, rightOf(heading), leftOf(heading), opposite(heading)};
  bool found = false;
  bool bestVisited = true;
  uint16_t bestDistance = FLOOD_UNREACHABLE;

  for (Direction direction : preference) {
    if (!canTravel(x, y, direction)) continue;
    const int nx = x + dx(direction);
    const int ny = y + dy(direction);
    const MazeCell &candidate = cells_[ny][nx];
    if (!found || candidate.distance < bestDistance ||
        (candidate.distance == bestDistance && bestVisited && !candidate.visited)) {
      found = true;
      next = direction;
      bestDistance = candidate.distance;
      bestVisited = candidate.visited;
    }
  }
  return found && bestDistance != FLOOD_UNREACHABLE;
}

const MazeCell &MazeMap::cell(int x, int y) const {
  return cells_[y][x];
}

bool MazeMap::save() const {
  SavedMaze saved = {};
  saved.magic = SAVED_MAZE_MAGIC;
  for (int y = 0; y < MAZE_SIZE; ++y) {
    for (int x = 0; x < MAZE_SIZE; ++x) {
      const int index = (y * MAZE_SIZE + x) * 4;
      const MazeCell &cell = cells_[y][x];
      saved.cells[index] = cell.walls;
      saved.cells[index + 1] = cell.known;
      saved.cells[index + 2] = cell.traversed;
      saved.cells[index + 3] = cell.visited ? 1 : 0;
    }
  }
  Preferences storage;
  if (!storage.begin("micromouse", false)) return false;
  const bool ok = storage.putBytes("maze", &saved, sizeof(saved)) == sizeof(saved);
  storage.end();
  return ok;
}

bool MazeMap::load() {
  Preferences storage;
  if (!storage.begin("micromouse", true)) return false;
  SavedMaze saved = {};
  const bool ok = storage.getBytesLength("maze") == sizeof(saved) &&
      storage.getBytes("maze", &saved, sizeof(saved)) == sizeof(saved) &&
      saved.magic == SAVED_MAZE_MAGIC;
  storage.end();
  if (!ok) return false;
  for (int y = 0; y < MAZE_SIZE; ++y) {
    for (int x = 0; x < MAZE_SIZE; ++x) {
      const int index = (y * MAZE_SIZE + x) * 4;
      MazeCell &cell = cells_[y][x];
      cell.walls = saved.cells[index];
      cell.known = saved.cells[index + 1];
      cell.traversed = saved.cells[index + 2];
      cell.visited = saved.cells[index + 3] != 0;
    }
  }
  floodFill();
  return true;
}

void MazeMap::clearSaved() {
  Preferences storage;
  if (!storage.begin("micromouse", false)) return;
  storage.remove("maze");
  storage.end();
}
