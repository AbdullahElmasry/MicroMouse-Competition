#include <cassert>
#include <cstdio>

#include "../MazeMap.h"
#include "../config.h"
#include "../MoveForward.h"

int main() {
  MazeMap maze;
  maze.reset();

  assert(maze.cell(0, 0).distance == 14);
  assert(maze.cell(7, 7).distance == 0);
  assert((maze.cell(0, 0).walls & (1U << (uint8_t)Direction::West)) != 0);
  assert((maze.cell(0, 0).walls & (1U << (uint8_t)Direction::South)) != 0);

  Direction next = Direction::West;
  assert(maze.chooseNext(0, 0, Direction::North, next));
  assert(next == Direction::North);

  // Facing north at the start: front wall, left incorrectly reported open at
  // the maze boundary, and right open. The boundary remains closed.
  const int conflicts =
      maze.observe(0, 0, Direction::North, true, false, false);
  assert(conflicts == 1);
  assert((maze.cell(0, 0).walls & (1U << (uint8_t)Direction::North)) != 0);
  assert((maze.cell(1, 0).known & (1U << (uint8_t)Direction::West)) != 0);
  assert((maze.cell(1, 0).walls & (1U << (uint8_t)Direction::West)) == 0);
  maze.floodFill();
  assert(maze.chooseNext(0, 0, Direction::North, next));
  assert(next == Direction::East);

  maze.markTraversed(0, 0, Direction::East);
  assert((maze.cell(0, 0).walls & (1U << (uint8_t)Direction::East)) == 0);
  assert((maze.cell(1, 0).walls & (1U << (uint8_t)Direction::West)) == 0);

  // The run reported 103-110 mm as open, then stopped after only 40-67 mm.
  assert(104 <= FRONT_MAP_OPEN_MM && 110 <= FRONT_MAP_OPEN_MM);
  assert(129 > FRONT_MAP_OPEN_MM && 140 > FRONT_MAP_OPEN_MM);
  assert(!cellTravelMeetsMinimum(66.9f, 53.1f, 120.0f));
  assert(!cellTravelMeetsMinimum(59.6f, 39.7f, 120.0f));
  assert(cellTravelMeetsMinimum(130.8f, 144.3f, 120.0f));
  assert(!cellTravelMeetsMinimum(390.0f, 250.0f, 300.0f));

  // A later scan must not place a wall across an edge already crossed.
  assert(maze.observe(1, 0, Direction::North, false, true, false) == 1);
  assert(maze.observe(0, 0, Direction::East, true, false, false) >= 1);
  assert((maze.cell(0, 0).walls & (1U << (uint8_t)Direction::East)) == 0);
  assert((maze.cell(1, 0).walls & (1U << (uint8_t)Direction::West)) == 0);
  maze.floodFill();
  assert(maze.cell(0, 0).distance != FLOOD_UNREACHABLE);

  assert(MazeMap::leftOf(Direction::North) == Direction::West);
  assert(MazeMap::rightOf(Direction::North) == Direction::East);
  assert(MazeMap::opposite(Direction::East) == Direction::West);
  assert(MazeMap::dx(Direction::West) == -1);
  assert(MazeMap::dy(Direction::North) == 1);
  assert(maze.isGoal(7, 8));
  assert(!maze.isGoal(6, 8));

  MazeMap knownCorridor;
  knownCorridor.reset();
  assert(knownCorridor.knownStraightRunLength(0, 0, Direction::North, 3) == 1);
  knownCorridor.observe(0, 1, Direction::North, false, true, true);
  knownCorridor.markTraversed(0, 0, Direction::North);
  knownCorridor.markTraversed(0, 1, Direction::North);
  knownCorridor.floodFill();
  assert(knownCorridor.knownStraightRunLength(0, 0, Direction::North, 3) == 2);
  knownCorridor.observe(0, 2, Direction::North, false, true, true);
  knownCorridor.markTraversed(0, 2, Direction::North);
  knownCorridor.floodFill();
  assert(knownCorridor.knownStraightRunLength(0, 0, Direction::North, 3) == 3);

  // More no-wall base speed leaves room to steer while both motors remain
  // above the motor floor; the old 100/85 pair allowed only 15 PWM.
  const ForwardMotorCommands steering =
      noWallCombinedCommands(120, 100, 0.0f, -35.0f, 0.80f);
  assert(steering.left == 100 && steering.right == 120);

  std::puts("PASS: map edges, flood routing, known-corridor rolling and no-wall steering headroom");
}
