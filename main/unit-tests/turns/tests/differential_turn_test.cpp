#include "../DifferentialTurn.h"
#include <assert.h>
#include <stdio.h>

int main() {
  auto command=differentialTurnCommands(true,60);
  assert(command.left==60 && command.right== -60);
  command=differentialTurnCommands(false,60);
  assert(command.left== -60 && command.right==60);
  command=differentialTurnCommands(true,0);
  assert(command.left==0 && command.right==0);
  puts("PASS: simultaneous counter-rotating left/right wheel commands");
}
