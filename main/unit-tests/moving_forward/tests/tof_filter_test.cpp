#include "../TofFilter.h"
#include <assert.h>
#include <stdio.h>
int main() {
  TofFilter filter;
  assert(filter.update(40)==40);
  assert(filter.update(40)==40);
  assert(filter.update(40)==40);
  assert(filter.update(130)==40); // Isolated false distant wall.
  assert(filter.update(41)==41);
  filter.reset();
  filter.update(40); filter.update(40); filter.update(40);
  assert(filter.update(90)==40);
  assert(filter.update(90)==90); // Real step accepted on second sample.
  assert(filter.update(200,false)==200); // No target never held as old wall.
  assert(filter.update(60)==60);
  assert(filter.update(-1)==-1); // Fault must not become a valid distance.
  assert(filter.update(30)==30);
  puts("PASS: ToF spike rejection, step response, no-target/fault bypass, reset");
}
