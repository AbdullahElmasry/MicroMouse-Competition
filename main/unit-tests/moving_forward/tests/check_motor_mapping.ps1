$ErrorActionPreference = 'Stop'
$sketch = Get-Content -LiteralPath (Join-Path $PSScriptRoot '../moving_forward.ino') -Raw
$definitions = [regex]::Matches($sketch, 'constexpr (?:int|float) (?:LEFT_IN1|RIGHT_IN1|LEFT_FACTOR|RIGHT_FACTOR)\s*=[^;]+;')
if ($definitions.Count -ne 4) { throw 'Motor configuration not found' }
$begin = $sketch.IndexOf('void motor(')
$end = $sketch.IndexOf('uint8_t probeTof(', $begin)
if ($begin -lt 0 -or $end -lt 0) { throw 'Motor functions not found' }
$prefix = @'
#include <cassert>
#include <cstdlib>
#include <cstdio>
int outputs[40] = {};
void analogWrite(int pin, int pwm) { outputs[pin] = pwm; }
int constrain(int x, int low, int high) { return x<low?low:(x>high?high:x); }
'@
$checks = @'
int main() {
  drive(140,0);
  assert(outputs[27]==0 && outputs[14]==138);
  assert(outputs[25]==255 && outputs[26]==255);
  drive(0,140);
  assert(outputs[25]==140 && outputs[26]==0);
  assert(outputs[27]==255 && outputs[14]==255);
  drive(0,0);
  assert(outputs[25]==255 && outputs[26]==255 && outputs[27]==255 && outputs[14]==255);
  puts("PASS: actual sketch maps physical motors, preserves forward polarity/factors, and brakes");
}
'@
$testDirectory = Join-Path ([IO.Path]::GetTempPath()) ('motor-map-' + [guid]::NewGuid().ToString('N'))
[IO.Directory]::CreateDirectory($testDirectory) | Out-Null
$testSource = Join-Path $testDirectory 'mapping.cpp'
$testExecutable = Join-Path $testDirectory 'mapping.exe'
[IO.File]::WriteAllText($testSource, $prefix + "`n" + ($definitions.Value -join "`n") + "`n" + $sketch.Substring($begin,$end-$begin) + $checks)
& g++ -std=c++11 -Wall -Wextra -Werror $testSource -o $testExecutable
if ($LASTEXITCODE -ne 0) { throw 'Motor mapping compilation failed' }
& $testExecutable
if ($LASTEXITCODE -ne 0) { throw 'Motor mapping test failed' }
