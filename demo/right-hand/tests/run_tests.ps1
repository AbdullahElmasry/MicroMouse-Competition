$ErrorActionPreference = 'Stop'
$testBuild = Join-Path ([IO.Path]::GetTempPath()) ('right-hand-tests-' + [guid]::NewGuid().ToString('N'))
[IO.Directory]::CreateDirectory($testBuild) | Out-Null
$testExecutable = Join-Path $testBuild 'navigation_test.exe'
& g++ -static -std=c++11 -Wall -Wextra -Werror "-I$PSScriptRoot/fakes" (Join-Path $PSScriptRoot 'navigation_test.cpp') -o $testExecutable
if ($LASTEXITCODE -ne 0) { throw 'Navigation tests failed to compile' }
& $testExecutable
if ($LASTEXITCODE -ne 0) { throw "Navigation tests failed (exit $LASTEXITCODE)" }
& (Join-Path $PSScriptRoot 'check_scan_retries.ps1')
& (Join-Path $PSScriptRoot 'check_forward_arrivals.ps1')
