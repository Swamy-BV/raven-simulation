#requires -Version 7.4
# SPDX-License-Identifier: AGPL-3.0-only
[CmdletBinding()]
param(
    [string]$Executable,
    [string]$SimulatorHost,
    [string]$BoardAddress,
    [string]$RobotTemplate,
    [string]$Scenario,
    [Parameter(Mandatory)][string]$OutputDirectory,
    [ValidateRange(1, 3600)][int]$Seconds = 600,
    [string]$GStreamerRoot,
    [string]$GStreamerPython,
    [string]$SerialPort,
    [string]$AssetDirectory,
    [switch]$Stop
)
$ErrorActionPreference = 'Stop'
$output = [IO.Path]::GetFullPath($OutputDirectory)
$stateFile = $output + '.state.json'
if ($Stop) {
    $state = Get-Content -LiteralPath $stateFile -Raw | ConvertFrom-Json
    $running = Get-Process -Id $state.process_id -ErrorAction SilentlyContinue
    if (-not $running) { return }
    if ($running.Path -ne $state.executable) { throw 'Bridge process identity changed' }
    Set-Content -LiteralPath (Join-Path $output 'stop') -Value ''
    # Each actor connection joins its SDK worker threads during shutdown.
    if (-not $running.WaitForExit(45000)) { throw 'Bridge did not stop gracefully; inspect bridge.log' }
    Write-Output 'Bridge stopped; recording finalized.'
    return
}
foreach ($value in @($Executable, $SimulatorHost, $BoardAddress, $RobotTemplate, $Scenario)) {
    if ([string]::IsNullOrWhiteSpace($value)) { throw 'Executable, simulator, board, template and scenario are required' }
}
if (Test-Path -LiteralPath $output) { throw 'Use a new run directory' }
if ($GStreamerRoot -and $GStreamerPython) { throw 'Choose one GStreamer runtime' }
$executablePath = (Resolve-Path -LiteralPath $Executable).Path
$templatePath = (Resolve-Path -LiteralPath $RobotTemplate).Path
$scenarioPath = (Resolve-Path -LiteralPath $Scenario).Path
$start = [Diagnostics.ProcessStartInfo]::new()
$start.FileName = $executablePath
$start.UseShellExecute = $false
$start.CreateNoWindow = $true
foreach ($argument in @($SimulatorHost, $BoardAddress, $templatePath, $scenarioPath, $output, "$Seconds", '--log-to-run')) {
    $start.ArgumentList.Add($argument)
}
if ($AssetDirectory) {
    $start.Environment['RAVEN_SIM_ASSETS'] = (Resolve-Path -LiteralPath $AssetDirectory).Path
}
if ($SerialPort) {
    $start.ArgumentList.Add('--serial-port')
    $start.ArgumentList.Add($SerialPort)
}
if ($GStreamerPython) {
    # The official wheel exposes its DLL and plugin environment to native apps.
    $environmentJson = & $GStreamerPython -c 'import json,os;from gstreamer_libs import gstreamer_env;e,_=gstreamer_env();print(json.dumps({k:v for k,v in e.items() if k.upper() in ("PATH","LD_LIBRARY_PATH","PYGI_DLL_DIRS","GI_TYPELIB_PATH","XDG_DATA_DIRS","XDG_CONFIG_DIRS") or k.startswith("GST_")}))'
    if ($LASTEXITCODE -ne 0) { throw 'Cannot obtain GStreamer runtime environment' }
    $environmentMap = $environmentJson | ConvertFrom-Json -AsHashtable
    foreach ($name in $environmentMap.Keys) { $start.Environment[$name] = [string]$environmentMap[$name] }
} elseif ($GStreamerRoot) {
    $runtime = (Resolve-Path -LiteralPath $GStreamerRoot).Path
    $start.Environment['PATH'] = (Join-Path $runtime 'bin') + ';' + $env:PATH
    $start.Environment['GST_PLUGIN_SYSTEM_PATH_1_0'] = Join-Path $runtime 'lib/gstreamer-1.0'
}
$process = [Diagnostics.Process]::Start($start)
@{process_id=$process.Id; executable=$executablePath; run_directory=$output} |
    ConvertTo-Json | Set-Content -LiteralPath $stateFile
if ($process.WaitForExit(1500)) { throw "Bridge exited ($($process.ExitCode)); inspect $output/bridge.log" }
Write-Output "Windows NVENC bridge process: $($process.Id)"
