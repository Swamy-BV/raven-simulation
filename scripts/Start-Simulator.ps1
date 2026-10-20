# SPDX-License-Identifier: AGPL-3.0-only
[CmdletBinding()]
param(
    [string]$DataDirectory = (Join-Path ([Environment]::GetFolderPath('LocalApplicationData')) 'raven-simulation'),
    [int]$GraphicsAdapter = 0,
    [switch]$Visible,
    [switch]$Stop
)
$ErrorActionPreference = 'Stop'
$base = Join-Path $DataDirectory 'Neighborhood'
$executable = Join-Path $base 'AirSimNH/Binaries/Win64/AirSimNH-Win64-Shipping.exe'
$stateFile = Join-Path $DataDirectory 'server.json'
if ($Stop) {
    if (Test-Path -LiteralPath $stateFile) {
        $saved = Get-Content -LiteralPath $stateFile -Raw | ConvertFrom-Json
        $running = Get-Process -Id $saved.process_id -ErrorAction SilentlyContinue
        if ($running -and $running.Path -eq $executable) { Stop-Process -Id $running.Id }
    }
    return
}
if (-not (Test-Path -LiteralPath $executable)) { throw 'Run Install-Windows.ps1 first' }
if (Test-Path -LiteralPath $stateFile) {
    $saved = Get-Content -LiteralPath $stateFile -Raw | ConvertFrom-Json
    $running = Get-Process -Id $saved.process_id -ErrorAction SilentlyContinue
    if ($running -and $running.Path -eq $executable) { throw 'Simulator is already running' }
}
$previousPath = $env:PATH
try {
    # The MSIX DirectX package supplies the legacy XInput DLL without modifying system directories.
    $runtime = Get-AppxPackage 'Microsoft.DirectXRuntime' | Select-Object -First 1
    if ($runtime) { $env:PATH = $runtime.InstallLocation + ';' + $previousPath }
    $arguments = @('-ResX=960', '-ResY=540', '-nosound', '-unattended', '-dx11', "-graphicsadapter=$GraphicsAdapter")
    $windowStyle = 'Hidden'
    if ($Visible) { $windowStyle = 'Normal' } else { $arguments += '-RenderOffScreen' }
    $process = Start-Process -FilePath $executable -ArgumentList $arguments -WorkingDirectory $base -WindowStyle $windowStyle -PassThru
    @{process_id = $process.Id; executable = $executable} | ConvertTo-Json | Set-Content -LiteralPath $stateFile
    Write-Output "Simulator process: $($process.Id)"
} finally { $env:PATH = $previousPath }
