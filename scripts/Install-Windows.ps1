# SPDX-License-Identifier: AGPL-3.0-only
[CmdletBinding()]
param(
    [string]$DataDirectory = (Join-Path ([Environment]::GetFolderPath('LocalApplicationData')) 'raven-simulation'),
    [string]$Archive,
    [switch]$Prerequisites
)
$ErrorActionPreference = 'Stop'
$versions = @{}
Get-Content (Join-Path $PSScriptRoot '../versions.env') | ForEach-Object {
    if ($_ -match '^([A-Z0-9_]+)=(.+)$') { $versions[$Matches[1]] = $Matches[2] }
}
if ($Prerequisites) {
    winget install --id Microsoft.VCRedist.2015+.x64 --exact --accept-package-agreements --accept-source-agreements
    if ($LASTEXITCODE -ne 0 -and $LASTEXITCODE -ne -1978335189) { throw 'Visual C++ runtime installation failed' }
    winget install --id Microsoft.DirectX --exact --accept-package-agreements --accept-source-agreements
    if ($LASTEXITCODE -ne 0 -and $LASTEXITCODE -ne -1978335189) { throw 'DirectX runtime installation failed' }
}
New-Item -ItemType Directory -Path $DataDirectory -Force | Out-Null
if (-not $Archive) {
    $Archive = Join-Path $DataDirectory "Neighborhood-Windows-$($versions.PROJECTAIRSIM_VERSION).zip"
}
if (-not (Test-Path -LiteralPath $Archive)) {
    $url = "https://github.com/iamaisim/ProjectAirSim/releases/download/v$($versions.PROJECTAIRSIM_VERSION)/Neighborhood-Windows-$($versions.PROJECTAIRSIM_VERSION).zip"
    Invoke-WebRequest -Uri $url -OutFile $Archive
}
$actual = (Get-FileHash -LiteralPath $Archive -Algorithm SHA256).Hash.ToLowerInvariant()
if ($actual -ne $versions.NEIGHBORHOOD_SHA256) { throw 'Simulator archive checksum mismatch' }
$destination = Join-Path $DataDirectory 'Neighborhood'
$executable = Join-Path $destination 'AirSimNH/Binaries/Win64/AirSimNH-Win64-Shipping.exe'
if (-not (Test-Path -LiteralPath $executable)) {
    Expand-Archive -LiteralPath $Archive -DestinationPath $destination
}
if (-not (Test-Path -LiteralPath $executable)) { throw 'Simulator executable not found after extraction' }
Write-Output "Verified simulator: $executable"

$assetDirectory = Join-Path $DataDirectory 'assets'
New-Item -ItemType Directory -Path $assetDirectory -Force | Out-Null
$manifest = Get-Content (Join-Path $PSScriptRoot '../scene-assets.json') -Raw | ConvertFrom-Json
foreach ($asset in $manifest.assets) {
    if ([IO.Path]::GetFileName($asset.file) -ne $asset.file) { throw 'Invalid scene asset filename' }
    $file = Join-Path $assetDirectory $asset.file
    if (-not (Test-Path -LiteralPath $file)) { Invoke-WebRequest -Uri $asset.url -OutFile $file }
    if ((Get-FileHash -LiteralPath $file -Algorithm SHA256).Hash.ToLowerInvariant() -ne $asset.sha256) {
        throw "Scene asset checksum mismatch: $($asset.file)"
    }
}
Write-Output "Verified scene textures: $assetDirectory"
