# SPDX-License-Identifier: AGPL-3.0-only
#Requires -RunAsAdministrator
[CmdletBinding()]
param(
    [Parameter(Mandatory)][System.Net.IPAddress]$WslAddress,
    [string]$DataDirectory = (Join-Path ([Environment]::GetFolderPath('LocalApplicationData')) 'raven-simulation')
)
$ErrorActionPreference = 'Stop'
if ($WslAddress.AddressFamily -ne [System.Net.Sockets.AddressFamily]::InterNetwork) { throw 'Use the WSL IPv4 address' }
$executable = Join-Path $DataDirectory 'Neighborhood/AirSimNH/Binaries/Win64/AirSimNH-Win64-Shipping.exe'
if (-not (Test-Path -LiteralPath $executable)) { throw 'Install the simulator first' }
foreach ($protocol in @('TCP', 'UDP')) {
    $name = "Raven-Simulation-WSL-$protocol"
    $existing = Get-NetFirewallRule -Name $name -ErrorAction SilentlyContinue
    if ($existing) { $existing | Remove-NetFirewallRule }
    $ports = if ($protocol -eq 'TCP') { @(4560, 8989, 8990) } else { @(14580) }
    New-NetFirewallRule -Name $name -DisplayName $name -Direction Inbound -Action Allow -Enabled True -Profile Any `
        -Program $executable -Protocol $protocol -LocalPort $ports -RemoteAddress $WslAddress.IPAddressToString | Out-Null
}
Write-Output 'Firewall access restricted to this executable and WSL IPv4 address.'
