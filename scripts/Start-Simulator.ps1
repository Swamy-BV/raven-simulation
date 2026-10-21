# SPDX-License-Identifier: AGPL-3.0-only
[CmdletBinding()]
param(
    [string]$DataDirectory = (Join-Path ([Environment]::GetFolderPath('LocalApplicationData')) 'raven-simulation'),
    [int]$GraphicsAdapter = 0,
    [switch]$Visible,
    [switch]$Stop
)
$ErrorActionPreference = 'Stop'
function Enable-SimulatorTimerRequests([int]$SimulatorProcessId) {
    # Windows 11 may ignore timer requests for hidden windows. HITL physics
    # must retain its requested 4 ms period independently of window visibility.
    if ([Environment]::OSVersion.Version.Build -lt 22000) { return }
    if (-not ('RavenSimulation.TimerPolicy' -as [type])) {
        Add-Type -TypeDefinition @'
using System;
using System.ComponentModel;
using System.Runtime.InteropServices;
namespace RavenSimulation {
    public static class TimerPolicy {
        [StructLayout(LayoutKind.Sequential)]
        private struct State {
            public uint Version, ControlMask, StateMask;
        }
        [DllImport("kernel32.dll", SetLastError = true)]
        private static extern IntPtr OpenProcess(uint access, bool inherit, int processId);
        [DllImport("kernel32.dll", SetLastError = true)]
        private static extern bool SetProcessInformation(IntPtr process, int kind, ref State state, uint size);
        [DllImport("kernel32.dll")]
        private static extern bool CloseHandle(IntPtr process);
        public static void Enable(int processId) {
            const uint ProcessSetInformation = 0x0200;
            const int ProcessPowerThrottling = 4;
            const uint IgnoreTimerResolution = 4;
            IntPtr handle = OpenProcess(ProcessSetInformation, false, processId);
            if (handle == IntPtr.Zero) { throw new Win32Exception(); }
            try {
                State state = new State { Version = 1, ControlMask = IgnoreTimerResolution, StateMask = 0 };
                uint size = (uint)Marshal.SizeOf<State>();
                if (!SetProcessInformation(handle, ProcessPowerThrottling, ref state, size)) {
                    throw new Win32Exception();
                }
            } finally { CloseHandle(handle); }
        }
    }
}
'@
    }
    [RavenSimulation.TimerPolicy]::Enable($SimulatorProcessId)
}
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
    try {
        Enable-SimulatorTimerRequests $process.Id
    } catch {
        Stop-Process -Id $process.Id -ErrorAction SilentlyContinue
        throw
    }
    @{process_id = $process.Id; executable = $executable} | ConvertTo-Json | Set-Content -LiteralPath $stateFile
    Write-Output "Simulator process: $($process.Id)"
} finally { $env:PATH = $previousPath }
