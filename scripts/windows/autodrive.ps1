# AutoDRIVE simulator launcher for the Windows host (optional helper).
# Dot-source it in PowerShell, then run it:
#   . .\autodrive.ps1
#   autodrive -SimPath "<folder>\autodrive_simulator\AutoDRIVE Simulator.exe"
# From WSL: powershell.exe -NoProfile -ExecutionPolicy Bypass -Command ". '<path to this file>'; autodrive -SimPath '...'"
# (-ExecutionPolicy Bypass applies to that process only.)
# -SimPath may be omitted when $env:AUTODRIVE_SIM_EXE holds the path.
# -Ip defaults to 127.0.0.1, where scripts/autodrive_gui.sh publishes the
# devkit bridge; a stand-in recorder on the WSL host needs the WSL IP instead.
# After launch it clicks Menu -> Connection, then Camera Switch x5 (Driver's
# Eye -> Trackcam), then closes the menu. The build has no flag or setting for
# either (CLIManager reads only -ip/-port and connects on its own only under
# -batchmode; CameraSwitch cycles only on its button), so this clicks at
# window-relative positions taken from the scene's RectTransforms, verified at
# one 1920x991 window only. -NoConnect / -NoTrackcam skip a step;
# -LoadSeconds waits longer on a slow load. A second Connection click
# disconnects. docs/agents/autodrive.md has the facts.
function autodrive {
    param([string]$SimPath = $env:AUTODRIVE_SIM_EXE, [string]$Ip = "127.0.0.1",
          [int]$Port = 4567, [switch]$NoConnect, [switch]$NoTrackcam,
          [int]$LoadSeconds = 12)
    if (-not $SimPath -or -not (Test-Path -LiteralPath $SimPath)) {
        Write-Error "autodrive: pass -SimPath (the AutoDRIVE Simulator.exe) or set AUTODRIVE_SIM_EXE"; return
    }
    $p = Start-Process -FilePath $SimPath -ArgumentList "-ip", $Ip, "-port", $Port -PassThru
    if ($NoConnect -and $NoTrackcam) { return }
    if (-not ("AdNative.U32" -as [type])) {
        Add-Type -Namespace AdNative -Name U32 -MemberDefinition @'
[DllImport("user32.dll")] public static extern bool SetProcessDPIAware();
[DllImport("user32.dll")] public static extern bool SetForegroundWindow(IntPtr h);
[DllImport("user32.dll")] public static extern bool ShowWindow(IntPtr h, int c);
[DllImport("user32.dll")] public static extern bool GetClientRect(IntPtr h, out RECT r);
[DllImport("user32.dll")] public static extern bool ClientToScreen(IntPtr h, ref POINT p);
[DllImport("user32.dll")] public static extern bool SetCursorPos(int x, int y);
[DllImport("user32.dll")] public static extern void mouse_event(uint f, uint x, uint y, uint d, IntPtr e);
[DllImport("user32.dll")] public static extern void keybd_event(byte k, byte s, uint f, IntPtr e);
public struct RECT { public int Left, Top, Right, Bottom; }
public struct POINT { public int X, Y; }
'@
    }
    [void][AdNative.U32]::SetProcessDPIAware()
    $deadline = (Get-Date).AddSeconds(60)
    while ($p.MainWindowHandle -eq 0 -and (Get-Date) -lt $deadline) { Start-Sleep -Milliseconds 250; $p.Refresh() }
    $h = $p.MainWindowHandle
    if ($h -eq 0) { Write-Warning "autodrive: no sim window after 60 s; click Connect / Camera by hand"; return }
    Start-Sleep -Seconds $LoadSeconds
    $click = {
        param([double]$fx, [double]$fy, [int]$after = 600)  # client-area fractions from top-left; ms to wait after
        [AdNative.U32]::keybd_event(0x12, 0, 0, [IntPtr]::Zero); [AdNative.U32]::keybd_event(0x12, 0, 2, [IntPtr]::Zero)
        [void][AdNative.U32]::SetForegroundWindow($h)
        $r = New-Object AdNative.U32+RECT; [void][AdNative.U32]::GetClientRect($h, [ref]$r)
        $pt = New-Object AdNative.U32+POINT
        $pt.X = [int]($r.Right * $fx); $pt.Y = [int]($r.Bottom * $fy)
        [void][AdNative.U32]::ClientToScreen($h, [ref]$pt)
        [void][AdNative.U32]::SetCursorPos($pt.X, $pt.Y); Start-Sleep -Milliseconds 150
        [AdNative.U32]::mouse_event(2, 0, 0, 0, [IntPtr]::Zero); Start-Sleep -Milliseconds 60
        [AdNative.U32]::mouse_event(4, 0, 0, 0, [IntPtr]::Zero); Start-Sleep -Milliseconds $after
    }
    & $click 0.04 0.04 1000                                   # Toolbar > Menu (opens the panel)
    if (-not $NoConnect) { & $click 0.04 0.379 }              # Menu > Connection
    if (-not $NoTrackcam) { 1..5 | ForEach-Object { & $click 0.04 0.563 } }  # Menu > Camera Switch x5
    & $click 0.04 0.04                                        # close the panel
}
