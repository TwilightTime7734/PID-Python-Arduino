param(
    [string]$PidToolboxRoot = "D:\Coding\Python\PIDtoolbox-master",
    [string]$LogDir = "D:\Coding\Python\Modbus\blackbox_imports",
    [string]$MatlabExe = ""
)

$ErrorActionPreference = "Stop"

function Resolve-MatlabExe {
    param([string]$ExplicitPath)

    if ($ExplicitPath -and (Test-Path -LiteralPath $ExplicitPath)) {
        return (Resolve-Path -LiteralPath $ExplicitPath).Path
    }

    $cmd = Get-Command matlab -ErrorAction SilentlyContinue
    if ($cmd -and $cmd.Source -and (Test-Path -LiteralPath $cmd.Source)) {
        return (Resolve-Path -LiteralPath $cmd.Source).Path
    }

    $candidates = @(
        "C:\Program Files\MATLAB\R2026a\bin\matlab.exe",
        "C:\Program Files\MATLAB\R2025b\bin\matlab.exe",
        "C:\Program Files\MATLAB\R2025a\bin\matlab.exe",
        "C:\Program Files\MATLAB\R2024b\bin\matlab.exe",
        "C:\Program Files\MATLAB\R2024a\bin\matlab.exe",
        "C:\Program Files\MATLAB\R2023b\bin\matlab.exe",
        "C:\Program Files\MATLAB\R2023a\bin\matlab.exe",
        "C:\Program Files\MATLAB\R2022b\bin\matlab.exe",
        "C:\Program Files\MATLAB\R2022a\bin\matlab.exe",
        "C:\Program Files\MATLAB\R2021b\bin\matlab.exe",
        "C:\Program Files\MATLAB\R2021a\bin\matlab.exe",
        "C:\Program Files\MATLAB\R2020b\bin\matlab.exe",
        "C:\Program Files\MATLAB\R2020a\bin\matlab.exe"
    )

    foreach ($candidate in $candidates) {
        if (Test-Path -LiteralPath $candidate) {
            return (Resolve-Path -LiteralPath $candidate).Path
        }
    }

    return $null
}

if (-not (Test-Path -LiteralPath $PidToolboxRoot)) {
    throw "PIDtoolbox folder not found: $PidToolboxRoot"
}

$resolvedRoot = (Resolve-Path -LiteralPath $PidToolboxRoot).Path
$resolvedLogDir = if (Test-Path -LiteralPath $LogDir) {
    (Resolve-Path -LiteralPath $LogDir).Path
} else {
    $LogDir
}

# PTB reads this file to decide the default location for file selection.
$logfileDirTxt = Join-Path $resolvedRoot "logfileDir.txt"
"logfileDirectory: $resolvedLogDir" | Set-Content -Path $logfileDirTxt -Encoding ascii

$matlabPath = Resolve-MatlabExe -ExplicitPath $MatlabExe
if ($matlabPath) {
    $startupCode = @(
        "cd('$($resolvedRoot -replace '\\','\\')');",
        "PIDtoolbox"
    ) -join " "

    Write-Output "Launching PIDtoolbox source with MATLAB: $matlabPath"
    & $matlabPath -nosplash -nodesktop -r $startupCode
    exit $LASTEXITCODE
}

$standaloneExe = Get-ChildItem -Path $resolvedRoot -Recurse -File -Filter "PIDtoolbox.exe" -ErrorAction SilentlyContinue |
    Select-Object -First 1
if ($standaloneExe) {
    Write-Output "Launching standalone PIDtoolbox executable: $($standaloneExe.FullName)"
    & $standaloneExe.FullName
    exit $LASTEXITCODE
}

Write-Output ""
Write-Output "No runnable PIDtoolbox target found."
Write-Output "Detected only MATLAB Runtime (e.g. C:\Program Files\MATLAB\MATLAB Runtime\v95),"
Write-Output "which cannot execute .m source files by itself."
Write-Output ""
Write-Output "Options:"
Write-Output "1) Install full MATLAB (with toolboxes) and rerun this script."
Write-Output "2) Obtain a standalone PIDtoolbox build (PIDtoolbox.exe) and place it under:"
Write-Output "   $resolvedRoot"
Write-Output ""
Write-Output "When ready, rerun:"
Write-Output "  powershell -ExecutionPolicy Bypass -File tools\launch_pidtoolbox.ps1"
exit 1
