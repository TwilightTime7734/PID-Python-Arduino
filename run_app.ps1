param(
    [switch]$CheckVenv,

    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]]$AppArgs
)

$ErrorActionPreference = "Stop"

$RepoRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$VenvRoot = Join-Path $RepoRoot ".venv"
$VenvScripts = Join-Path $VenvRoot "Scripts"
$PythonExe = Join-Path $VenvScripts "python.exe"
$MainPy = Join-Path $RepoRoot "main.py"

if (-not (Test-Path -LiteralPath $PythonExe)) {
    throw "Missing venv Python: $PythonExe"
}

$env:VIRTUAL_ENV = $VenvRoot
$env:PATH = "$VenvScripts;$env:PATH"
$env:PYTHONNOUSERSITE = "1"

Set-Location -LiteralPath $RepoRoot
if ($CheckVenv) {
    & $PythonExe -c "import sys, importlib.util; print(sys.executable); print(importlib.util.find_spec('plotly').origin if importlib.util.find_spec('plotly') else 'plotly_missing')"
    exit $LASTEXITCODE
}

& $PythonExe $MainPy @AppArgs
exit $LASTEXITCODE
