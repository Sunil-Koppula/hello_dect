$venvPath = "c:/ncs/.venv3.2.4"
$activateScript = "$venvPath/Scripts/Activate.ps1"
$CurrentDirectory = (Get-Location).Path.Replace('\', '/')
$BuildDir = "$CurrentDirectory/build"

# Activate virtual environment if not already active
if (-not $env:VIRTUAL_ENV) {
    Write-Output "Activating virtual environment..."
    & $activateScript
} else {
    Write-Output "Virtual environment is already activated."
}

# Set environment variables
$env:NCS_TOOLCHAIN_VERSION = "v3.2.4"
$env:PATH = "c:/ncs/toolchains/fd21892d0f/opt/bin;" + $env:PATH
$env:ZEPHYR_BASE = "c:/ncs/v3.2.4/zephyr"

# Clear CONF_FILE/BOARD_ROOT so they don't leak into MCUboot child build
Remove-Item Env:\CONF_FILE -ErrorAction SilentlyContinue
Remove-Item Env:\BOARD_ROOT -ErrorAction SilentlyContinue

# Check if build directory exists for incremental build
if (Test-Path "$BuildDir/build.ninja") {
    Write-Output "Incremental build (build directory exists)..."
    west build -d $BuildDir
} else {
    Write-Output "Full build (no existing build directory)..."
    west build -d $BuildDir --board nrf9151dk/nrf9151/ns --sysbuild $CurrentDirectory
}

# Return to original directory
Set-Location -Path $CurrentDirectory
