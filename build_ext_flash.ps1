# Usage: ./build_ext_flash.ps1 (NVS on external flash — testing without EEPROM)

$venvPath = "c:/ncs/.venv3.2.4"
$activateScript = "$venvPath/Scripts/Activate.ps1"
$CurrentDirectory = (Get-Location).Path.Replace('\', '/')
$BuildDir = "$CurrentDirectory/build"
$NvsConf = "$CurrentDirectory/src/testing/overlay-nvs.conf"
$NvsOverlay = "$CurrentDirectory/src/testing/overlay-nvs.overlay"

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

Write-Output "Storage backend: NVS on external flash (testing)"

# Check if build directory exists for incremental build
if (Test-Path "$BuildDir/build.ninja") {
    Write-Output "Incremental build..."
    west build -d $BuildDir -- "-DEXTRA_CONF_FILE=$NvsConf" "-DEXTRA_DTC_OVERLAY_FILE=$NvsOverlay"
} else {
    Write-Output "Full build..."
    west build -d $BuildDir --board nrf9151dk/nrf9151/ns --sysbuild $CurrentDirectory -- "-DEXTRA_CONF_FILE=$NvsConf" "-DEXTRA_DTC_OVERLAY_FILE=$NvsOverlay"
}

# Return to original directory
Set-Location -Path $CurrentDirectory
