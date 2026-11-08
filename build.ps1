# CMake Build Script for Baremetal ECG
# Windows PowerShell Version

param(
    [ValidateSet("Debug", "Release")]
    [string]$BuildType = "Release"
)

Write-Host "=== Building Baremetal ECG ===" -ForegroundColor Cyan

# Configuration
$BuildDir = "cmake_build"
$ToolchainFile = "cmake/arm-toolchain.cmake"

# Create build directory
if (-not (Test-Path $BuildDir)) {
    New-Item -ItemType Directory -Path $BuildDir | Out-Null
}

# Configure with CMake
Write-Host "Configuring CMake..." -ForegroundColor Yellow
Push-Location $BuildDir

cmake .. `
    -DCMAKE_TOOLCHAIN_FILE="$ToolchainFile" `
    -DCMAKE_BUILD_TYPE=$BuildType `
    -G "Ninja" `
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

if ($LASTEXITCODE -ne 0) {
    Write-Error "CMake configuration failed!"
    exit 1
}

# Build
Write-Host "Building project..." -ForegroundColor Yellow
cmake --build . --config $BuildType --parallel

if ($LASTEXITCODE -ne 0) {
    Write-Error "Build failed!"
    exit 1
}

Pop-Location

Write-Host "=== Build Complete ===" -ForegroundColor Green
Write-Host "Output files:" -ForegroundColor Cyan
Write-Host "  - ELF: $BuildDir/Baremetal_ECG.elf"
Write-Host "  - HEX: $BuildDir/Baremetal_ECG.hex"
Write-Host "  - BIN: $BuildDir/Baremetal_ECG.bin"
