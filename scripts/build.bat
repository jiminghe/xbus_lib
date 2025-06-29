@echo off
echo ======================================
echo Building Xbus Serial Reader Project
echo ======================================

:: Get the directory where this script is located
set SCRIPT_DIR=%~dp0
:: Go to parent directory (project root)
cd /d "%SCRIPT_DIR%.."

:: Create build directory if it doesn't exist
if not exist "build" (
    echo Creating build directory...
    mkdir build
)

:: Navigate to build directory
cd build

:: Run CMake configuration
echo.
echo Configuring CMake...
cmake ..
if %errorlevel% neq 0 (
    echo CMake configuration failed!
    pause
    exit /b 1
)

:: Build the project
echo.
echo Building project...
cmake --build . --config Release
if %errorlevel% neq 0 (
    echo Build failed!
    pause
    exit /b 1
)

echo.
echo ======================================
echo Build completed successfully!
echo ======================================
echo Executable location: build\bin\Release\xbus_reader.exe
echo.
echo Press any key to exit...
pause > nul