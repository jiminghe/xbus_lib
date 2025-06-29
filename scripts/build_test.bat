@echo off
echo Building XBus Parser Tests...

cd /d "%~dp0\..\test"

if not exist build mkdir build
cd build

cmake .. -G "Visual Studio 16 2019" -A x64
if %errorlevel% neq 0 (
    echo CMake configuration failed!
    pause
    exit /b %errorlevel%
)

cmake --build . --config Release
if %errorlevel% neq 0 (
    echo Build failed!
    pause
    exit /b %errorlevel%
)

echo Build completed successfully!
echo Running tests...
echo.

Release\xbus_parser_test.exe

pause