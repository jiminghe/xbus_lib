@echo off
echo Cleaning XBus Parser Test Build...

cd /d "%~dp0\..\test"

if exist build (
    echo Removing build directory...
    rmdir /s /q build
    if %errorlevel% neq 0 (
        echo Failed to remove build directory!
        pause
        exit /b %errorlevel%
    )
    echo Build directory removed successfully!
) else (
    echo Build directory does not exist, nothing to clean.
)

echo Clean completed!
pause