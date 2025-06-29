@echo off
echo ======================================
echo Cleaning Xbus Serial Reader Project
echo ======================================

:: Get the directory where this script is located
set SCRIPT_DIR=%~dp0
:: Go to parent directory (project root)
cd /d "%SCRIPT_DIR%.."

:: Check if build directory exists
if not exist "build" (
    echo Build directory does not exist. Nothing to clean.
    echo.
    echo Press any key to exit...
    pause > nul
    exit /b 0
)

:: Confirm deletion
echo This will delete the entire "build" directory and all its contents.
set /p confirm="Are you sure you want to continue? (y/N): "
if /i not "%confirm%"=="y" (
    echo Operation cancelled.
    echo.
    echo Press any key to exit...
    pause > nul
    exit /b 0
)

:: Remove build directory
echo.
echo Removing build directory...
rmdir /s /q "build"
if %errorlevel% neq 0 (
    echo Failed to remove build directory!
    echo Please make sure no files are in use and try again.
    pause
    exit /b 1
)

:: Also remove any executable that might have been copied to root
if exist "xbus_reader.exe" (
    echo Removing executable from root directory...
    del "xbus_reader.exe"
)

echo.
echo ======================================
echo Clean completed successfully!
echo ======================================
echo All build files have been removed.
echo.
echo Press any key to exit...
pause > nul