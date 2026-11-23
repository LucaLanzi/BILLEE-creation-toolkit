@echo off
rem Double-click to install Rover and Arm Builder on Windows.
rem To uninstall, run in Command Prompt: Install-Windows.bat uninstall
setlocal EnableExtensions EnableDelayedExpansion

set "PACKAGE=user_default.rover_arm_builder"
set "MODULE=bl_ext.user_default.rover_arm_builder"
set /a MIN_SCORE=5003
set "HERE=%~dp0"
set "ACTION=install"
if /i "%~1"=="uninstall" set "ACTION=uninstall"
set "BEST="
set "BEST_VERSION="
set /a BEST_SCORE=-1
set "VERSION_FILE=%TEMP%\rab_blender_version_%RANDOM%.txt"

echo Rover ^& Arm Builder
echo Looking for Blender 5.3 or newer...

if defined BLENDER (
    call :consider "%BLENDER%"
    if not defined BEST (
        set "MESSAGE=BLENDER is set, but it does not point to a working Blender."
        goto :fail
    )
    goto :found
)

for /f "delims=" %%B in ('where blender.exe 2^>nul') do call :consider "%%B"
for /d %%D in ("%ProgramFiles%\Blender Foundation\Blender *") do call :consider "%%D\blender.exe"
if defined ProgramW6432 for /d %%D in ("%ProgramW6432%\Blender Foundation\Blender *") do call :consider "%%D\blender.exe"
call :consider "%ProgramFiles(x86)%\Steam\steamapps\common\Blender\blender.exe"
call :consider "%ProgramFiles%\Steam\steamapps\common\Blender\blender.exe"
call :consider "%LOCALAPPDATA%\Microsoft\WindowsApps\blender.exe"

:found
if exist "%VERSION_FILE%" del "%VERSION_FILE%" >nul 2>&1
if not defined BEST (
    set "MESSAGE=Blender was not found. Install it from https://www.blender.org/download/ and run this again."
    goto :fail
)
if !BEST_SCORE! LSS %MIN_SCORE% (
    set "MESSAGE=Found Blender !BEST_VERSION!, but 5.3 or newer is required."
    goto :fail
)
echo Using Blender !BEST_VERSION! (!BEST!)

if /i "%ACTION%"=="uninstall" goto :uninstall

set "ZIP="
for %%F in ("%HERE%rover_arm_builder-*.zip") do set "ZIP=%%~fF"
if not defined ZIP for %%F in ("%HERE%..\dist\rover_arm_builder-*.zip") do set "ZIP=%%~fF"
if not defined ZIP (
    set "MESSAGE=Could not find rover_arm_builder-*.zip next to this installer."
    goto :fail
)

echo Installing !ZIP!...
"!BEST!" --command extension install-file -r user_default -e "!ZIP!"
if errorlevel 1 (
    set "MESSAGE=Blender could not install the extension."
    goto :fail
)
"!BEST!" --background --python-exit-code 1 --python-expr "import bpy, sys; sys.exit(0 if '%MODULE%' in bpy.context.preferences.addons else 1)" >nul 2>&1
if errorlevel 1 (
    set "MESSAGE=The extension installed but is not enabled. Enable it in Edit > Preferences > Add-ons."
    goto :fail
)
echo.
echo Installed and enabled.
echo Open Blender, press N in the 3D Viewport, and choose the Robot tab.
goto :done

:uninstall
"!BEST!" --command extension remove %PACKAGE%
if errorlevel 1 (
    set "MESSAGE=Blender could not remove the extension."
    goto :fail
)
echo Removed Rover ^& Arm Builder.
goto :done

:consider
if "%~1"=="" exit /b 0
if not exist "%~1" exit /b 0
"%~1" --version > "%VERSION_FILE%" 2>nul
set "FOUND="
for /f "usebackq tokens=1,2" %%A in ("%VERSION_FILE%") do (
    if /i "%%A"=="Blender" if not defined FOUND set "FOUND=%%B"
)
if not defined FOUND exit /b 0
for /f "tokens=1,2 delims=." %%M in ("!FOUND!") do set /a SCORE=%%M*1000+%%N
if !SCORE! GTR !BEST_SCORE! (
    set /a BEST_SCORE=!SCORE!
    set "BEST=%~1"
    set "BEST_VERSION=!FOUND!"
)
exit /b 0

:fail
echo.
echo ERROR: !MESSAGE!
if "%~1"=="" if not defined RAB_NO_PAUSE pause
exit /b 1

:done
if "%~1"=="" if not defined RAB_NO_PAUSE pause
exit /b 0
