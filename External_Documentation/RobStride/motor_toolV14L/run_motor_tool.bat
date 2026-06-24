@echo off
setlocal
cd /d "%~dp0"

echo Motor Studio launcher (cwd=%CD%)
echo.

rem Unblock DLLs/exe downloaded from the internet (common Windows issue)
powershell -NoProfile -Command "Get-ChildItem -LiteralPath '%CD%' -Recurse | Unblock-File -ErrorAction SilentlyContinue" >nul 2>&1

set "QT_PLUGIN_PATH=%CD%"
set "PATH=%CD%;%PATH%"

if not exist "motor_tool.exe" (
  echo ERROR: motor_tool.exe not found in this folder.
  echo Run this .bat from the motor_toolV14L directory, not a shortcut to the exe alone.
  pause
  exit /b 1
)
if not exist "platforms\qwindows.dll" (
  echo ERROR: missing platforms\qwindows.dll — incomplete extract?
  pause
  exit /b 1
)
if not exist "Qt5Core.dll" (
  echo ERROR: missing Qt5Core.dll — copy the full motor_toolV14L folder, not just the exe.
  pause
  exit /b 1
)

start "" /wait "motor_tool.exe"
set ERR=%ERRORLEVEL%
if not "%ERR%"=="0" (
  echo.
  echo motor_tool.exe exited with code %ERR%.
  echo If you saw "side-by-side" or missing DLL errors, see README-windows.txt
  pause
)
exit /b %ERR%
