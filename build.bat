@echo off
REM Builds bc1_example.c against the Thorlabs TLBC1 VISA driver.
REM The TLBC1 driver DLL is 32-bit only (TLBC1_32.dll), so this must
REM produce a 32-bit (x86) executable.

setlocal

set "VISA_INC=C:\Program Files (x86)\IVI Foundation\VISA\WinNT\Include"
set "VISA_LIB=C:\Program Files (x86)\IVI Foundation\VISA\WinNT\lib\msc"

where cl.exe >nul 2>nul
if errorlevel 1 goto :nocl
goto :build

:nocl
echo cl.exe not found on PATH.
echo Run this from a "Developer Command Prompt for VS" - x86 - first, for example:
echo   "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars32.bat"
exit /b 1

:build

cl.exe /nologo /W3 /I "%VISA_INC%" bc1_example.c /link /LIBPATH:"%VISA_LIB%" visa32.lib TLBC1_32.lib /OUT:bc1_example.exe

endlocal
