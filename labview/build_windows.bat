@echo off
rem ---------------------------------------------------------------------------
rem  Build windemu_64.dll / windemu_32.dll with Microsoft Visual C++
rem  Run from "x64 Native Tools Command Prompt for VS"  -> builds windemu_64.dll
rem  Run from "x86 Native Tools Command Prompt for VS"  -> builds windemu_32.dll
rem ---------------------------------------------------------------------------
cd /d %~dp0
if not exist bin mkdir bin
set BITS=64
if /i "%VSCMD_ARG_TGT_ARCH%"=="x86" set BITS=32
cl /nologo /O2 /W3 /LD /MT /D_CRT_SECURE_NO_WARNINGS src\windemu.c ^
   /Fe:bin\windemu_%BITS%.dll /Fo:bin\ ws2_32.lib winmm.lib || exit /b 1
del bin\*.obj bin\*.exp 2>nul
echo Built bin\windemu_%BITS%.dll
