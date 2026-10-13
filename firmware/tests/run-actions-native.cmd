@echo off
setlocal
cd /d "%~dp0..\..\.."
if errorlevel 1 exit /b 1
if not exist output\compatibility-20260908 mkdir output\compatibility-20260908
if not exist output\native-tests mkdir output\native-tests
call "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat" >nul
if errorlevel 1 exit /b 1
for %%S in (multi_action macro held_report) do (
  python scripts\prepare_%%S_native_test.py
  if errorlevel 1 exit /b 1
)
for %%T in (multi-action macro held-report) do (
  cl /nologo /EHsc /std:c++14 output\compatibility-20260908\%%T-native.cpp /Fooutput\native-tests\%%T.obj /Feoutput\native-tests\%%T.exe
  if errorlevel 1 exit /b 1
  output\native-tests\%%T.exe
  if errorlevel 1 exit /b 1
)
