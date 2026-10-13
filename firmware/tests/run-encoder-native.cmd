@echo off
setlocal
cd /d "%~dp0..\..\.."
if errorlevel 1 exit /b 1
if not exist output\native-tests mkdir output\native-tests
if errorlevel 1 exit /b 1
call "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat" >nul
if errorlevel 1 exit /b 1
python scripts\prepare_encoder_power_test.py
if errorlevel 1 exit /b 1
cl /nologo /EHsc /std:c++14 /I firmware\upstream\include firmware\upstream\tests\encoder_mode_gesture_native.cpp /Fooutput\native-tests\encoder.obj /Feoutput\native-tests\encoder.exe
if errorlevel 1 exit /b 1
output\native-tests\encoder.exe
if errorlevel 1 exit /b 1
cl /nologo /EHsc /std:c++14 /I firmware\upstream\include output\compatibility-20260909\encoder-power-native.cpp /Fooutput\native-tests\encoder-power.obj /Feoutput\native-tests\encoder-power.exe
if errorlevel 1 exit /b 1
output\native-tests\encoder-power.exe
if errorlevel 1 exit /b 1
echo Encoder gesture and power integration tests passed.
python scripts\prepare_layer_cycle_test.py
if errorlevel 1 exit /b 1
cl /nologo /EHsc /std:c++14 output\native-tests\layer-cycle.cpp /Fooutput\native-tests\layer-cycle.obj /Feoutput\native-tests\layer-cycle.exe
if errorlevel 1 exit /b 1
output\native-tests\layer-cycle.exe
if errorlevel 1 exit /b 1
echo Layer cycle routing tests passed.
