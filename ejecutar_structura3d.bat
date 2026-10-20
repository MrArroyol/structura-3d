@echo off
title Structura 3D - Entorno Interactivo en Godot 4
echo ============================================================================
echo   STRUCTURA 3D - Iniciando Motor de Calculo y Visor Interactivo 3D (Godot 4)
echo   AVISO: Modelo elastico lineal idealizado. No apto para construccion.
echo ============================================================================

py -m solver.cli --run-all-benchmarks

for /f "delims=" %%F in ('powershell -NoProfile -Command "(Get-ChildItem -Path \"$env:LOCALAPPDATA\Microsoft\WinGet\Packages\GodotEngine.GodotEngine*\" -Filter \"Godot*_win64.exe\" -Recurse | Select-Object -First 1).FullName"') do set "GODOT_EXE=%%F"

if defined GODOT_EXE (
    echo Abriendo Godot 4: "%GODOT_EXE%"
    start "" "%GODOT_EXE%" --path "%~dp0"
) else (
    echo No se encontro el binario de Godot 4 en WinGet Packages.
    pause
)
