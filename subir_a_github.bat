@echo off
chcp 65001 >nul
cd /d "%~dp0"
set "GH_EXE=C:\Program Files\GitHub CLI\gh.exe"

echo ========================================================
echo   SUBIDA AUTOMATICA A GITHUB — STRUCTURA 3D
echo ========================================================

"%GH_EXE%" auth status >nul 2>&1
if %ERRORLEVEL% NEQ 0 (
    echo [1/2] Iniciando sesion en GitHub mediante navegador...
    "%GH_EXE%" auth login --web -h github.com -p https
)

echo [2/2] Creando repositorio 'structura-3d' en GitHub y subiendo la rama 'main'...
"%GH_EXE%" repo create structura-3d --public --source=. --remote=origin --push
if %ERRORLEVEL% EQU 0 (
    echo.
    echo ¡Proyecto subido exitosamente a GitHub!
) else (
    echo.
    echo Si el repositorio ya existe, haciendo git push...
    git push -u origin main
)
pause
