@echo off
chcp 65001 >nul
setlocal

set "MODEL_DIR=%~dp0..\Codes\Touch_Client\models\cr3"
set "TEMP_DIR=%TEMP%\cr3_models_clone"

echo ================================================
echo   Fetch Dobot CR3 STL Models
echo ================================================
echo.
echo Target: %MODEL_DIR%
echo.

if exist "%MODEL_DIR%\base.stl" (
    echo Models already exist. Skipping fetch.
    goto :done
)

echo Cloning movensys_manipulator_description (sparse, models only)...
echo This may take a minute...

git clone --depth 1 --filter=blob:none --sparse ^
    https://github.com/movensys/movensys_manipulator_description.git "%TEMP_DIR%" 2>&1
if %ERRORLEVEL% NEQ 0 (
    echo.
    echo [WARNING] git clone failed. Falling back to geometric models.
    echo The application will use geometric primitives (cylinders + boxes) instead of STL meshes.
    goto :done
)

cd /d "%TEMP_DIR%"
git sparse-checkout set meshes/cr3a

if not exist "meshes\cr3a\*.stl" (
    echo [WARNING] No STL files found in expected path. Falling back to geometric models.
    cd /d "%~dp0"
    rmdir /s /q "%TEMP_DIR%" 2>nul
    goto :done
)

echo Copying STL files...
if not exist "%MODEL_DIR%" mkdir "%MODEL_DIR%"

:: CR3 的 STL 文件名可能不同，批量拷贝
copy /Y "meshes\cr3a\*.stl" "%MODEL_DIR%\" >nul

:: 如果文件命名不符合预期(base/link1~6)，列出可用文件供手动调整
echo.
echo Available STL files:
dir /b "%MODEL_DIR%"

cd /d "%~dp0"
rmdir /s /q "%TEMP_DIR%" 2>nul
echo.
echo Models fetched successfully.

:done
endlocal
