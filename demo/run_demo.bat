@echo off
setlocal
echo ======================================================================
echo   VIRTUAL MULTI-AXIS PAPER CUTTER DEMO LAUNCHER
echo ======================================================================
echo.

set PYTHON_EXE=python
where python >nul 2>&1
if %ERRORLEVEL% NEQ 0 (
    if exist "%LOCALAPPDATA%\Programs\Python\Python311\python.exe" (
        set PYTHON_EXE=%LOCALAPPDATA%\Programs\Python\Python311\python.exe
    ) else (
        echo [ERROR] Python not found on PATH.
        exit /b 1
    )
)

echo Using Python: %PYTHON_EXE%
%PYTHON_EXE% "%~dp0demo.py" %*
echo.
pause
