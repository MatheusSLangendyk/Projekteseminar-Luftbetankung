@echo off
@REM Argumente: Bei -nopause wird nie angehalten
@REM 
SET THISDIR=%~dp0
SET WORKDIR=%CD%
@REM ============ Anfang anzupassende Werte ==================
@REM Das Verzeichnis in dem sich die pdflatex.exe befindet:
SET MIKTEX_DIR=C:\Program Files\MiKTeX 2.9\miktex\bin\x64\
@REM Der Name des Dokumentes ohne .tex Endung
SET MAIN_TEX_FILE=%THISDIR%\sada_tudreport
@REM ============ Ende anzupassende Werte ====================

SET NOPAUSE=0
if "%~1" == "-nopause" SET NOPAUSE=1


cd /D "%THISDIR%" || goto :error

SET PDFLATEX_BIN=%MIKTEX_DIR%\pdflatex.exe
SET BIBER_BIN=%MIKTEX_DIR%\biber.exe

@REM Maximalwerte:
@REM getestete Maximalwerte (bei größeren Werten funktioniert pdflatex nicht mehr):
@REM --save-size=80000
@REM --stack-size=65535
@REM --main-memory=256000000  
@REM --extra-mem-top=2147483647
@REM --extra-mem-bot=2147483647
SET TEXEXTRAARGS=--save-size=80000 --main-memory=256000000  --stack-size=65535 --extra-mem-top=2147483647 --extra-mem-bot=2147483647

SET PDFLATEX_ARGS=-shell-escape -interaction=nonstopmode -synctex=-1 %TEXEXTRAARGS% --halt-on-error "%MAIN_TEX_FILE%.tex"
SET BIBER_ARGS="%MAIN_TEX_FILE%"

"%PDFLATEX_BIN%" %PDFLATEX_ARGS% || goto :error
"%BIBER_BIN%" %BIBER_ARGS%  || goto :error
"%PDFLATEX_BIN%" %PDFLATEX_ARGS% || goto :error
"%BIBER_BIN%" %BIBER_ARGS% || goto :error
"%PDFLATEX_BIN%" %PDFLATEX_ARGS% || goto :error
"%BIBER_BIN%" %BIBER_ARGS% || goto :error

cd /D %WORKDIR%
goto :EOF
:ERROR
	SET Fehler=%errorlevel%
	cd /D %WORKDIR%
	echo/
	echo Failed with error #%Fehler%
	if "%NOPAUSE%" == "0" pause
	exit /B %Fehler%

:EOF