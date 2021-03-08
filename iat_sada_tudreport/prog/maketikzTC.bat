@echo off
@REM Diese Datei existiert aus Kompatibilitätsgründen und ruft nur die maketiz.bat mit den gleichen
@REM Argumenten auf, mit der es selbst aufgerufen wurde
@REM Die anderen .bat Dateien befinden sich im gleichen Verzeichnis wie die maketikzTC.bat
set "TikZPath=%~dp0..\..\iat_sada_common\texmf\scripts\iatsada\"
@REM maketikz aufrufen
call "%TikZPath%\maketikz.bat" %1 %2 %3 %4 %5   -s --preview
