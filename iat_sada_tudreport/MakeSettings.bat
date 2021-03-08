@echo off

@REM SET Make_EXE=prog\make.exe
SET Make_EXE=..\iat_sada_common\prog\make.exe
SET Make_FILE=..\iat_sada_common\prog\makefile

SET MAKE_CALL="%MAKE_EXE%" --makefile="%Make_FILE%"

SET NAME_DER_HAUPTDATEI_OHNE_ENDUNG=sada_tudreport