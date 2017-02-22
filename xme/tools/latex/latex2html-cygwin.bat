@echo OFF

SET CYGWIN=C:\cygwin
SET LATEX2HTML=%CYGWIN%\bin\perl.exe '/usr/local/bin/latex2html' -split 0 -nonavigation -noinfo
SET TEXFILE=%1

SET TEXFILEDRIVE=%~d1
SET TEXFILEPATH=%~p1
SET TEXFILENAME=%~n1

SET LANG=
SET LC_ALL=

set STYLESHEET=%TEXFILEDRIVE%%TEXFILEPATH%%TEXFILENAME%\%TEXFILENAME%.css

ECHO.========== Removing stylesheet '%STYLESHEET%'...
@CALL del %STYLESHEET%

:: Convert Windows path to Cygwin path
FOR /F "tokens=1 delims=" %%A in ('%CYGWIN%\bin\cygpath.exe --unix %TEXFILE%') do SET TEXFILECYG=%%A

ECHO.========== Running LaTeX2HTML on '%TEXFILE%'...
@CALL %LATEX2HTML% %TEXFILECYG%

:: Add custom styles to stylesheet
ECHO.========== Patching stylesheet '%STYLESHEET%'...
ECHO.body { >> %STYLESHEET%
ECHO.	font-family: Arial, Helvetical, sanf-serif; >> %STYLESHEET%
ECHO.	width: 1000px; >> %STYLESHEET%
ECHO.} >> %STYLESHEET%

ECHO.========== Done.
