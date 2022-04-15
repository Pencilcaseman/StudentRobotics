@ECHO OFF
if exist robot.zip (
	echo Deleting robot.zip
    del robot.zip
) else (
    echo robot.zip not found
)

echo Zipping code into robot.zip
tar.exe -a -c -f robot.zip *.py

echo Reformatting drive...
for /f %%D in ('wmic volume get DriveLetter^, Label ^| find "CHODE"') do set usb=%%D
if "%usb%"=="" goto :nousb
echo Using drive %usb%.

:choice
set /P c=Are you sure you want to continue[Y/N]?
if /I "%c%" EQU "Y" goto :continue
if /I "%c%" EQU "N" goto :exit
goto :choice

:continue
rd %usb%\. /S /Q
copy robot.zip %usb%\robot.zip
RemoveDrive.exe %usb% -L
echo Complete
goto :exit

:nousb
echo USB not found. Change label??

:exit
PAUSE
