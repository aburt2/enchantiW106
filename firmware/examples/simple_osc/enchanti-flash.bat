@echo off

rem Set common timeout
set TIMEOUT=-t 60000
set PORT_SPEED=2000000

rem BLHOST_PATH should be saved as an environmental variable 
set BLHOST_PATH=blhost 
set F_APP_BIN="%cd%/build/zephyr/zephyr.bin" 

rem Function to show the main menu and prompt for module variant
:main_menu
echo Flashing EnchantiW106 Board

set MODULE=Fidelix
goto :prompt_inputs

rem Function to prompt for user inputs
:prompt_inputs
set /p PORT=Enter COM port number (e.g., COM1): 
set COM_PORT=%PORT%,%PORT_SPEED%
goto :execute

rem Function to perform all steps for Fidelix
:fidelix_steps
echo Starting Fidelix steps...
timeout /t 2

rem Step 1: Writing FCB_FID.bin to memory
set FCB_FID="%cd%\..\FCB_FID.bin"
%BLHOST_PATH% -p %COM_PORT% %TIMEOUT% -- write-memory 0x20001000 %FCB_FID%
if %errorlevel% neq 0 goto :step_failed

echo Step 1 successful.

echo Step 2: Configuring memory...
%BLHOST_PATH% -p %COM_PORT% %TIMEOUT% -- configure-memory 0x9 0x20001000
if %errorlevel% neq 0 goto :step_failed

echo Step 2 successful.

echo Step 3: Getting property...
%BLHOST_PATH% -p %COM_PORT% %TIMEOUT% -- get-property 0x19 0x9
if %errorlevel% neq 0 goto :step_failed

echo Step 3 successful.

echo Step 4: Flash erasing region 0x08000000 
%BLHOST_PATH% -p %COM_PORT% %TIMEOUT% -- flash-erase-region 0x08000000 0x3FFFFF
if %errorlevel% neq 0 goto :step_failed

echo Step 4 successful.

echo Step 5: Filling memory...
%BLHOST_PATH% -p %COM_PORT% %TIMEOUT% -- fill-memory 0x20001000 0x04 0xf000000f
if %errorlevel% neq 0 goto :step_failed

echo Step 5 successful.

echo Step 6: Configuring memory...
%BLHOST_PATH% -p %COM_PORT% %TIMEOUT% -- configure-memory 0x9 0x20001000
if %errorlevel% neq 0 goto :step_failed

echo Step 6 successful.

echo Step 7: Flash erasing region 0x08400000 
%BLHOST_PATH% -p %COM_PORT% %TIMEOUT% -- flash-erase-region 0x08400000 0x1DFFFF
if %errorlevel% neq 0 goto :step_failed

echo Step 7 successful.

echo Step 8: Flash erasing region 0x08000000 
%BLHOST_PATH% -p %COM_PORT% %TIMEOUT% -- flash-erase-region 0x08000000 0x3FFFFF
if %errorlevel% neq 0 goto :step_failed

echo Step 8 successful.


rem Flashing application
echo Step 9: Writing application .bin to memory...
%BLHOST_PATH% -p %COM_PORT% %TIMEOUT% -- write-memory 0x08000000 %F_APP_BIN%
if %errorlevel% neq 0 goto :step_failed

echo Step 9 successful.

echo End of Fidelix .

goto :done

rem Main script execution
:execute
if /i "%MODULE%" equ "Macronix" (
    call :macronix_steps
) else if /i "%MODULE%" equ "Fidelix" (
    call :fidelix_steps
)

:done
echo End
goto :eof

rem Handle failed steps
:step_failed
echo An error occurred. Exiting...
goto :done

rem End of script
:end
echo Script terminated.
