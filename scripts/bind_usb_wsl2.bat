REM Required for Arduino programming
REM Source: https://learn.microsoft.com/pt-br/windows/wsl/connect-usb

REM usbipd list # For listing the available USB ports

REM Requires administrative privileges
usbipd bind --busid 2-4
usbipd attach --wsl --busid 2-4