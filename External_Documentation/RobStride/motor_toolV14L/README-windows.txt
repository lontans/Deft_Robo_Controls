RobStride Motor Studio (motor_toolV14L) — Windows launch notes
============================================================

HOW TO START
  Double-click run_motor_tool.bat  (preferred)
  Do NOT copy motor_tool.exe elsewhere — Qt plugins must stay as sibling folders:
    platforms\qwindows.dll
    imageformats\
    styles\
    iconengines\
    printsupport\
    Qt5*.dll, libgcc_s_seh-1.dll, libstdc++-6.dll, libwinpthread-1.dll

COMMON ERRORS
  1) "Could not find or load the Qt platform plugin windows"
     -> Run from this folder via run_motor_tool.bat (qt.conf is included).

  2) "Side-by-side configuration is incorrect" / MinGW runtime
     -> Re-extract the FULL zip from RobStride (do not mix DLLs from other versions).
     -> Install: Microsoft Visual C++ 2015-2022 Redistributable (x64)
        https://aka.ms/vs/17/release/vc_redist.x64.exe
     -> Unblock files: right-click zip -> Properties -> Unblock -> Extract again.

  3) "Missing XXX.dll"
     -> You may have only motor_toolV14L from a larger package. Check the original
        zip for a sibling folder (e.g. motor_toolV14) or extra DLLs at the same
        level as motor_toolV14L. Extract the entire archive to e.g. C:\RobStride\

  4) App starts but no COM port
     -> Install CH340 USB driver for the RobStride USB-CAN adapter (WCH CH340).
        Device Manager should show "USB-SERIAL CH340 (COMx)" not an unknown device.

  5) Running from git repo deep path
     -> If still failing, copy the whole motor_toolV14L folder to C:\RobStride\motor_toolV14L
        and run run_motor_tool.bat there.

CONNECTION (for RS02 encoder cal)
  - RobStride USB-CAN dongle on motor CANH/CANL (not the Deft PCB for first cal)
  - 48 V supply to motor
  - Device module -> Connect -> motor type RS02 -> CAN ID 0x70
  - 电机编码器标定 (encoder cal) -> 设置机械零位 -> 参数保存
