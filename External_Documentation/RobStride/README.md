# RobStride vendor docs

Firmware PDFs live under `RS01/` … `RS04/`. Sample C sources are in this folder.

## Motor Studio (GUI) — not in this repo

`motor_toolV14L` (Qt Windows GUI, ~35–50 MB of `motor_tool.exe` + Qt DLLs) was
removed from git tracking: it is a vendor installer tree, not documentation.

Get it from RobStride’s release package / support channel and keep it outside
the firmware tree (e.g. `C:\RobStride\motor_toolV14L`). Run via their
`run_motor_tool.bat` so Qt plugins stay beside the exe.
