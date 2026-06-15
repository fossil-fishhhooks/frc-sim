@echo off
setlocal enabledelayedexpansion
set BASE=%~dp0build\frc_sim.exe
set SCENE=%~dp0assets\scenes\955ML-1.json
set ROBOT=%~dp0assets\bodies\robot.json
set FLAGS=--threads 2 --speed 16 --substeps 1 --dt 0.0333 --w 480 --h 300

for /l %%i in (0,1,19) do (
    set /a PORT=5810+%%i
    start /min "%PORT%" "%BASE%" --scene "%SCENE%" --robot "%ROBOT%@127.0.0.1:!PORT!" !FLAGS!
)
