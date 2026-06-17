@echo off
set BASE=%~dp0build\frc_sim.exe
set SCENE=%~dp0assets\scenes\955ML-1.json
set ROBOT=%~dp0assets\bodies\robot.json
set RAYS=%~dp0assets\raycasts\def.json
set FLAGS=--threads 2 --speed 16 --substeps 1 --dt 0.0333 --w 480 --h 300 --raycast %RAYS%

start /min "6810" "%BASE%" --scene "%SCENE%" --robot "%ROBOT%@127.0.0.1:6810" %FLAGS%
start /min "6811" "%BASE%" --scene "%SCENE%" --robot "%ROBOT%@127.0.0.1:6811" %FLAGS%
start /min "6812" "%BASE%" --scene "%SCENE%" --robot "%ROBOT%@127.0.0.1:6812" %FLAGS%
start /min "6813" "%BASE%" --scene "%SCENE%" --robot "%ROBOT%@127.0.0.1:6813" %FLAGS%
start /min "6814" "%BASE%" --scene "%SCENE%" --robot "%ROBOT%@127.0.0.1:6814" %FLAGS%
start /min "6815" "%BASE%" --scene "%SCENE%" --robot "%ROBOT%@127.0.0.1:6815" %FLAGS%
start /min "6816" "%BASE%" --scene "%SCENE%" --robot "%ROBOT%@127.0.0.1:6816" %FLAGS%
start /min "6817" "%BASE%" --scene "%SCENE%" --robot "%ROBOT%@127.0.0.1:6817" %FLAGS%
start /min "6818" "%BASE%" --scene "%SCENE%" --robot "%ROBOT%@127.0.0.1:6818" %FLAGS%
start /min "6819" "%BASE%" --scene "%SCENE%" --robot "%ROBOT%@127.0.0.1:6819" %FLAGS%
start /min "6820" "%BASE%" --scene "%SCENE%" --robot "%ROBOT%@127.0.0.1:6820" %FLAGS%
start /min "6821" "%BASE%" --scene "%SCENE%" --robot "%ROBOT%@127.0.0.1:6821" %FLAGS%
start /min "6822" "%BASE%" --scene "%SCENE%" --robot "%ROBOT%@127.0.0.1:6822" %FLAGS%
start /min "6823" "%BASE%" --scene "%SCENE%" --robot "%ROBOT%@127.0.0.1:6823" %FLAGS%
start /min "6824" "%BASE%" --scene "%SCENE%" --robot "%ROBOT%@127.0.0.1:6824" %FLAGS%
start /min "6825" "%BASE%" --scene "%SCENE%" --robot "%ROBOT%@127.0.0.1:6825" %FLAGS%
