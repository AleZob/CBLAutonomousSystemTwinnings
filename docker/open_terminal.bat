@echo off
REM Find the container ID or name
for /f "tokens=*" %%i in ('docker ps --format "{{.Names}}"') do (
    set CONTAINER_NAME=%%i
    goto :OPEN_TERMINAL
)

:OPEN_TERMINAL
REM Open a Bash terminal in the running container
docker exec -it %CONTAINER_NAME% /bin/bash