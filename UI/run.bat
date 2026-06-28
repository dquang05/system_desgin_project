@echo off
title ESP32 Log Monitor Server
echo ==========================================
echo    ESP32 Local Log Server - Port 8080
echo ==========================================

cd /d "%~dp0server"

if not exist node_modules (
    echo Installing dependencies Express, Socket.io ...
    call npm install
)

echo.
echo Starting Backend Server...
node server.js

pause
