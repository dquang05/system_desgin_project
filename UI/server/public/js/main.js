/**
 * Main application entry point.
 * Initializes modules and sets up global UI event listeners.
 */

import { SocketManager } from './socketManager.js';
import { LogViewer } from './logViewer.js';
import { ChartManager } from './chartManager.js';
import { MotorManager } from './motorManager.js';
import { TuningManager } from './tuningManager.js';
import { ManualDriveManager } from './manualDriveManager.js';
import { DataRecordingManager } from './dataRecordingManager.js';

document.addEventListener('DOMContentLoaded', () => {
    // UI Elements
    const statusDot = document.getElementById('status-dot');
    const statusText = document.getElementById('status-text');
    const btnConnect = document.getElementById('btn-connect');
    const btnDisconnect = document.getElementById('btn-disconnect');
    
    // Tab Elements
    const tabBtns = document.querySelectorAll('.tab-btn');
    const tabContents = document.querySelectorAll('.tab-content');

    // Initialize Modules
    const socketMgr = new SocketManager();
    const logViewer = new LogViewer();
    const chartMgr = new ChartManager();
    const motorMgr = new MotorManager();
    const tuningMgr = new TuningManager(socketMgr);
    const manualDriveMgr = new ManualDriveManager(socketMgr);
    const dataRecMgr = new DataRecordingManager(socketMgr);
    
    // Setup Socket Listeners
    socketMgr.onStatusChange((state) => {
        statusDot.className = 'dot';
        if (state === 'listening') {
            statusDot.classList.add('green');
            statusText.textContent = 'Listening (Port 54321)';
        } else if (state === 'disconnected') {
            statusDot.classList.add('red');
            statusText.textContent = 'Disconnected';
        } else if (state === 'reconnecting') {
            statusDot.classList.add('yellow');
            statusText.textContent = 'Reconnecting...';
        }
    });

    socketMgr.onLogReceived((logEntry) => {
        logViewer.addLog(logEntry);
        chartMgr.processLog(logEntry);
        motorMgr.processLog(logEntry);
        tuningMgr.processLog(logEntry);
        dataRecMgr.processLog(logEntry);
    });

    socketMgr.onErrorMsg((msg) => {
        alert(`Error: ${msg}`);
    });

    // Setup Global Buttons
    btnConnect.addEventListener('click', () => socketMgr.connect());
    btnDisconnect.addEventListener('click', () => socketMgr.disconnect());
    document.getElementById('btn-start').addEventListener('click', () => socketMgr.startSystem());
    document.getElementById('btn-stop').addEventListener('click', () => socketMgr.stopSystem());
    document.getElementById('btn-clear').addEventListener('click', () => logViewer.clear());

    // Setup Tabs
    tabBtns.forEach(btn => {
        btn.addEventListener('click', () => {
            // Remove active class from all
            tabBtns.forEach(b => b.classList.remove('active'));
            tabContents.forEach(c => c.classList.remove('active'));
            
            // Add active class to selected
            btn.classList.add('active');
            const targetId = btn.getAttribute('data-target');
            document.getElementById(targetId).classList.add('active');
            
            // Notify modules
            logViewer.setActive(targetId === 'tab-main');
            chartMgr.setActive(targetId === 'tab-adc');
            motorMgr.setActive(targetId === 'tab-motor');
            tuningMgr.setActive(targetId === 'tab-motor');
            // manualDriveMgr & dataRecMgr don't need setActive polling for now
        });
    });
});
