// manualDriveManager.js

/**
 * Manages the manual drive UI and communicates commands to the backend.
 */
export class ManualDriveManager {
    /**
     * Initializes the ManualDriveManager, binds UI elements, and sets up event listeners.
     * @param {SocketManager} socketMgr - Instance of SocketManager for communication.
     */
    constructor(socketMgr) {
        this.socketMgr = socketMgr;
        
        // Input elements
        this.inputRpmL = document.getElementById('input-manual-rpm-l');
        this.inputRpmR = document.getElementById('input-manual-rpm-r');
        
        // Buttons
        this.btnSend = document.getElementById('btn-manual-send');
        this.btnStop = document.getElementById('btn-manual-stop');
        
        // Attach listeners
        this.btnSend.addEventListener('click', () => this.sendManualDriveCommand());
        this.btnStop.addEventListener('click', () => this.sendStopCommand());
        
        // Listen to system state to disable buttons
        this.socketMgr.onSystemStateChange((isRunning) => {
            this.btnSend.disabled = isRunning;
            this.btnStop.disabled = isRunning;
        });
    }
    
    /**
     * Sends the manual drive command with current RPM values.
     */
    sendManualDriveCommand() {
        const payload = {
            cmd: 'manual_drive',
            rpm_l: Math.min(Math.max(parseFloat(this.inputRpmL.value) || 0, -333), 333),
            rpm_r: Math.min(Math.max(parseFloat(this.inputRpmR.value) || 0, -333), 333)
        };
        
        if (this.socketMgr && this.socketMgr.socket) {
            this.socketMgr.sendUdp(payload);
        }
    }
    
    /**
     * Sends a stop command (0 RPM) and resets the inputs.
     */
    sendStopCommand() {
        this.inputRpmL.value = 0;
        this.inputRpmR.value = 0;
        
        const payload = {
            cmd: 'manual_drive',
            rpm_l: 0,
            rpm_r: 0
        };
        
        if (this.socketMgr && this.socketMgr.socket) {
            this.socketMgr.sendUdp(payload);
        }
    }
}
