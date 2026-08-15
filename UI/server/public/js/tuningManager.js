// tuningManager.js

/**
 * Manages the tuning parameters UI and communicates tuning commands to the backend.
 */
export class TuningManager {
    /**
     * Initializes the TuningManager, binds UI elements, and sets up event listeners.
     * @param {SocketManager} socketMgr - Instance of SocketManager for communication.
     */
    constructor(socketMgr) {
        this.socketMgr = socketMgr;
        
        // Active display elements
        this.activeEls = {
            kpL: document.getElementById('active-kp-l'),
            kiL: document.getElementById('active-ki-l'),
            kdL: document.getElementById('active-kd-l'),
            
            kpR: document.getElementById('active-kp-r'),
            kiR: document.getElementById('active-ki-r'),
            kdR: document.getElementById('active-kd-r'),
            
            kpT: document.getElementById('active-kp-t'),
            kiT: document.getElementById('active-ki-t'),
            kdT: document.getElementById('active-kd-t')
        };
        
        // Input elements
        this.inputEls = {
            kpL: document.getElementById('input-kp-l'),
            kiL: document.getElementById('input-ki-l'),
            kdL: document.getElementById('input-kd-l'),
            
            kpR: document.getElementById('input-kp-r'),
            kiR: document.getElementById('input-ki-r'),
            kdR: document.getElementById('input-kd-r'),
            
            kpT: document.getElementById('input-kp-t'),
            kiT: document.getElementById('input-ki-t'),
            kdT: document.getElementById('input-kd-t')
        };
        
        // Buttons
        this.btnTestTune = document.getElementById('btn-tune-test');
        this.btnSave = document.getElementById('btn-tune-save');
        this.btnTuneTrack = document.getElementById('btn-tune-track');
        
        // Track Config Inputs
        this.trackInputEls = {
            vNormal: document.getElementById('input-track-v-normal'),
            vTurn: document.getElementById('input-track-v-turn'),
            slowStart: document.getElementById('input-track-slow-start'),
            slowEnd: document.getElementById('input-track-slow-end'),
            p1Out: document.getElementById('input-track-p1-out'),
            p1In: document.getElementById('input-track-p1-in'),
            p1Timeout: document.getElementById('input-track-p1-timeout'),
            p2Out: document.getElementById('input-track-p2-out'),
            p2In: document.getElementById('input-track-p2-in'),
            p2Thresh: document.getElementById('input-track-p2-thresh')
        };
        
        // State
        this.isActive = false;
        this.needsUpdate = false;
        this.renderPending = false; // Prevents multiple requestAnimationFrame calls in the same frame
        
        this.currentPid = {
            L: [0, 0, 0],
            R: [0, 0, 0],
            T: [0, 0, 0]
        };
        
        // Bind methods
        this.renderLoop = this.renderLoop.bind(this);
        
        // Attach listeners
        this.btnTestTune.addEventListener('click', () => this.sendTuneCommand());
        this.btnSave.addEventListener('click', () => this.sendSaveCommand());
        this.btnTuneTrack.addEventListener('click', () => this.sendTuneTrackCommand());
        
        // Listen to system state to disable buttons
        this.socketMgr.onSystemStateChange((isRunning) => {
            this.btnTestTune.disabled = isRunning;
            this.btnSave.disabled = isRunning;
            this.btnTuneTrack.disabled = isRunning;
        });
    }
    
    /**
     * Sets the active state of the tuning tab. Triggers rendering if needed.
     * @param {boolean} active - True if the tab is currently visible.
     */
    setActive(active) {
        this.isActive = active;
        if (active && this.needsUpdate && !this.renderPending) {
            this.renderPending = true;
            requestAnimationFrame(this.renderLoop);
        }
    }
    
    /**
     * Processes incoming log data to update active PID parameters.
     * @param {Object} logEntry - The log object.
     */
    processLog(logEntry) {
        try {
            const obj = JSON.parse(logEntry.data);
            if (obj.pid) {
                if (obj.pid.L && Array.isArray(obj.pid.L)) this.currentPid.L = obj.pid.L;
                if (obj.pid.R && Array.isArray(obj.pid.R)) this.currentPid.R = obj.pid.R;
                if (obj.pid.T && Array.isArray(obj.pid.T)) this.currentPid.T = obj.pid.T;
                
                this.needsUpdate = true;
                if (this.isActive && !this.renderPending) {
                    this.renderPending = true;
                    requestAnimationFrame(this.renderLoop);
                }
            }
        } catch (e) {
            // Ignore parse errors
        }
    }
    
    /**
     * Updates the DOM with the current active PID values.
     */
    renderLoop() {
        this.renderPending = false;
        
        if (!this.isActive || !this.needsUpdate) return;
        
        this.activeEls.kpL.textContent = this.currentPid.L[0].toFixed(3);
        this.activeEls.kiL.textContent = this.currentPid.L[1].toFixed(3);
        this.activeEls.kdL.textContent = this.currentPid.L[2].toFixed(3);
        
        this.activeEls.kpR.textContent = this.currentPid.R[0].toFixed(3);
        this.activeEls.kiR.textContent = this.currentPid.R[1].toFixed(3);
        this.activeEls.kdR.textContent = this.currentPid.R[2].toFixed(3);
        
        this.activeEls.kpT.textContent = this.currentPid.T[0].toFixed(3);
        this.activeEls.kiT.textContent = this.currentPid.T[1].toFixed(3);
        this.activeEls.kdT.textContent = this.currentPid.T[2].toFixed(3);
        
        this.needsUpdate = false;
    }
    
    /**
     * Sends the updated tuning parameters (Kp, Ki, Kd) for motors and tracker to the backend.
     */
    sendTuneCommand() {
        const payload = {
            cmd: 'tune',
            pid_L: [
                parseFloat(this.inputEls.kpL.value) || 0,
                parseFloat(this.inputEls.kiL.value) || 0,
                parseFloat(this.inputEls.kdL.value) || 0
            ],
            pid_R: [
                parseFloat(this.inputEls.kpR.value) || 0,
                parseFloat(this.inputEls.kiR.value) || 0,
                parseFloat(this.inputEls.kdR.value) || 0
            ],
            pid_T: [
                parseFloat(this.inputEls.kpT.value) || 0,
                parseFloat(this.inputEls.kiT.value) || 0,
                parseFloat(this.inputEls.kdT.value) || 0
            ],
            v_ref: parseFloat(this.trackInputEls.vNormal.value) || 200.0
        };
        
        if (this.socketMgr && this.socketMgr.socket) {
            this.socketMgr.sendUdp(payload);
        }
    }
    
    /**
     * Sends a command to save the current tuning parameters to flash memory on the ESP32.
     */
    sendSaveCommand() {
        const payload = {
            cmd: 'save'
        };
        
        if (this.socketMgr && this.socketMgr.socket) {
            this.socketMgr.sendUdp(payload);
        }
    }

    /**
     * Sends the track configuration (tune_track) to the backend.
     */
    sendTuneTrackCommand() {
        const payload = {
            cmd: 'tune_track',
            speed: [
                parseFloat(this.trackInputEls.vNormal.value) || 0,
                parseFloat(this.trackInputEls.vTurn.value) || 0,
                parseFloat(this.trackInputEls.slowStart.value) || 0,
                parseFloat(this.trackInputEls.slowEnd.value) || 0
            ],
            turn: [
                parseFloat(this.trackInputEls.p1Out.value) || 0,
                parseFloat(this.trackInputEls.p1In.value) || 0,
                parseInt(this.trackInputEls.p1Timeout.value) || 0,
                parseFloat(this.trackInputEls.p2Out.value) || 0,
                parseFloat(this.trackInputEls.p2In.value) || 0,
                parseFloat(this.trackInputEls.p2Thresh.value) || 0
            ]
        };
        
        if (this.socketMgr && this.socketMgr.socket) {
            this.socketMgr.sendUdp(payload);
        }
    }
}
