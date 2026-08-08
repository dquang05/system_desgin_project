// tuningManager.js

export class TuningManager {
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
        
        // State
        this.isActive = false;
        this.needsUpdate = false;
        
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
    }
    
    setActive(active) {
        this.isActive = active;
        if (active && this.needsUpdate) {
            requestAnimationFrame(this.renderLoop);
        }
    }
    
    processLog(logEntry) {
        try {
            const obj = JSON.parse(logEntry.data);
            if (obj.pid) {
                if (obj.pid.L && Array.isArray(obj.pid.L)) this.currentPid.L = obj.pid.L;
                if (obj.pid.R && Array.isArray(obj.pid.R)) this.currentPid.R = obj.pid.R;
                if (obj.pid.T && Array.isArray(obj.pid.T)) this.currentPid.T = obj.pid.T;
                
                this.needsUpdate = true;
                if (this.isActive) {
                    requestAnimationFrame(this.renderLoop);
                }
            }
        } catch (e) {
            // Ignore parse errors
        }
    }
    
    renderLoop() {
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
            ]
        };
        
        if (this.socketMgr && this.socketMgr.socket) {
            this.socketMgr.socket.emit('send_tune', payload);
        }
    }
    
    sendSaveCommand() {
        const payload = {
            cmd: 'save'
        };
        
        if (this.socketMgr && this.socketMgr.socket) {
            this.socketMgr.socket.emit('send_tune', payload);
        }
    }
}
