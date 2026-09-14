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
            kdT: document.getElementById('active-kd-t'),
            tauT: document.getElementById('active-tau-t'),
            
            kpT1: document.getElementById('active-kp-t1'),
            kdT1: document.getElementById('active-kd-t1'),
            tauT1: document.getElementById('active-tau-t1'),
            
            kpT2: document.getElementById('active-kp-t2'),
            kdT2: document.getElementById('active-kd-t2'),
            tauT2: document.getElementById('active-tau-t2'),

            w04: document.getElementById('active-w-04'),
            w13: document.getElementById('active-w-13'),

            vRef: document.getElementById('active-v-ref'),
            vRefTurn: document.getElementById('active-v-ref-turn'),

            blindS1RpmL: document.getElementById('active-blind-s1-rpm-l'),
            blindS1RpmR: document.getElementById('active-blind-s1-rpm-r'),
            blindS1PulsesL: document.getElementById('active-blind-s1-pulses-l'),
            blindS1PulsesR: document.getElementById('active-blind-s1-pulses-r'),

            blindS2RpmL: document.getElementById('active-blind-s2-rpm-l'),
            blindS2RpmR: document.getElementById('active-blind-s2-rpm-r'),
            blindS2PulsesL: document.getElementById('active-blind-s2-pulses-l'),
            blindS2PulsesR: document.getElementById('active-blind-s2-pulses-r'),

            blindS3RpmL: document.getElementById('active-blind-s3-rpm-l'),
            blindS3RpmR: document.getElementById('active-blind-s3-rpm-r')
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
            kdT: document.getElementById('input-kd-t'),
            tauT: document.getElementById('input-tau-t'),
            
            kpT1: document.getElementById('input-kp-t1'),
            kdT1: document.getElementById('input-kd-t1'),
            tauT1: document.getElementById('input-tau-t1'),
            
            kpT2: document.getElementById('input-kp-t2'),
            kdT2: document.getElementById('input-kd-t2'),
            tauT2: document.getElementById('input-tau-t2'),

            w04: document.getElementById('input-w-04'),
            w13: document.getElementById('input-w-13'),

            vRef: document.getElementById('input-v-ref'),
            vRefTurn: document.getElementById('input-v-ref-turn'),

            blindS1RpmL: document.getElementById('input-blind-s1-rpm-l'),
            blindS1RpmR: document.getElementById('input-blind-s1-rpm-r'),
            blindS1PulsesL: document.getElementById('input-blind-s1-pulses-l'),
            blindS1PulsesR: document.getElementById('input-blind-s1-pulses-r'),

            blindS2RpmL: document.getElementById('input-blind-s2-rpm-l'),
            blindS2RpmR: document.getElementById('input-blind-s2-rpm-r'),
            blindS2PulsesL: document.getElementById('input-blind-s2-pulses-l'),
            blindS2PulsesR: document.getElementById('input-blind-s2-pulses-r'),

            blindS3RpmL: document.getElementById('input-blind-s3-rpm-l'),
            blindS3RpmR: document.getElementById('input-blind-s3-rpm-r')
        };
        
        // Buttons
        this.btnTestTune = document.getElementById('btn-tune-test');
        this.btnSave = document.getElementById('btn-tune-save');
        this.btnToggleFuzzy = document.getElementById('btn-toggle-fuzzy');
        
        // State
        this.isActive = false;
        this.needsUpdate = false;
        this.renderPending = false; // Prevents multiple requestAnimationFrame calls in the same frame
        this.initializedInputs = false;
        this.fuzzyMode = false;
        this.activeFuzzy = false;
        this.userChangedFuzzy = false;
        
        this.currentPid = {
            L: [0, 0, 0],
            R: [0, 0, 0],
            T: [0, 0, 0.05],
            T1: [0, 0, 0, 0.05],
            T2: [0, 0, 0, 0.05],
            W: [4.0, 2.0],
            vRef: 0,
            vRefTurn: 0
        };

        this.currentBlind = {
            s1: [80.0, 80.0, 1000, 1000],
            s2: [60.0, 100.0, 1500, 1500],
            s3: [80.0, 80.0]
        };
        
        // Bind methods
        this.renderLoop = this.renderLoop.bind(this);
        this.updateFuzzyButtonUI = this.updateFuzzyButtonUI.bind(this);
        
        // Attach listeners
        this.btnTestTune.addEventListener('click', () => this.sendTuneCommand());
        this.btnSave.addEventListener('click', () => this.sendSaveCommand());
        if (this.btnToggleFuzzy) {
            this.btnToggleFuzzy.addEventListener('click', () => {
                this.userChangedFuzzy = true;
                this.fuzzyMode = !this.fuzzyMode;
                this.updateFuzzyButtonUI();
            });
        }
        
        // Listen to system state to disable buttons
        this.socketMgr.onSystemStateChange((isRunning) => {
            this.btnTestTune.disabled = isRunning;
            this.btnSave.disabled = isRunning;
        });

        // Enforce 3 decimal places on change/blur for Line Tracker inputs
        [
            this.inputEls.kpT, this.inputEls.kdT, this.inputEls.tauT,
            this.inputEls.kpT1, this.inputEls.kdT1, this.inputEls.tauT1,
            this.inputEls.kpT2, this.inputEls.kdT2, this.inputEls.tauT2
        ].forEach(input => {
            if (input) {
                input.addEventListener('change', () => {
                    const val = parseFloat(input.value);
                    if (!isNaN(val)) input.value = val.toFixed(3);
                });
            }
        });
    }

    /**
     * Updates the Fuzzy Mode toggle button appearance.
     */
    updateFuzzyButtonUI() {
        if (!this.btnToggleFuzzy) return;
        if (this.fuzzyMode) {
            this.btnToggleFuzzy.textContent = 'Fuzzy: ON';
            this.btnToggleFuzzy.className = 'btn fuzzy-on';
        } else {
            this.btnToggleFuzzy.textContent = 'Fuzzy: OFF';
            this.btnToggleFuzzy.className = 'btn secondary';
        }
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
            if (obj.fuzzy !== undefined) {
                this.activeFuzzy = Boolean(obj.fuzzy);
                if (!this.userChangedFuzzy) {
                    this.fuzzyMode = this.activeFuzzy;
                    this.updateFuzzyButtonUI();
                }
            }

            if (obj.blind) {
                if (obj.blind.s1 && Array.isArray(obj.blind.s1)) this.currentBlind.s1 = obj.blind.s1;
                if (obj.blind.s2 && Array.isArray(obj.blind.s2)) this.currentBlind.s2 = obj.blind.s2;
                if (obj.blind.s3 && Array.isArray(obj.blind.s3)) this.currentBlind.s3 = obj.blind.s3;
            }

            if (obj.pid) {
                if (obj.pid.L && Array.isArray(obj.pid.L)) this.currentPid.L = obj.pid.L;
                if (obj.pid.R && Array.isArray(obj.pid.R)) this.currentPid.R = obj.pid.R;
                if (obj.pid.T && Array.isArray(obj.pid.T)) this.currentPid.T = obj.pid.T;
                if (obj.pid.T1 && Array.isArray(obj.pid.T1)) this.currentPid.T1 = obj.pid.T1;
                if (obj.pid.T2 && Array.isArray(obj.pid.T2)) this.currentPid.T2 = obj.pid.T2;
                if (obj.pid.W && Array.isArray(obj.pid.W)) this.currentPid.W = obj.pid.W;
                
                if (obj.v_ref !== undefined) {
                    this.currentPid.vRef = obj.v_ref;
                }
                if (obj.v_ref_turn !== undefined) {
                    this.currentPid.vRefTurn = obj.v_ref_turn;
                }
                
                if (!this.initializedInputs) {
                    this.initializedInputs = true;
                    // Populate input fields with current values from ESP32
                    if (this.inputEls.kpL) this.inputEls.kpL.value = this.currentPid.L[0].toFixed(4);
                    if (this.inputEls.kiL) this.inputEls.kiL.value = this.currentPid.L[1].toFixed(4);
                    if (this.inputEls.kdL) this.inputEls.kdL.value = this.currentPid.L[2].toFixed(4);

                    if (this.inputEls.kpR) this.inputEls.kpR.value = this.currentPid.R[0].toFixed(4);
                    if (this.inputEls.kiR) this.inputEls.kiR.value = this.currentPid.R[1].toFixed(4);
                    if (this.inputEls.kdR) this.inputEls.kdR.value = this.currentPid.R[2].toFixed(4);

                    if (this.inputEls.kpT && this.currentPid.T.length > 0) this.inputEls.kpT.value = this.currentPid.T[0].toFixed(3);
                    if (this.inputEls.kdT && this.currentPid.T.length > 1) this.inputEls.kdT.value = this.currentPid.T[1].toFixed(3);
                    if (this.inputEls.tauT && this.currentPid.T.length > 2) this.inputEls.tauT.value = this.currentPid.T[2].toFixed(3);

                    if (this.inputEls.kpT1 && this.currentPid.T1) {
                        this.inputEls.kpT1.value = this.currentPid.T1[0].toFixed(3);
                        if (this.currentPid.T1.length > 1) this.inputEls.kdT1.value = this.currentPid.T1[1].toFixed(3);
                        if (this.inputEls.tauT1 && this.currentPid.T1.length > 2) this.inputEls.tauT1.value = this.currentPid.T1[2].toFixed(3);
                    }

                    if (this.inputEls.kpT2 && this.currentPid.T2) {
                        this.inputEls.kpT2.value = this.currentPid.T2[0].toFixed(3);
                        if (this.currentPid.T2.length > 1) this.inputEls.kdT2.value = this.currentPid.T2[1].toFixed(3);
                        if (this.inputEls.tauT2 && this.currentPid.T2.length > 2) this.inputEls.tauT2.value = this.currentPid.T2[2].toFixed(3);
                    }

                    if (this.inputEls.w04 && this.currentPid.W) {
                        this.inputEls.w04.value = this.currentPid.W[0].toFixed(1);
                        this.inputEls.w13.value = this.currentPid.W[1].toFixed(1);
                    }

                    if (this.inputEls.vRef) this.inputEls.vRef.value = this.currentPid.vRef.toFixed(1);
                    if (this.inputEls.vRefTurn) this.inputEls.vRefTurn.value = this.currentPid.vRefTurn.toFixed(1);

                    if (this.inputEls.blindS1RpmL && this.currentBlind.s1) {
                        this.inputEls.blindS1RpmL.value = this.currentBlind.s1[0].toFixed(1);
                        this.inputEls.blindS1RpmR.value = this.currentBlind.s1[1].toFixed(1);
                        this.inputEls.blindS1PulsesL.value = this.currentBlind.s1[2];
                        this.inputEls.blindS1PulsesR.value = this.currentBlind.s1[3];
                    }
                    if (this.inputEls.blindS2RpmL && this.currentBlind.s2) {
                        this.inputEls.blindS2RpmL.value = this.currentBlind.s2[0].toFixed(1);
                        this.inputEls.blindS2RpmR.value = this.currentBlind.s2[1].toFixed(1);
                        this.inputEls.blindS2PulsesL.value = this.currentBlind.s2[2];
                        this.inputEls.blindS2PulsesR.value = this.currentBlind.s2[3];
                    }
                    if (this.inputEls.blindS3RpmL && this.currentBlind.s3) {
                        this.inputEls.blindS3RpmL.value = this.currentBlind.s3[0].toFixed(1);
                        this.inputEls.blindS3RpmR.value = this.currentBlind.s3[1].toFixed(1);
                    }
                }
                
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
     * Updates the DOM with the current active PID and Blind Run values.
     */
    renderLoop() {
        this.renderPending = false;
        
        if (!this.isActive || !this.needsUpdate) return;
        
        this.activeEls.kpL.textContent = this.currentPid.L[0].toFixed(4);
        this.activeEls.kiL.textContent = this.currentPid.L[1].toFixed(4);
        this.activeEls.kdL.textContent = this.currentPid.L[2].toFixed(4);
        
        this.activeEls.kpR.textContent = this.currentPid.R[0].toFixed(4);
        this.activeEls.kiR.textContent = this.currentPid.R[1].toFixed(4);
        this.activeEls.kdR.textContent = this.currentPid.R[2].toFixed(4);
        
        if (this.activeEls.kpT && this.currentPid.T.length > 0) this.activeEls.kpT.textContent = this.currentPid.T[0].toFixed(3);
        if (this.activeEls.kdT && this.currentPid.T.length > 1) this.activeEls.kdT.textContent = this.currentPid.T[1].toFixed(3);
        if (this.activeEls.tauT && this.currentPid.T.length > 2) this.activeEls.tauT.textContent = this.currentPid.T[2].toFixed(3);

        if (this.activeEls.kpT1 && this.currentPid.T1) {
            this.activeEls.kpT1.textContent = this.currentPid.T1[0].toFixed(3);
            if (this.currentPid.T1.length > 1) this.activeEls.kdT1.textContent = this.currentPid.T1[1].toFixed(3);
            if (this.activeEls.tauT1 && this.currentPid.T1.length > 2) this.activeEls.tauT1.textContent = this.currentPid.T1[2].toFixed(3);
        }

        if (this.activeEls.kpT2 && this.currentPid.T2) {
            this.activeEls.kpT2.textContent = this.currentPid.T2[0].toFixed(3);
            if (this.currentPid.T2.length > 1) this.activeEls.kdT2.textContent = this.currentPid.T2[1].toFixed(3);
            if (this.activeEls.tauT2 && this.currentPid.T2.length > 2) this.activeEls.tauT2.textContent = this.currentPid.T2[2].toFixed(3);
        }

        if (this.activeEls.w04 && this.currentPid.W) {
            this.activeEls.w04.textContent = this.currentPid.W[0].toFixed(1);
            this.activeEls.w13.textContent = this.currentPid.W[1].toFixed(1);
        }
        
        if (this.activeEls.vRef) {
            this.activeEls.vRef.textContent = this.currentPid.vRef.toFixed(1);
        }
        if (this.activeEls.vRefTurn) {
            this.activeEls.vRefTurn.textContent = this.currentPid.vRefTurn.toFixed(1);
        }

        if (this.activeEls.blindS1RpmL && this.currentBlind.s1) {
            this.activeEls.blindS1RpmL.textContent = this.currentBlind.s1[0].toFixed(1);
            this.activeEls.blindS1RpmR.textContent = this.currentBlind.s1[1].toFixed(1);
            this.activeEls.blindS1PulsesL.textContent = this.currentBlind.s1[2];
            this.activeEls.blindS1PulsesR.textContent = this.currentBlind.s1[3];
        }
        if (this.activeEls.blindS2RpmL && this.currentBlind.s2) {
            this.activeEls.blindS2RpmL.textContent = this.currentBlind.s2[0].toFixed(1);
            this.activeEls.blindS2RpmR.textContent = this.currentBlind.s2[1].toFixed(1);
            this.activeEls.blindS2PulsesL.textContent = this.currentBlind.s2[2];
            this.activeEls.blindS2PulsesR.textContent = this.currentBlind.s2[3];
        }
        if (this.activeEls.blindS3RpmL && this.currentBlind.s3) {
            this.activeEls.blindS3RpmL.textContent = this.currentBlind.s3[0].toFixed(1);
            this.activeEls.blindS3RpmR.textContent = this.currentBlind.s3[1].toFixed(1);
        }
        
        this.needsUpdate = false;
    }
    
    /**
     * Sends the updated tuning parameters (Kp, Ki, Kd, Tau, Fuzzy, Blind Run) to the backend.
     */
    sendTuneCommand() {
        const payload = {
            cmd: 'tune',
            fuzzy: this.fuzzyMode,
            blind_s1: [
                parseFloat(this.inputEls.blindS1RpmL?.value) || 0,
                parseFloat(this.inputEls.blindS1RpmR?.value) || 0,
                parseInt(this.inputEls.blindS1PulsesL?.value, 10) || 0,
                parseInt(this.inputEls.blindS1PulsesR?.value, 10) || 0
            ],
            blind_s2: [
                parseFloat(this.inputEls.blindS2RpmL?.value) || 0,
                parseFloat(this.inputEls.blindS2RpmR?.value) || 0,
                parseInt(this.inputEls.blindS2PulsesL?.value, 10) || 0,
                parseInt(this.inputEls.blindS2PulsesR?.value, 10) || 0
            ],
            blind_s3: [
                parseFloat(this.inputEls.blindS3RpmL?.value) || 0,
                parseFloat(this.inputEls.blindS3RpmR?.value) || 0
            ],
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
                Math.round((parseFloat(this.inputEls.kpT.value) || 0) * 1000) / 1000,
                Math.round((parseFloat(this.inputEls.kdT.value) || 0) * 1000) / 1000,
                Math.round((parseFloat(this.inputEls.tauT.value) || 0) * 1000) / 1000
            ],
            pid_T_1: [
                Math.round((parseFloat(this.inputEls.kpT1?.value) || 0) * 1000) / 1000,
                Math.round((parseFloat(this.inputEls.kdT1?.value) || 0) * 1000) / 1000,
                Math.round((parseFloat(this.inputEls.tauT1?.value) || 0) * 1000) / 1000
            ],
            pid_T_2: [
                Math.round((parseFloat(this.inputEls.kpT2?.value) || 0) * 1000) / 1000,
                Math.round((parseFloat(this.inputEls.kdT2?.value) || 0) * 1000) / 1000,
                Math.round((parseFloat(this.inputEls.tauT2?.value) || 0) * 1000) / 1000
            ],
            sensor_weights: [
                parseFloat(this.inputEls.w04?.value) || 0,
                parseFloat(this.inputEls.w13?.value) || 0
            ],
            v_ref: parseFloat(this.inputEls.vRef.value) || 0,
            v_ref_turn: parseFloat(this.inputEls.vRefTurn.value) || 0
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
}
