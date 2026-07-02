// motorManager.js

export class MotorManager {
    constructor() {
        this.container = document.getElementById('motor-container');
        this.isActive = false;
        
        // Internal state
        this.state = {
            encLeft: 0,
            encRight: 0,
            pwmLeft: 0,
            pwmRight: 0,
            rpmTgtLeft: 0,
            rpmTgtRight: 0,
            rpmActLeft: 0,
            rpmActRight: 0
        };
        
        this.needsUpdate = false;
        
        // Build DOM
        this.buildUI();
        
        // Bind loop
        this.renderLoop = this.renderLoop.bind(this);
    }
    
    buildUI() {
        this.container.innerHTML = `
            <div class="motor-grid">
                <div class="motor-card">
                    <h3>Motor Left</h3>
                    <div class="motor-stat">
                        <span class="stat-label">PWM</span>
                        <span class="stat-value" id="pwm-l-val">0.00%</span>
                    </div>
                    <div class="pwm-bar-container">
                        <div class="pwm-bar fill-left" id="pwm-l-fill-neg"></div>
                        <div class="pwm-bar-center"></div>
                        <div class="pwm-bar fill-right" id="pwm-l-fill-pos"></div>
                    </div>
                    <div class="motor-stat mt-15">
                        <span class="stat-label">Encoder</span>
                        <span class="stat-value" id="enc-l-val">0</span>
                    </div>
                    <div class="motor-stat mt-15">
                        <span class="stat-label">RPM (Act/Tgt)</span>
                        <span class="stat-value"><span id="rpm-act-l-val" style="color: var(--text-primary)">0.00</span> / <span id="rpm-tgt-l-val" style="color: var(--text-secondary)">0.00</span></span>
                    </div>
                </div>
                <div class="motor-card">
                    <h3>Motor Right</h3>
                    <div class="motor-stat">
                        <span class="stat-label">PWM</span>
                        <span class="stat-value" id="pwm-r-val">0.00%</span>
                    </div>
                    <div class="pwm-bar-container">
                        <div class="pwm-bar fill-left" id="pwm-r-fill-neg"></div>
                        <div class="pwm-bar-center"></div>
                        <div class="pwm-bar fill-right" id="pwm-r-fill-pos"></div>
                    </div>
                    <div class="motor-stat mt-15">
                        <span class="stat-label">Encoder</span>
                        <span class="stat-value" id="enc-r-val">0</span>
                    </div>
                    <div class="motor-stat mt-15">
                        <span class="stat-label">RPM (Act/Tgt)</span>
                        <span class="stat-value"><span id="rpm-act-r-val" style="color: var(--text-primary)">0.00</span> / <span id="rpm-tgt-r-val" style="color: var(--text-secondary)">0.00</span></span>
                    </div>
                </div>
            </div>
        `;
        
        // Cache DOM elements
        this.elements = {
            pwmLVal: document.getElementById('pwm-l-val'),
            pwmLFillNeg: document.getElementById('pwm-l-fill-neg'),
            pwmLFillPos: document.getElementById('pwm-l-fill-pos'),
            encLVal: document.getElementById('enc-l-val'),
            rpmActLVal: document.getElementById('rpm-act-l-val'),
            rpmTgtLVal: document.getElementById('rpm-tgt-l-val'),
            
            pwmRVal: document.getElementById('pwm-r-val'),
            pwmRFillNeg: document.getElementById('pwm-r-fill-neg'),
            pwmRFillPos: document.getElementById('pwm-r-fill-pos'),
            encRVal: document.getElementById('enc-r-val'),
            rpmActRVal: document.getElementById('rpm-act-r-val'),
            rpmTgtRVal: document.getElementById('rpm-tgt-r-val')
        };
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
            let updated = false;
            
            if (obj.enc && Array.isArray(obj.enc) && obj.enc.length === 2) {
                this.state.encLeft = obj.enc[0];
                this.state.encRight = obj.enc[1];
                updated = true;
            }
            
            if (obj.pwm && Array.isArray(obj.pwm) && obj.pwm.length === 2) {
                this.state.pwmLeft = obj.pwm[0];
                this.state.pwmRight = obj.pwm[1];
                updated = true;
            }
            
            if (obj.rpm_tgt && Array.isArray(obj.rpm_tgt) && obj.rpm_tgt.length === 2) {
                this.state.rpmTgtLeft = obj.rpm_tgt[0];
                this.state.rpmTgtRight = obj.rpm_tgt[1];
                updated = true;
            }
            
            if (obj.rpm_act && Array.isArray(obj.rpm_act) && obj.rpm_act.length === 2) {
                this.state.rpmActLeft = obj.rpm_act[0];
                this.state.rpmActRight = obj.rpm_act[1];
                updated = true;
            }
            
            if (updated) {
                this.needsUpdate = true;
                if (this.isActive) {
                    requestAnimationFrame(this.renderLoop);
                }
            }
        } catch (e) {
            // Ignore non-JSON or improperly formatted logs
        }
    }
    
    updatePWMBar(valElem, negElem, posElem, value) {
        // Clamp value between -100 and 100
        const clamped = Math.max(-100, Math.min(100, value));
        valElem.textContent = clamped.toFixed(2) + '%';
        
        if (clamped < 0) {
            negElem.style.width = (Math.abs(clamped) / 2) + '%';
            posElem.style.width = '0%';
        } else {
            negElem.style.width = '0%';
            posElem.style.width = (clamped / 2) + '%';
        }
    }
    
    renderLoop() {
        if (!this.isActive || !this.needsUpdate) {
            return;
        }
        
        // Update DOM elements natively
        this.updatePWMBar(
            this.elements.pwmLVal, 
            this.elements.pwmLFillNeg, 
            this.elements.pwmLFillPos, 
            this.state.pwmLeft
        );
        this.elements.encLVal.textContent = this.state.encLeft.toString();
        this.elements.rpmActLVal.textContent = this.state.rpmActLeft.toFixed(2);
        this.elements.rpmTgtLVal.textContent = this.state.rpmTgtLeft.toFixed(2);
        
        this.updatePWMBar(
            this.elements.pwmRVal, 
            this.elements.pwmRFillNeg, 
            this.elements.pwmRFillPos, 
            this.state.pwmRight
        );
        this.elements.encRVal.textContent = this.state.encRight.toString();
        this.elements.rpmActRVal.textContent = this.state.rpmActRight.toFixed(2);
        this.elements.rpmTgtRVal.textContent = this.state.rpmTgtRight.toFixed(2);
        
        this.needsUpdate = false;
    }
}
