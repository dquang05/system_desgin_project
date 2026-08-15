// motorManager.js

/**
 * Manages the motor telemetry and UI rendering, including PWM bars and charts.
 */
export class MotorManager {
    /**
     * Initializes the MotorManager, builds the DOM, and sets up charts.
     */
    constructor() {
        this.isActive = false;
        this.state = {
            encLeft: 0,
            encRight: 0,
            pwmLeft: 0,
            pwmRight: 0,
            rpmTgtLeft: 0,
            rpmTgtRight: 0,
            rpmActLeft: 0,
            rpmActRight: 0,
            weight: 0,
            overshootLeft: 0,
            sseLeft: 0,
            overshootRight: 0,
            sseRight: 0
        };
        
        this.stepTracking = {
            left: { target: 0, maxAct: 0, minAct: 0, inStep: false },
            right: { target: 0, maxAct: 0, minAct: 0, inStep: false }
        };
        
        this.needsUpdate = false;
        this.renderPending = false; // Prevents multiple requestAnimationFrame calls in the same frame
        
        // Chart configuration
        this.maxDataPoints = 100;
        
        // Build DOM
        this.buildUI();
        
        // Initialize Charts
        this.initCharts();
        
        // Bind loop
        this.renderLoop = this.renderLoop.bind(this);
    }
    
    /**
     * Builds the inner HTML structure for the motor container.
     */
    buildUI() {
        // The HTML structure is now statically defined in index.html
        
        // Cache DOM elements
        this.elements = {
            pwmLVal: document.getElementById('pwm-l-val'),
            pwmLFillNeg: document.getElementById('pwm-l-fill-neg'),
            pwmLFillPos: document.getElementById('pwm-l-fill-pos'),
            encLVal: document.getElementById('enc-l-val'),
            rpmActLVal: document.getElementById('rpm-act-l-val'),
            rpmTgtLVal: document.getElementById('rpm-tgt-l-val'),
            overshootLVal: document.getElementById('overshoot-l-val'),
            sseLVal: document.getElementById('sse-l-val'),
            
            pwmRVal: document.getElementById('pwm-r-val'),
            pwmRFillNeg: document.getElementById('pwm-r-fill-neg'),
            pwmRFillPos: document.getElementById('pwm-r-fill-pos'),
            encRVal: document.getElementById('enc-r-val'),
            rpmActRVal: document.getElementById('rpm-act-r-val'),
            rpmTgtRVal: document.getElementById('rpm-tgt-r-val'),
            overshootRVal: document.getElementById('overshoot-r-val'),
            sseRVal: document.getElementById('sse-r-val'),
            
            weightVal: document.getElementById('weight-val')
        };
    }

    /**
     * Helper to create standard chart configurations.
     * @param {string} yLabel - Label for the Y axis.
     * @param {number} [ySuggestedMax] - Optional maximum value for the Y axis.
     * @param {Array} datasets - Array of dataset configuration objects.
     * @returns {Object} Chart.js configuration object.
     */
    createChartConfig(yLabel, ySuggestedMax, datasets) {
        return {
            type: 'line',
            data: {
                labels: [],
                datasets: datasets
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                animation: false, // For performance
                interaction: {
                    mode: 'nearest',
                    axis: 'x',
                    intersect: false
                },
                plugins: {
                    legend: {
                        labels: { color: '#e6edf3' }
                    }
                },
                scales: {
                    x: {
                        type: 'category',
                        display: true,
                        ticks: { color: '#8b949e', maxTicksLimit: 10 },
                        grid: { color: '#30363d' }
                    },
                    y: {
                        display: true,
                        title: { display: true, text: yLabel, color: '#8b949e' },
                        ticks: { color: '#8b949e' },
                        grid: { color: '#30363d' },
                        suggestedMin: 0,
                        ...(ySuggestedMax && { suggestedMax: ySuggestedMax })
                    }
                }
            }
        };
    }

    /**
     * Initializes the Chart.js instances for motors and loadcell.
     */
    initCharts() {
        const ctxL = document.getElementById('motor-l-chart').getContext('2d');
        this.chartL = new Chart(ctxL, this.createChartConfig('RPM', undefined, [
            { label: 'Target', borderColor: '#8b949e', data: [], fill: false, tension: 0.1, pointRadius: 0, borderDash: [5, 5] },
            { label: 'Actual', borderColor: '#2f81f7', data: [], fill: false, tension: 0.1, pointRadius: 0 }
        ]));
        
        const ctxR = document.getElementById('motor-r-chart').getContext('2d');
        this.chartR = new Chart(ctxR, this.createChartConfig('RPM', undefined, [
            { label: 'Target', borderColor: '#8b949e', data: [], fill: false, tension: 0.1, pointRadius: 0, borderDash: [5, 5] },
            { label: 'Actual', borderColor: '#3fb950', data: [], fill: false, tension: 0.1, pointRadius: 0 }
        ]));
        
        const ctxW = document.getElementById('weight-chart').getContext('2d');
        this.chartW = new Chart(ctxW, this.createChartConfig('Weight', undefined, [
            { label: 'Weight', borderColor: '#d29922', data: [], fill: false, tension: 0.1, pointRadius: 0 }
        ]));
    }
    
    /**
     * Sets the active state. Triggers rendering if the tab becomes active.
     * @param {boolean} active - True if the motor tab is visible.
     */
    setActive(active) {
        this.isActive = active;
        if (active && this.needsUpdate && !this.renderPending) {
            this.renderPending = true;
            requestAnimationFrame(this.renderLoop);
        }
    }
    
    /**
     * Processes incoming telemetry and updates the internal state.
     * @param {Object} logEntry - The log data containing telemetry.
     */
    processLog(logEntry) {
        try {
            const obj = JSON.parse(logEntry.data);
            let updated = false;
            const timeStr = logEntry.timestamp;
            
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
                if (Math.abs(obj.rpm_tgt[0] - this.state.rpmTgtLeft) > 1.0) {
                    this.stepTracking.left.target = obj.rpm_tgt[0];
                    this.stepTracking.left.maxAct = this.state.rpmActLeft;
                    this.stepTracking.left.minAct = this.state.rpmActLeft;
                    this.stepTracking.left.inStep = true;
                }
                if (Math.abs(obj.rpm_tgt[1] - this.state.rpmTgtRight) > 1.0) {
                    this.stepTracking.right.target = obj.rpm_tgt[1];
                    this.stepTracking.right.maxAct = this.state.rpmActRight;
                    this.stepTracking.right.minAct = this.state.rpmActRight;
                    this.stepTracking.right.inStep = true;
                }
                this.state.rpmTgtLeft = obj.rpm_tgt[0];
                this.state.rpmTgtRight = obj.rpm_tgt[1];
                updated = true;
            }
            
            if (obj.rpm_act && Array.isArray(obj.rpm_act) && obj.rpm_act.length === 2) {
                this.state.rpmActLeft = obj.rpm_act[0];
                this.state.rpmActRight = obj.rpm_act[1];
                
                // Left metrics
                if (this.stepTracking.left.inStep) {
                    this.stepTracking.left.maxAct = Math.max(this.stepTracking.left.maxAct, this.state.rpmActLeft);
                    this.stepTracking.left.minAct = Math.min(this.stepTracking.left.minAct, this.state.rpmActLeft);
                    if (this.stepTracking.left.target >= 0) {
                        this.state.overshootLeft = Math.max(0, this.stepTracking.left.maxAct - this.stepTracking.left.target);
                    } else {
                        this.state.overshootLeft = Math.max(0, this.stepTracking.left.target - this.stepTracking.left.minAct);
                    }
                }
                this.state.sseLeft = this.state.rpmTgtLeft - this.state.rpmActLeft;
                
                // Right metrics
                if (this.stepTracking.right.inStep) {
                    this.stepTracking.right.maxAct = Math.max(this.stepTracking.right.maxAct, this.state.rpmActRight);
                    this.stepTracking.right.minAct = Math.min(this.stepTracking.right.minAct, this.state.rpmActRight);
                    if (this.stepTracking.right.target >= 0) {
                        this.state.overshootRight = Math.max(0, this.stepTracking.right.maxAct - this.stepTracking.right.target);
                    } else {
                        this.state.overshootRight = Math.max(0, this.stepTracking.right.target - this.stepTracking.right.minAct);
                    }
                }
                this.state.sseRight = this.state.rpmTgtRight - this.state.rpmActRight;
                
                updated = true;
            }

            if (obj.weight !== undefined) {
                this.state.weight = obj.weight;
                updated = true;
            }
            
            if (updated) {
                // Update chart data arrays
                this.updateChartData(this.chartL, timeStr, [this.state.rpmTgtLeft, this.state.rpmActLeft]);
                this.updateChartData(this.chartR, timeStr, [this.state.rpmTgtRight, this.state.rpmActRight]);
                this.updateChartData(this.chartW, timeStr, [this.state.weight]);

                this.needsUpdate = true;
                if (this.isActive && !this.renderPending) {
                    this.renderPending = true;
                    requestAnimationFrame(this.renderLoop);
                }
            }
        } catch (e) {
            // Ignore non-JSON or improperly formatted logs
        }
    }

    /**
     * Updates data in a chart instance and shifts the dataset if it exceeds max size.
     * @param {Object} chart - Chart.js instance.
     * @param {string} label - The label (timestamp).
     * @param {Array} dataArr - Array of new values matching the datasets.
     */
    updateChartData(chart, label, dataArr) {
        chart.data.labels.push(label);
        for (let i = 0; i < dataArr.length; i++) {
            chart.data.datasets[i].data.push(dataArr[i]);
        }
        
        if (chart.data.labels.length > this.maxDataPoints) {
            chart.data.labels.shift();
            for (let i = 0; i < dataArr.length; i++) {
                chart.data.datasets[i].data.shift();
            }
        }
    }
    
    /**
     * Updates the UI for the PWM bar to visually represent -100% to 100%.
     * @param {HTMLElement} valElem - Element displaying the PWM text.
     * @param {HTMLElement} negElem - Element for the negative fill.
     * @param {HTMLElement} posElem - Element for the positive fill.
     * @param {number} value - The PWM value.
     */
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
    
    /**
     * Renders the current state to the DOM and updates charts via requestAnimationFrame.
     */
    renderLoop() {
        this.renderPending = false;
        
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
        if (this.elements.overshootLVal) this.elements.overshootLVal.textContent = this.state.overshootLeft.toFixed(2);
        if (this.elements.sseLVal) this.elements.sseLVal.textContent = this.state.sseLeft.toFixed(2);
        
        this.updatePWMBar(
            this.elements.pwmRVal, 
            this.elements.pwmRFillNeg, 
            this.elements.pwmRFillPos, 
            this.state.pwmRight
        );
        this.elements.encRVal.textContent = this.state.encRight.toString();
        this.elements.rpmActRVal.textContent = this.state.rpmActRight.toFixed(2);
        this.elements.rpmTgtRVal.textContent = this.state.rpmTgtRight.toFixed(2);
        if (this.elements.overshootRVal) this.elements.overshootRVal.textContent = this.state.overshootRight.toFixed(2);
        if (this.elements.sseRVal) this.elements.sseRVal.textContent = this.state.sseRight.toFixed(2);

        if (this.elements.weightVal) {
            this.elements.weightVal.textContent = this.state.weight.toFixed(2);
        }

        // Update charts natively
        this.chartL.update();
        this.chartR.update();
        this.chartW.update();
        
        this.needsUpdate = false;
    }
}
