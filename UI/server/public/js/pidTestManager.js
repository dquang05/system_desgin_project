// pidTestManager.js

/**
 * Manages the PID Step Response Test UI and communication.
 */
export class PidTestManager {
    constructor(socketMgr) {
        this.socketMgr = socketMgr;
        
        // UI Elements
        this.btnTestPid = document.getElementById('btn-test-pid');
        this.inputRpmL = document.getElementById('input-manual-rpm-l');
        this.inputRpmR = document.getElementById('input-manual-rpm-r');
        
        this.btnShowChart = document.getElementById('btn-show-pid-chart');
        this.modal = document.getElementById('pid-chart-modal');
        this.btnCloseModal = document.getElementById('close-pid-modal');
        
        // State
        this.isTesting = false;
        this.isRecording = false;
        this.testTimer = null;
        
        // Data Arrays
        this.timestamps = [];
        this.rpm_act_l = [];
        this.rpm_act_r = [];
        this.rpm_tgt_l = [];
        this.rpm_tgt_r = [];
        
        this.chart = null;
        
        // Bindings
        this.startTest = this.startTest.bind(this);
        this.stopTest = this.stopTest.bind(this);
        this.processLog = this.processLog.bind(this);
        
        // Listeners
        if (this.btnTestPid) {
            this.btnTestPid.addEventListener('click', this.startTest);
        }
        
        if (this.btnShowChart) {
            this.btnShowChart.addEventListener('click', () => {
                this.modal.style.display = 'block';
                this.renderChart();
            });
        }
        
        if (this.btnCloseModal) {
            this.btnCloseModal.addEventListener('click', () => {
                this.modal.style.display = 'none';
            });
        }
        
        window.addEventListener('click', (event) => {
            if (this.modal && event.target === this.modal) {
                this.modal.style.display = 'none';
            }
        });
        
        // Disable inputs when system is running or testing
        this.socketMgr.onSystemStateChange((isRunning) => {
            const disabled = isRunning || this.isTesting;
            if (this.inputRpmL) this.inputRpmL.disabled = disabled;
            if (this.inputRpmR) this.inputRpmR.disabled = disabled;
            if (this.btnTestPid) this.btnTestPid.disabled = disabled;
            
            // If system stops externally (e.g. Stop button), and we were testing
            if (!isRunning && this.isTesting) {
                this.stopTest();
            }
        });
    }
    
    startTest() {
        if (this.isTesting) return;
        
        this.isTesting = true;
        this.isRecording = true;
        
        // Clear old data
        this.timestamps = [];
        this.rpm_act_l = [];
        this.rpm_act_r = [];
        this.rpm_tgt_l = [];
        this.rpm_tgt_r = [];
        
        // Tell backend we are formally running so other UI components disable appropriately
        this.socketMgr.isRobotRunning = true;
        this.socketMgr.notifyStateChange();
        
        console.log("PID Test started. Recording data...");
        
        // Wait 1s, then send the target RPMs
        this.testTimer = setTimeout(() => {
            const rpmL = Math.min(Math.max(parseFloat(this.inputRpmL.value) || 0, -333), 333);
            const rpmR = Math.min(Math.max(parseFloat(this.inputRpmR.value) || 0, -333), 333);
            
            const payload = {
                cmd: 'test_pid',
                rpm_l: rpmL,
                rpm_r: rpmR
            };
            
            this.socketMgr.sendUdp(payload);
            console.log("Sent test_pid command: ", payload);
        }, 1000);
    }
    
    stopTest() {
        if (this.testTimer) {
            clearTimeout(this.testTimer);
            this.testTimer = null;
        }
        
        // MCU has already received 'stop' from socketMgr.stopSystem() (which triggers this stopTest via onSystemStateChange)
        // We wait 1 second to capture the spin down, then stop recording and show chart
        setTimeout(() => {
            this.isRecording = false;
            this.isTesting = false;
            
            console.log("PID Test finished.");
            
            // Normalize timestamps so the graph starts at 0s
            if (this.timestamps.length > 0) {
                const t0 = this.timestamps[0];
                for (let i = 0; i < this.timestamps.length; i++) {
                    this.timestamps[i] = this.timestamps[i] - t0;
                }
            }
            
            // Calculate and display metrics
            const metricsL = this.calculateMetrics(this.timestamps, this.rpm_act_l, this.rpm_tgt_l);
            const metricsR = this.calculateMetrics(this.timestamps, this.rpm_act_r, this.rpm_tgt_r);
            
            const osL = document.getElementById('metric-os-l');
            const tsL = document.getElementById('metric-ts-l');
            const sseL = document.getElementById('metric-sse-l');
            if (osL) osL.textContent = metricsL.os;
            if (tsL) tsL.textContent = metricsL.ts;
            if (sseL) sseL.textContent = metricsL.sse;
            
            const osR = document.getElementById('metric-os-r');
            const tsR = document.getElementById('metric-ts-r');
            const sseR = document.getElementById('metric-sse-r');
            if (osR) osR.textContent = metricsR.os;
            if (tsR) tsR.textContent = metricsR.ts;
            if (sseR) sseR.textContent = metricsR.sse;
            
            // Auto open chart
            if (this.modal) {
                this.modal.style.display = 'block';
            }
            this.renderChart();
            
        }, 1000);
    }
    
    calculateMetrics(timestamps, act, tgt) {
        if (timestamps.length === 0) return { os: 'N/A', ts: 'N/A', sse: 'N/A' };
        
        // Find start of step (where target > 0 or < 0, ignoring 0)
        let idx_start = -1;
        for (let i = 0; i < tgt.length; i++) {
            if (tgt[i] !== 0) {
                idx_start = i;
                break;
            }
        }
        
        if (idx_start === -1) return { os: 'N/A', ts: 'N/A', sse: 'N/A' }; // No step found
        
        const target_val = tgt[idx_start];
        
        // Find end of step (where target returns to 0)
        let idx_stop = tgt.length - 1;
        for (let i = idx_start; i < tgt.length; i++) {
            if (tgt[i] === 0) {
                idx_stop = i - 1;
                break;
            }
        }
        
        if (idx_stop < idx_start) return { os: 'N/A', ts: 'N/A', sse: 'N/A' };
        
        // Steady State Value (Average of last 500ms of the step, or last 20% of samples if short)
        const duration = timestamps[idx_stop] - timestamps[idx_start];
        let ss_start_time = timestamps[idx_stop] - 500;
        if (duration < 1000) {
            ss_start_time = timestamps[idx_start] + (duration * 0.8);
        }
        
        let sum = 0;
        let count = 0;
        for (let i = idx_stop; i >= idx_start; i--) {
            if (timestamps[i] < ss_start_time) break;
            sum += act[i];
            count++;
        }
        const y_ss = count > 0 ? sum / count : act[idx_stop];
        const sse = Math.abs(target_val - y_ss);
        
        let sse_str = '0.00%';
        if (target_val !== 0) {
            sse_str = ((sse / Math.abs(target_val)) * 100).toFixed(2) + '%';
        } else {
            sse_str = sse.toFixed(2);
        }
        
        // Overshoot
        let y_max = act[idx_start];
        let y_min = act[idx_start];
        for (let i = idx_start; i <= idx_stop; i++) {
            if (act[i] > y_max) y_max = act[i];
            if (act[i] < y_min) y_min = act[i];
        }
        
        let os = 0;
        let os_str = '0.00%';
        if (target_val > 0) {
            os = Math.max(0, y_max - target_val);
            os_str = ((os / target_val) * 100).toFixed(2) + '%';
        } else if (target_val < 0) {
            os = Math.max(0, Math.abs(y_min) - Math.abs(target_val)); // If target is negative
            os_str = ((os / Math.abs(target_val)) * 100).toFixed(2) + '%';
        }
        
        // Settling Time (5% band)
        const band_upper = target_val > 0 ? target_val * 1.05 : target_val * 0.95;
        const band_lower = target_val > 0 ? target_val * 0.95 : target_val * 1.05;
        
        let idx_settled = -1;
        for (let i = idx_stop; i >= idx_start; i--) {
            // Check if out of bounds
            if (act[i] > Math.max(band_upper, band_lower) || act[i] < Math.min(band_upper, band_lower)) {
                if (i < idx_stop) {
                    idx_settled = i + 1;
                }
                break;
            }
        }
        
        let ts = 'N/A';
        // Ensure the final value is actually within the band
        if (act[idx_stop] <= Math.max(band_upper, band_lower) && act[idx_stop] >= Math.min(band_upper, band_lower)) {
            if (idx_settled !== -1) {
                ts = (timestamps[idx_settled] - timestamps[idx_start]).toFixed(0) + ' ms';
            } else {
                ts = '0 ms'; // Remained within band entirely
            }
        } else {
            ts = 'Did not settle';
        }
        
        return {
            os: os_str,
            ts: ts,
            sse: sse_str
        };
    }
    
    processLog(logEntry) {
        if (!this.isRecording) return;
        
        try {
            const obj = JSON.parse(logEntry.data);
            
            if (obj.rpm_act && obj.rpm_tgt) {
                this.timestamps.push(obj.ts); // ESP32 sends millis()
                
                this.rpm_act_l.push(obj.rpm_act[0]);
                this.rpm_act_r.push(obj.rpm_act[1]);
                
                this.rpm_tgt_l.push(obj.rpm_tgt[0]);
                this.rpm_tgt_r.push(obj.rpm_tgt[1]);
            }
        } catch (e) {
            // Ignore parse errors
        }
    }
    
    renderChart() {
        const ctx = document.getElementById('pid-test-chart');
        if (!ctx) return;
        
        if (this.chart) {
            this.chart.destroy();
        }
        
        this.chart = new Chart(ctx.getContext('2d'), {
            type: 'line',
            data: {
                labels: this.timestamps.map(t => (t / 1000).toFixed(1)),
                datasets: [
                    {
                        label: 'Target L',
                        data: this.rpm_tgt_l,
                        borderColor: '#8b949e',
                        borderDash: [5, 5],
                        fill: false,
                        tension: 0.1,
                        pointRadius: 0,
                        borderWidth: 2
                    },
                    {
                        label: 'Actual L',
                        data: this.rpm_act_l,
                        borderColor: '#2f81f7',
                        fill: false,
                        tension: 0.1,
                        pointRadius: 0,
                        borderWidth: 2
                    },
                    {
                        label: 'Target R',
                        data: this.rpm_tgt_r,
                        borderColor: '#8b949e',
                        borderDash: [2, 2],
                        fill: false,
                        tension: 0.1,
                        pointRadius: 0,
                        borderWidth: 2
                    },
                    {
                        label: 'Actual R',
                        data: this.rpm_act_r,
                        borderColor: '#3fb950',
                        fill: false,
                        tension: 0.1,
                        pointRadius: 0,
                        borderWidth: 2
                    }
                ]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                animation: false,
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
                        display: true,
                        title: { display: true, text: 'Time (s)', color: '#8b949e' },
                        ticks: { color: '#8b949e', maxTicksLimit: 20 },
                        grid: { color: '#30363d' }
                    },
                    y: {
                        display: true,
                        title: { display: true, text: 'RPM', color: '#8b949e' },
                        ticks: { color: '#8b949e' },
                        grid: { color: '#30363d' }
                    }
                }
            }
        });
    }
}
