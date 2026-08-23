export class DataRecordingManager {
    constructor(socketMgr) {
        this.socketMgr = socketMgr;
        
        // Data arrays
        this.isRecording = false;
        this.timestamps = [];
        this.e2 = [];
        this.rpm_l = [];
        this.rpm_r = [];
        this.v_l = [];
        this.v_r = [];

        // Chart instances
        this.e2Chart = null;
        this.rpmChart = null;
        this.vChart = null;

        // Constants
        this.WHEEL_RADIUS_MM = 40.0;

        // Bind methods
        this.onSystemStateChange = this.onSystemStateChange.bind(this);
        this.processLog = this.processLog.bind(this);

        // Listeners
        this.socketMgr.onSystemStateChange(this.onSystemStateChange);
    }

    onSystemStateChange(isRobotRunning) {
        if (isRobotRunning) {
            // Start recording: clear old data
            this.isRecording = true;
            this.timestamps = [];
            this.e2 = [];
            this.rpm_l = [];
            this.rpm_r = [];
            this.v_l = [];
            this.v_r = [];
            console.log("DataRecordingManager: Started recording");
        } else {
            // Stop recording: render charts
            if (this.isRecording) {
                this.isRecording = false;
                console.log(`DataRecordingManager: Stopped recording. Captured ${this.timestamps.length} data points.`);
                this.renderCharts();
            }
        }
    }

    processLog(logEntry) {
        if (!this.isRecording) return;

        try {
            const obj = JSON.parse(logEntry.data);
            
            // Check if required fields exist
            if (obj.e2 !== undefined && obj.rpm_act !== undefined) {
                // Record timestamp (convert from us to ms for better readability, or just use as is)
                this.timestamps.push(obj.ts / 1000); 
                
                // Record e2
                this.e2.push(obj.e2);
                
                // Record RPM
                const rpmLeft = obj.rpm_act[0];
                const rpmRight = obj.rpm_act[1];
                this.rpm_l.push(rpmLeft);
                this.rpm_r.push(rpmRight);
                
                // Calculate and record Velocity (v = rpm * 2 * pi * r / 60)
                const vLeft = rpmLeft * 2 * Math.PI * this.WHEEL_RADIUS_MM / 60;
                const vRight = rpmRight * 2 * Math.PI * this.WHEEL_RADIUS_MM / 60;
                this.v_l.push(vLeft);
                this.v_r.push(vRight);
            }
        } catch (e) {
            // Ignore parse errors
        }
    }

    renderCharts() {
        this.renderE2Chart();
        this.renderRpmChart();
        this.renderVelocityChart();
    }

    // Helper for common chart options
    getCommonOptions(yAxisTitle) {
        return {
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
                    title: { display: true, text: 'Time (ms)', color: '#8b949e' },
                    ticks: { color: '#8b949e', maxTicksLimit: 20 },
                    grid: { color: '#30363d' }
                },
                y: {
                    display: true,
                    title: { display: true, text: yAxisTitle, color: '#8b949e' },
                    ticks: { color: '#8b949e' },
                    grid: { color: '#30363d' }
                }
            }
        };
    }

    renderE2Chart() {
        const ctx = document.getElementById('e2-chart').getContext('2d');
        if (this.e2Chart) {
            this.e2Chart.destroy();
        }
        this.e2Chart = new Chart(ctx, {
            type: 'line',
            data: {
                labels: this.timestamps,
                datasets: [
                    {
                        label: 'e2 (Lateral Error)',
                        data: this.e2,
                        borderColor: '#f85149',
                        fill: false,
                        tension: 0.1,
                        pointRadius: 0,
                        borderWidth: 2
                    }
                ]
            },
            options: this.getCommonOptions('Error e2')
        });
    }

    renderRpmChart() {
        const ctx = document.getElementById('rpm-analysis-chart').getContext('2d');
        if (this.rpmChart) {
            this.rpmChart.destroy();
        }
        this.rpmChart = new Chart(ctx, {
            type: 'line',
            data: {
                labels: this.timestamps,
                datasets: [
                    {
                        label: 'Left RPM',
                        data: this.rpm_l,
                        borderColor: '#2f81f7',
                        fill: false,
                        tension: 0.1,
                        pointRadius: 0,
                        borderWidth: 2
                    },
                    {
                        label: 'Right RPM',
                        data: this.rpm_r,
                        borderColor: '#3fb950',
                        fill: false,
                        tension: 0.1,
                        pointRadius: 0,
                        borderWidth: 2
                    }
                ]
            },
            options: this.getCommonOptions('RPM')
        });
    }

    renderVelocityChart() {
        const ctx = document.getElementById('velocity-chart').getContext('2d');
        if (this.vChart) {
            this.vChart.destroy();
        }
        this.vChart = new Chart(ctx, {
            type: 'line',
            data: {
                labels: this.timestamps,
                datasets: [
                    {
                        label: 'Left Velocity (mm/s)',
                        data: this.v_l,
                        borderColor: '#2f81f7',
                        fill: false,
                        tension: 0.1,
                        pointRadius: 0,
                        borderWidth: 2
                    },
                    {
                        label: 'Right Velocity (mm/s)',
                        data: this.v_r,
                        borderColor: '#3fb950',
                        fill: false,
                        tension: 0.1,
                        pointRadius: 0,
                        borderWidth: 2
                    }
                ]
            },
            options: this.getCommonOptions('Velocity (mm/s)')
        });
    }
}
