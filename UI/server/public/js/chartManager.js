// chartManager.js

export class ChartManager {
    constructor() {
        this.ctx = document.getElementById('adc-chart').getContext('2d');
        this.isActive = false;
        
        // 5 channels based on the telemetry JSON: "adc":[a,b,c,d,e]
        this.maxDataPoints = 100;
        
        // Chart.js instance
        this.chart = new Chart(this.ctx, {
            type: 'line',
            data: {
                labels: [], // Timestamps
                datasets: [
                    { label: 'ADC 1', borderColor: '#f85149', data: [], fill: false, tension: 0.1, pointRadius: 0 },
                    { label: 'ADC 2', borderColor: '#3fb950', data: [], fill: false, tension: 0.1, pointRadius: 0 },
                    { label: 'ADC 3', borderColor: '#d29922', data: [], fill: false, tension: 0.1, pointRadius: 0 },
                    { label: 'ADC 4', borderColor: '#2f81f7', data: [], fill: false, tension: 0.1, pointRadius: 0 },
                    { label: 'ADC 5', borderColor: '#a5d6ff', data: [], fill: false, tension: 0.1, pointRadius: 0 }
                ]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                animation: false, // Turn off animation for performance with high-frequency data
                parsing: false, // Disable internal data parsing for performance
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
                        title: { display: true, text: 'Voltage (mV)', color: '#8b949e' },
                        ticks: { color: '#8b949e' },
                        grid: { color: '#30363d' },
                        suggestedMin: 0,
                        suggestedMax: 3300 // Typical ESP32 ADC max voltage
                    }
                }
            }
        });
        
        // Use a flag to throttle Chart.js updates if data comes in too fast
        this.needsUpdate = false;
        this.updateLoop = this.updateLoop.bind(this);
    }
    
    setActive(active) {
        this.isActive = active;
        if (active && this.needsUpdate) {
            this.chart.update();
            this.needsUpdate = false;
        }
    }
    
    updateLoop() {
        if (this.isActive && this.needsUpdate) {
            this.chart.update();
            this.needsUpdate = false;
        }
        // Throttle updates to ~30fps for the chart
        setTimeout(this.updateLoop, 33);
    }

    startUpdateLoop() {
        this.updateLoop();
    }

    processLog(logEntry) {
        try {
            const obj = JSON.parse(logEntry.data);
            if (obj.adc && Array.isArray(obj.adc) && obj.adc.length === 5) {
                
                const timeStr = logEntry.timestamp;
                this.chart.data.labels.push(timeStr);
                
                // Add data to datasets
                for (let i = 0; i < 5; i++) {
                    this.chart.data.datasets[i].data.push(obj.adc[i]);
                }
                
                // Keep moving window
                if (this.chart.data.labels.length > this.maxDataPoints) {
                    this.chart.data.labels.shift();
                    for (let i = 0; i < 5; i++) {
                        this.chart.data.datasets[i].data.shift();
                    }
                }
                
                this.needsUpdate = true;
            }
        } catch (e) {
            // Ignore non-JSON or improperly formatted logs
        }
    }
}
