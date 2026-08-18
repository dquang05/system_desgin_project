export class SteadyStateErrorCalculator {
    constructor() {
        this.errorHistory = [];
        this.previousTarget = null;
        this.steadyStateError = 0;
        this.TIME_WINDOW_MS = 20000; 
    }

    update(currentTarget, currentValue) {
        const now = Date.now();
        const currentError = currentTarget - currentValue;

        if (this.previousTarget !== null && Math.abs(currentTarget - this.previousTarget) > 0.001) {
            this.errorHistory = [];
        }
        this.previousTarget = currentTarget;

        this.errorHistory.push({
            time: now,
            error: currentError
        });

        while (this.errorHistory.length > 0 && (now - this.errorHistory[0].time) > this.TIME_WINDOW_MS) {
            this.errorHistory.shift(); 
        }

        if (this.errorHistory.length === 0) {
            this.steadyStateError = 0;
        } else {
            const sum = this.errorHistory.reduce((acc, curr) => acc + curr.error, 0);
            this.steadyStateError = sum / this.errorHistory.length;
        }

        return this.steadyStateError;
    }
}
