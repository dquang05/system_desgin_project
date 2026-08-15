/**
 * Manages WebSocket connections and event listeners for real-time telemetry.
 */
export class SocketManager {
    constructor() {
        this.socket = io();
        this.statusListeners = [];
        this.logListeners = [];
        this.errorMsgListeners = [];
        this.systemStateListeners = [];
        this.isRobotRunning = false;
        
        this.socket.on('status', (state) => {
            this.statusListeners.forEach(cb => cb(state));
        });

        this.socket.on('log', (logEntry) => {
            this.logListeners.forEach(cb => cb(logEntry));
        });

        this.socket.on('error_msg', (msg) => {
            this.errorMsgListeners.forEach(cb => cb(msg));
        });
    }

    /**
     * Registers a callback for connection status changes.
     * @param {function(string)} callback - The callback function.
     */
    onStatusChange(callback) {
        this.statusListeners.push(callback);
    }

    /**
     * Registers a callback for receiving log entries.
     * @param {function(Object)} callback - The callback function handling log entries.
     */
    onLogReceived(callback) {
        this.logListeners.push(callback);
    }

    /**
     * Registers a callback for receiving error messages from the backend.
     * @param {function(string)} callback - The callback function.
     */
    onErrorMsg(callback) {
        this.errorMsgListeners.push(callback);
    }

    /**
     * Emits a connect command to start the UDP listener on the backend.
     */
    connect() {
        this.socket.emit('command', 'connect');
    }

    /**
     * Emits a disconnect command to stop receiving logs.
     */
    disconnect() {
        this.socket.emit('command', 'disconnect');
    }

    /**
     * Emits a generic UDP payload to the backend to be forwarded to ESP32.
     * @param {Object} payload - The JSON payload to send.
     */
    sendUdp(payload) {
        this.socket.emit('send_udp', payload);
    }

    /**
     * Starts the system and notifies listeners.
     */
    startSystem() {
        this.isRobotRunning = true;
        this.sendUdp({ cmd: 'start' });
        this.notifyStateChange();
    }

    /**
     * Stops the system and notifies listeners.
     */
    stopSystem() {
        this.isRobotRunning = false;
        this.sendUdp({ cmd: 'stop' });
        this.notifyStateChange();
    }

    /**
     * Registers a callback for system state changes (start/stop).
     * @param {function(boolean)} callback - The callback receiving the isRobotRunning state.
     */
    onSystemStateChange(callback) {
        if (!this.systemStateListeners) {
            this.systemStateListeners = [];
        }
        this.systemStateListeners.push(callback);
    }

    /**
     * Notifies listeners of the current system state.
     */
    notifyStateChange() {
        if (this.systemStateListeners) {
            this.systemStateListeners.forEach(cb => cb(this.isRobotRunning));
        }
    }
}
