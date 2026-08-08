/**
 * Manages WebSocket connections and event listeners for real-time telemetry.
 */
export class SocketManager {
    constructor() {
        this.socket = io();
        this.statusListeners = [];
        this.logListeners = [];
        this.errorMsgListeners = [];
        
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
}
