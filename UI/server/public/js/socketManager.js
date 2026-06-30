// socketManager.js

export class SocketManager {
    constructor() {
        this.socket = io();
        this.statusListeners = [];
        this.logListeners = [];
        
        this.socket.on('status', (state) => {
            this.statusListeners.forEach(cb => cb(state));
        });

        this.socket.on('log', (logEntry) => {
            this.logListeners.forEach(cb => cb(logEntry));
        });
    }

    onStatusChange(callback) {
        this.statusListeners.push(callback);
    }

    onLogReceived(callback) {
        this.logListeners.push(callback);
    }

    connect() {
        this.socket.emit('command', 'connect');
    }

    disconnect() {
        this.socket.emit('command', 'disconnect');
    }
}
