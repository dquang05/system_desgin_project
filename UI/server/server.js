const express = require('express');
const http = require('http');
const { Server } = require('socket.io');
const dgram = require('dgram');
const path = require('path');

const app = express();
const server = http.createServer(app);
const io = new Server(server);

// Cấu hình cổng mạng dựa trên hardware_analyzer
const UDP_PORT = process.env.UDP_PORT || 54321;
const HTTP_PORT = process.env.PORT || 3000;

let udpSocket = null;
let isReconnecting = false;
let reconnectTimer = null;
let esp32Ip = null;

app.use(express.static(path.join(__dirname, 'public')));

/**
 * Cleans up RAM and comprehensively releases the UDP Socket.
 * Closes the socket to prevent memory leaks and "Address already in use" errors.
 */
function cleanupResources() {
    if (udpSocket) {
        try {
            udpSocket.close();
        } catch (err) {
            console.error('Error closing UDP socket:', err.message);
        }
        udpSocket = null;
    }
    
    if (reconnectTimer) {
        clearTimeout(reconnectTimer);
        reconnectTimer = null;
    }
    
    isReconnecting = false;
    console.log("Resources cleaned up");
}

/**
 * Initializes and starts the UDP server for receiving telemetry data.
 * Implements self-recovery in case of errors.
 */
function startUdpServer() {
    if (udpSocket) return; // Prevent duplicate socket creation

    try {
        udpSocket = dgram.createSocket('udp4');

        udpSocket.on('error', (err) => {
            console.error(`UDP server error:\n${err.stack}`);
            
            // Handle EADDRINUSE specifically
            if (err.code === 'EADDRINUSE') {
                console.error(`Port ${UDP_PORT} is already in use.`);
            }

            cleanupResources();
            io.to('log_subscribers').emit('status', 'reconnecting'); 
            
            // Self-recovery: Attempt to reconnect after 3 seconds
            if (!isReconnecting) {
                isReconnecting = true;
                reconnectTimer = setTimeout(() => {
                    console.log("Attempting to reconnect...");
                    isReconnecting = false;
                    startUdpServer();
                }, 3000);
            }
        });

        // Backend event loop: Process asynchronous network I/O
        udpSocket.on('message', (msg, rinfo) => {
            // Store ESP32 IP to send tuning commands later
            if (rinfo && rinfo.address) {
                esp32Ip = rinfo.address;
            }
            
            try {
                // Parse log, send realtime to UI via WebSockets
                const logData = msg.toString('utf8');
                const fromAddress = rinfo ? `${rinfo.address}:${rinfo.port}` : 'unknown';
                
                io.to('log_subscribers').emit('log', { 
                    timestamp: new Date().toLocaleTimeString(), 
                    data: logData, 
                    from: fromAddress
                });
            } catch (err) {
                console.error("Malformed packet received", err.message);
            }
        });

        udpSocket.on('listening', () => {
            const address = udpSocket.address();
            console.log(`UDP server listening ${address.address}:${address.port}`);
            isReconnecting = false;
            io.to('log_subscribers').emit('status', 'listening');
        });

        udpSocket.bind(UDP_PORT);
    } catch (err) {
        console.error("Failed to create UDP socket", err.message);
        cleanupResources();
        io.to('log_subscribers').emit('status', 'reconnecting');
    }
}

io.on('connection', (socket) => {
    console.log('Frontend connected');
    
    // By default, the frontend is disconnected from UDP when first visiting the web
    socket.emit('status', 'disconnected');

    /**
     * Listen for tuning commands from the frontend and forward them to ESP32 via UDP.
     */
    socket.on('send_tune', (data) => {
        if (esp32Ip && udpSocket) {
            try {
                const payload = JSON.stringify(data);
                udpSocket.send(payload, 54322, esp32Ip, (err) => {
                    if (err) {
                        console.error('Failed to send tuning packet:', err);
                        socket.emit('error_msg', 'Failed to send tuning packet via UDP.');
                    } else {
                        console.log(`Sent tuning packet to ${esp32Ip}:54322`, payload);
                    }
                });
            } catch (err) {
                console.error('Error stringifying tuning payload', err);
                socket.emit('error_msg', 'Error processing tuning data.');
            }
        } else {
            console.warn('Cannot send tuning packet: ESP32 IP not known or UDP socket not open.');
            // Send error notification back to the frontend
            socket.emit('error_msg', 'Cannot send tuning command. No connection to ESP32 yet (Unknown IP).');
        }
    });

    socket.on('send_manual_drive', (data) => {
        if (esp32Ip && udpSocket) {
            try {
                const payload = JSON.stringify(data);
                udpSocket.send(payload, 54322, esp32Ip, (err) => {
                    if (err) {
                        console.error('Failed to send manual drive packet:', err);
                        socket.emit('error_msg', 'Failed to send manual drive packet via UDP.');
                    } else {
                        console.log(`Sent manual drive packet to ${esp32Ip}:54322`, payload);
                    }
                });
            } catch (err) {
                console.error('Error stringifying manual drive payload', err);
                socket.emit('error_msg', 'Error processing manual drive data.');
            }
        } else {
            console.warn('Cannot send manual drive packet: ESP32 IP not known or UDP socket not open.');
            socket.emit('error_msg', 'Cannot send manual drive command. No connection to ESP32 yet (Unknown IP).');
        }
    });

    socket.on('command', (cmd) => {
        if (cmd === 'connect') {
            socket.join('log_subscribers');
            if (udpSocket && !isReconnecting) {
                socket.emit('status', 'listening');
            } else if (isReconnecting) {
                socket.emit('status', 'reconnecting');
            } else {
                startUdpServer();
            }
        } else if (cmd === 'disconnect') {
            socket.leave('log_subscribers');
            socket.emit('status', 'disconnected');
        }
    });

    socket.on('disconnect', () => {
        console.log('Frontend disconnected');
    });
});

server.listen(HTTP_PORT, () => {
    console.log(`Web interface running at http://localhost:${HTTP_PORT}`);
    // Always run UDP server in the background instead of waiting for frontend call
    startUdpServer();
});

// Catch OS events to completely clean up RAM when process terminates
process.on('SIGINT', () => {
    cleanupResources();
    process.exit();
});
process.on('SIGTERM', () => {
    cleanupResources();
    process.exit();
});
