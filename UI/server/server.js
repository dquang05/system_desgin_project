const express = require('express');
const http = require('http');
const { Server } = require('socket.io');
const dgram = require('dgram');
const path = require('path');

const app = express();
const server = http.createServer(app);
const io = new Server(server);

// Cấu hình cổng mạng dựa trên hardware_analyzer
const UDP_PORT = 8080;
const HTTP_PORT = 3000;

let udp_socket = null;
let is_reconnecting = false;
let reconnect_timer = null;

app.use(express.static(path.join(__dirname, 'public')));

// Hàm dọn dẹp RAM & Giải phóng Socket triệt để
function cleanup_resources() {
    if (udp_socket) {
        try {
            // Đóng triệt để socket UDP để tránh rò rỉ và Address already in use
            udp_socket.close();
        } catch (err) {
            console.error('Error closing UDP socket:', err.message);
        }
        udp_socket = null;
    }
    
    if (reconnect_timer) {
        clearTimeout(reconnect_timer);
        reconnect_timer = null;
    }
    
    // Dọn dẹp biến toàn cục để phiên sau chạy sạch sẽ
    is_reconnecting = false;
    console.log("Resources cleaned up");
}

function start_udp_server() {
    if (udp_socket) return; // Không tạo trùng socket

    try {
        udp_socket = dgram.createSocket('udp4');

        udp_socket.on('error', (err) => {
            console.error(`UDP server error:\n${err.stack}`);
            cleanup_resources();
            io.emit('status', 'reconnecting'); 
            
            // Self-recovery: Thử kết nối lại sau 3 giây
            if (!is_reconnecting) {
                is_reconnecting = true;
                reconnect_timer = setTimeout(() => {
                    console.log("Attempting to reconnect...");
                    is_reconnecting = false;
                    start_udp_server();
                }, 3000);
            }
        });

        // Backend event loop: Xử lý I/O mạng phi đồng bộ
        udp_socket.on('message', (msg, rinfo) => {
            try {
                // Parse log, gửi realtime tới UI qua WebSockets
                const logData = msg.toString('utf8');
                io.emit('log', { 
                    timestamp: new Date().toLocaleTimeString(), 
                    data: logData, 
                    from: `${rinfo.address}:${rinfo.port}` 
                });
            } catch (err) {
                console.error("Malformed packet received", err.message);
            }
        });

        udp_socket.on('listening', () => {
            const address = udp_socket.address();
            console.log(`UDP server listening ${address.address}:${address.port}`);
            is_reconnecting = false;
            io.emit('status', 'listening');
        });

        udp_socket.bind(UDP_PORT);
    } catch (err) {
        console.error("Failed to create UDP socket", err.message);
        cleanup_resources();
        io.emit('status', 'reconnecting');
    }
}

io.on('connection', (socket) => {
    console.log('Frontend connected');
    
    // Sync trạng thái ban đầu
    if (udp_socket) {
        socket.emit('status', 'listening');
    } else {
        socket.emit('status', 'disconnected');
    }

    socket.on('command', (cmd) => {
        if (cmd === 'connect') {
            start_udp_server();
        } else if (cmd === 'disconnect') {
            cleanup_resources();
            io.emit('status', 'disconnected');
        }
    });

    socket.on('disconnect', () => {
        console.log('Frontend disconnected');
        // Không bắt buộc phải đóng Socket UDP nếu UI refresh, 
        // nhưng nếu muốn đóng theo luồng kết nối thì mở comment dưới đây:
        // if (io.engine.clientsCount === 0) cleanup_resources();
    });
});

server.listen(HTTP_PORT, () => {
    console.log(`Web interface running at http://localhost:${HTTP_PORT}`);
});

// Bắt sự kiện OS để dọn dẹp RAM triệt để khi tắt process
process.on('SIGINT', () => {
    cleanup_resources();
    process.exit();
});
process.on('SIGTERM', () => {
    cleanup_resources();
    process.exit();
});
