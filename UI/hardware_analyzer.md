# Hardware Analysis

Dựa trên mã nguồn C/C++ trong thư mục `hardware_code/`, các thông số mạng được trích xuất như sau:

- **Protocol**: UDP (`SOCK_DGRAM`)
  - Tìm thấy tại `wifi_manager.cpp`, dòng 184: `int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);`
- **Port**: `8080`
  - Tìm thấy tại `WifiTest.cpp`, dòng 59: `wifi.send_log_data("192.168.1.100", 8080, ...)`

Server backend sẽ được thiết lập để lắng nghe UDP Datagram trên cổng `8080`.
