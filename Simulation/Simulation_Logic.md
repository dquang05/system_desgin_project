# Tài liệu Mô tả Logic Mô phỏng Hệ thống Xe Tự hành (Line Tracking)

Hệ thống mô phỏng một xe tự hành bám đường (line tracking) dựa trên mô hình động học vi sai (differential drive). Quá trình điều khiển được chia làm hai vòng lặp lồng nhau (cascade control): vòng ngoài (Outer Loop) điều hướng xe để bám quỹ đạo dựa trên cảm biến mô phỏng; vòng trong (Inner Loop) điều khiển tốc độ thực tế của từng động cơ trái/phải thông qua các hàm truyền.

Dưới đây là mô tả chi tiết logic và luồng hoạt động của hệ thống, cùng với vai trò của từng file tham gia vào các bước:

## 1. Khởi tạo hệ thống và Sa bàn
**Script chính:** [`main.m`](file:///d:/Projects/system_desgin_project/Simulation/main.m)

- **Thông số cơ học:**
  - Khoảng cách giữa 2 bánh xe (Track width): `b = 246 mm`
  - Khoảng cách từ tâm mảng cảm biến (dò line) đến trục bánh xe: `d = 176 mm`
  - Bán kính bánh xe: `r = 40 mm`
- **Khởi tạo vị trí ban đầu:** 
  - Tâm xe ban đầu tại `(xM, yM) = (0, 0)`, góc lệch `phi = pi`.
  - Tọa độ tâm mảng cảm biến dò line: `xC, yC` được xác định theo tọa độ tâm xe và khoảng cách `d`.
- **Vẽ sa bàn:** Gọi script phụ trợ [`saban.m`](file:///d:/Projects/system_desgin_project/Simulation/saban.m) để phác họa bản đồ môi trường di chuyển với các điểm mốc và giới hạn tọa độ.

---

## 2. Vòng lặp Mô phỏng (Simulation Loop)
Quá trình chạy trong 200 bước (tần số lấy mẫu chung `tsampsys = 0.2s`, khoảng 40 giây thực thi). Tại mỗi chu kỳ, hệ thống thực hiện tuần tự:

### 2.1 Tính toán sai số bám line (Đọc cảm biến)
**Hàm thực thi:** [`get_e2.m`](file:///d:/Projects/system_desgin_project/Simulation/get_e2.m)

- Trạng thái đường đi của xe được chia thành **8 phân đoạn (lines)** dựa trên tọa độ di chuyển dọc trục `xC`.
- **Rẽ nhánh (Phân đoạn 1):** Khi `xC < -2000`, hàm gọi `questdlg` hiển thị popup để chọn lộ trình rẽ: **"Red" (Đỏ)** hoặc **"Blue" (Xanh)**. Biến `color` (1 hoặc 2) lưu trữ lựa chọn sẽ định tuyến cho các phương trình quỹ đạo ở phân đoạn 5, 6, 7 và 8.
- **Tính toán sai số lệch đường (`e`):** Hàm dùng phương trình hình học của đường (thẳng/cong) để chiếu tìm vị trí tâm đường lý tưởng `(xR, yR)`. Từ đó tính toán khoảng cách vuông góc `e` (bằng mm) đại diện cho độ lệch giữa mảng cảm biến xe và trung tâm vạch kẻ lý tưởng.

### 2.2 Bộ điều khiển hướng (Outer Loop PID)
**Hàm thực thi:** [`tracking_line.m`](file:///d:/Projects/system_desgin_project/Simulation/tracking_line.m)

- **Đầu vào:** Sai số vị trí `e` ở chu kỳ hiện tại.
- Bộ điều khiển sử dụng cấu trúc **PID vị trí** (ví dụ: `Kp = 0.045`, `Ki = 0.0`, `Kd = 0.001`) để tính ra tín hiệu điều khiển `w` - vận tốc góc bẻ lái để đưa xe về đúng quỹ đạo.
- Kết hợp với tốc độ chạy tới danh định `vR = 500 mm/s`, bộ điều khiển tổng hợp và trả về **vận tốc góc mục tiêu (RPM)** cho 2 bánh xe trái/phải (`wl`, `wr`).

### 2.3 Mô phỏng điều khiển động cơ (Inner Loop PID)
**Hàm thực thi:** [`pid_motor_left.m`](file:///d:/Projects/system_desgin_project/Simulation/pid_motor_left.m), [`pid_motor_right.m`](file:///d:/Projects/system_desgin_project/Simulation/pid_motor_right.m) và các hàm truyền như [`motor1_tf.m`](file:///d:/Projects/system_desgin_project/Simulation/motor1_tf.m).

Động cơ thực tế có quán tính và độ trễ, do đó phần này được lấy mẫu cao hơn (vòng lặp con với chu kỳ `tsamppid = 0.03s`).
- **PID Tốc độ:** Mỗi động cơ chạy bộ điều khiển PI riêng biệt (ví dụ: `kp1 = 1.0673`, `ki1 = 13.948`) nhằm điều chỉnh sai lệch giữa tốc độ đang chạy và tốc độ yêu cầu (`wl`, `wr`). Đầu ra của PID là giá trị băm xung PWM (`ul`, `ur` trong giới hạn 0-255).
- **Hàm truyền:** Cấp PWM vào hàm truyền bậc 1 của động cơ (`K = 12.05`, `a = 10.64`).
- Hệ thống giải phương trình vi phân liên tục qua lệnh `ode45` trong MATLAB để tìm ra **vận tốc tịnh tiến thực tế** `vl`, `vr` (mm/s) đáp ứng ra tại bánh xe.

### 2.4 Cập nhật động học hệ thống (Kinematics)
**Hàm thực thi:** [`dynamic_tf.m`](file:///d:/Projects/system_desgin_project/Simulation/dynamic_tf.m) tích hợp trong `main.m`.

- Từ vận tốc thực tế của hai bánh `vl`, `vr`, xác định vận tốc tịnh tiến hệ thống: $v_{input} = \frac{vl + vr}{2}$ và vận tốc xoay thân xe $w_{input} = \frac{vr - vl}{b}$.
- Đưa qua phương trình vi phân động học robot vi sai:
  - $\dot{x} = v \cos(\phi)$
  - $\dot{y} = v \sin(\phi)$
  - $\dot{\phi} = w$
- Giải qua `ode45` để tìm ra tọa độ không gian mới `(xM, yM)` và góc hướng mũi xe `phi`. Cập nhật tịnh tiến mảng cảm biến `(xC, yC)` lên phía trước.
- Xóa frame cũ và vẽ đè hình ảnh xe mới bằng các hàm `plot` và lệnh `drawnow`, sinh ra hoạt ảnh xe di chuyển mượt mà trên môi trường giả lập.

---

## 3. Phân tích Đáp ứng Động cơ (Hỗ trợ)
**Script phụ trợ:** [`motor_response.m`](file:///d:/Projects/system_desgin_project/Simulation/motor_response.m)

- Đóng vai trò là kịch bản mô phỏng tĩnh để đánh giá thiết kế vòng điều khiển vận tốc.
- Sinh đồ thị hàm Step (Step Response) cho vòng kín chứa Động cơ + Bộ PID vận tốc.
- Phân tích và trích xuất các chỉ số chất lượng: **Overshoot** (Độ vọt lố), **Settling Time** (Thời gian xác lập), và **Steady-State Error** (Sai số xác lập) lưu vào tập tin báo cáo `motor_simulation_results.txt`.

---

## 4. Tổng hợp Kết quả và Báo cáo (Kết thúc mô phỏng)
Sau 200 chu kỳ chạy, cuối file `main.m` thực hiện lưu vết và phân tích:
1. **Biểu đồ trực quan:**
   - Đồ thị sai số cảm biến dò line theo thời gian (quan sát độ lặp, lắc thân xe).
   - Đồ thị tốc độ góc mong muốn của 2 bánh xe (RPM).
   - Đồ thị tốc độ dài thực tế của động cơ sau xử lý độ trễ (mm/s).
2. **Xuất báo cáo văn bản:** Trích xuất kết quả ra tệp [`main_simulation_results.txt`](file:///d:/Projects/system_desgin_project/Simulation/main_simulation_results.txt) gồm:
   - Sai số hiệu dụng (RMS Error) và sai số tích phân (IAE).
   - Số lần cắt ngang tâm đường (Zero-crossings) cảnh báo xe bị lắc.
   - Vị trí và khoảng thời gian xe bị overshoot lệch khỏi quỹ đạo xa nhất.
