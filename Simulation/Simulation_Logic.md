# Tài liệu Mô tả Logic Mô phỏng Hệ thống Xe Tự hành (Line Tracking)

Hệ thống mô phỏng một xe tự hành bám đường (line tracking) dựa trên mô hình động học vi sai (differential drive). Hệ thống sử dụng bộ điều khiển nhiều vòng (cascade control): vòng ngoài điều khiển hướng bám quỹ đạo dựa trên cảm biến, vòng trong điều khiển tốc độ thực tế của từng động cơ.

Dưới đây là chi tiết toàn bộ logic và luồng hoạt động của hệ thống, kèm theo các file/hàm đảm nhiệm tại từng bước chuyển động:

## 1. Khởi tạo hệ thống và Sa bàn
**Nơi thực hiện:** [`main.m`](file:///d:/Projects/system_desgin_project/Simulation/main.m)

- **Cài đặt thông số cơ khí:** Định nghĩa thiết kế cơ khí của xe.
  - Khoảng cách giữa 2 bánh xe (`b = 246` mm)
  - Khoảng cách từ tâm mảng cảm biến dò line đến trục bánh xe (`d = 176` mm)
  - Bán kính bánh xe (`r = 40` mm)
- **Vị trí ban đầu:** Khởi tạo tọa độ tâm xe (`xM(1)`, `yM(1)`), góc nghiêng ban đầu (`phi(1) = pi`) và tọa độ tâm mảng cảm biến (`xC(1)`, `yC(1)`).
- **Vẽ sa bàn:** Hệ thống gọi hàm [`saban()`](file:///d:/Projects/system_desgin_project/Simulation/saban.m) để vẽ bản đồ di chuyển mô phỏng. Bản đồ bao gồm các đoạn đường thẳng và đường cong, đặc biệt có đoạn rẽ nhánh chọn màu sắc (Đỏ/Xanh).

---

## 2. Vòng lặp Mô phỏng Chính (System Loop)
**Nơi thực hiện:** Vòng lặp `for i = 1:200` trong [`main.m`](file:///d:/Projects/system_desgin_project/Simulation/main.m).
Tần số lấy mẫu chung của cả hệ thống là `tsampsys = 0.2` giây. Mỗi bước lặp, quá trình sau đây sẽ diễn ra:

### Bước 2.1: Tính toán sai số bám line (Đọc cảm biến)
**Hàm thực thi:** [`get_e2()`](file:///d:/Projects/system_desgin_project/Simulation/get_e2.m)
- Xe di chuyển qua các giai đoạn dựa vào tọa độ `xC` của đầu cảm biến (chia làm 8 đoạn, từ `line = 1` đến `line = 8`).
- **Chuyển động rẽ nhánh:** Ở đoạn `line = 1` (khi `xC < -2000`), hệ thống sẽ hiện popup (`questdlg`) để người dùng chọn rẽ nhánh "Red" hoặc "Blue". Các phương trình quỹ đạo rẽ nhánh phía sau (`line` 5, 6, 7, 8) sẽ thay đổi tùy thuộc vào biến `color` này.
- **Tính sai số lệch đường (`e`):** Căn cứ vào vị trí hiện tại và quỹ đạo đường lý tưởng, hàm tính toán khoảng cách lệch vuông góc và trả về sai số lệch tâm `e` (bằng mm) từ tâm mảng cảm biến thực tế đến đường dẫn.

### Bước 2.2: Bộ điều khiển hướng (Outer Loop PID)
**Hàm thực thi:** [`tracking_line()`](file:///d:/Projects/system_desgin_project/Simulation/tracking_line.m)
- **Đầu vào:** Sai số vị trí `e(i)` vừa nhận được từ cảm biến.
- **Xử lý logic chuyển hướng:** Bộ điều khiển PID (với $K_p = 0.045$, $K_i = 0$, $K_d = 0.001$) xử lý sai số `e` để nội suy ra vận tốc góc điều chỉnh `w` cần thiết để kéo xe về lại đường trung tâm.
- **Tính toán truyền động:** Kết hợp với vận tốc xe chạy thẳng được set cố định (`vR = 500` mm/s), hàm quy đổi ra vận tốc góc lý thuyết mong muốn mà 2 động cơ trái/phải cần đạt được (`wl`, `wr`).

### Bước 2.3: Mô phỏng điều khiển động cơ (Inner Loop PID)
**Nơi thực hiện:** Vòng lặp `for j = 1:jj` nằm bên trong vòng lặp chính của [`main.m`](file:///d:/Projects/system_desgin_project/Simulation/main.m).
Vì mô tơ thực tế có độ trễ cơ học nên vòng lặp này lấy mẫu ở tần số cao hơn (`tsamppid = 0.03` giây).
- **Chuyển động Động cơ Trái:** Gọi hàm [`pid_motor_left()`](file:///d:/Projects/system_desgin_project/Simulation/pid_motor_left.m).
  - PID tốc độ ($K_{p1}, K_{i1}, K_{d1}$) tính toán đầu ra tín hiệu PWM `ul` cho motor nhằm bám theo tốc độ mong muốn `wl`. 
  - Quán tính của motor trái được mô phỏng bằng cách lấy hàm truyền [`motor1_tf.m`](file:///d:/Projects/system_desgin_project/Simulation/motor1_tf.m) giải qua phương trình vi phân `ode45`, trả ra kết quả vận tốc dài thực tế `vl` (mm/s).
- **Chuyển động Động cơ Phải:** Gọi hàm [`pid_motor_right()`](file:///d:/Projects/system_desgin_project/Simulation/pid_motor_right.m).
  - Tương tự, PID tốc độ riêng ($K_{p2}, K_{i2}, K_{d2}$) và hàm truyền của motor phải [`motor2_tf.m`](file:///d:/Projects/system_desgin_project/Simulation/motor2_tf.m) được mô phỏng qua `ode45`, trả về vận tốc dài thực tế của bánh phải `vr` (mm/s).

### Bước 2.4: Cập nhật động học của toàn hệ thống (Kinematics Update)
**Nơi thực hiện:** Phải gọi hàm [`dynamic_tf.m`](file:///d:/Projects/system_desgin_project/Simulation/dynamic_tf.m) qua lệnh `ode45` trong [`main.m`](file:///d:/Projects/system_desgin_project/Simulation/main.m).
- Hệ thống tổng hợp lại vận tốc dài đầu vào $v_{input} = (vl + vr)/2$ và vận tốc xoay $w_{input} = (vr - vl)/b$.
- Hệ phương trình vi phân động học (Differential Kinematic Equations):
  - Chuyển động tịnh tiến trục X: $\dot{x} = v \cdot \cos(\phi)$
  - Chuyển động tịnh tiến trục Y: $\dot{y} = v \cdot \sin(\phi)$
  - Chuyển động xoay trục Z: $\dot{\phi} = w$
- Giải hệ phương trình tịnh tiến - xoay này trả về vị trí mới của tâm xe (`xM`, `yM`) và góc hướng mới (`phi`). Cuối cùng, tịnh tiến và cập nhật lại tọa độ mảng cảm biến mới ở phía trước mũi xe (`xC`, `yC`).

### Bước 2.5: Cập nhật đồ họa (Hiển thị chuyển động)
**Nơi thực hiện:** Vẽ hình ở cuối vòng lặp lồng trong [`main.m`](file:///d:/Projects/system_desgin_project/Simulation/main.m).
- Cập nhật frame hình ảnh xe di chuyển (xóa hình ảnh cũ, tịnh tiến thân xe hình chữ nhật, xoay mũi xe theo tọa độ mới tính được).
- Sinh hiệu ứng Animation bằng hàm `drawnow` trên mặt phẳng Sa bàn.

---

## 3. Tổng hợp biểu đồ và Báo cáo (Kết thúc mô phỏng)
**Nơi thực hiện:** Đoạn lệnh cuối cùng trong script [`main.m`](file:///d:/Projects/system_desgin_project/Simulation/main.m).
- Tổng hợp các mảng dữ liệu lịch sử (`t`, `e`, `wl`, `wr`, `vl`, `vr`) để vẽ ra 3 biểu đồ chính hiển thị toàn bộ động thái của quá trình chạy: 
  - Sai số bám line theo thời gian (kiểm chứng độ lắc).
  - Tốc độ xoay (RPM) của bánh xe.
  - Tốc độ dài (mm/s) thực tế của 2 bánh xe.
- Phân tích định lượng và đánh giá: tính giá trị độ lệch chuẩn RMS Error, tổng sai số, số lần đong đưa (vượt 0), trích xuất và chẩn đoán chất lượng PID thông qua file xuất [`main_simulation_results.txt`](file:///d:/Projects/system_desgin_project/Simulation/main_simulation_results.txt).
