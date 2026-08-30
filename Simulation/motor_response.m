% motor_response.m
% Script to analyze the speed response of two motors

clc; clear; close all;

%% 1. Khởi tạo Target (Tốc độ mong muốn)
target_speed = 100; % Thay đổi target speed ở đây (ví dụ: vòng/phút hoặc rad/s)

%% 2. Khai báo Hàm truyền (Transfer Functions)
s = tf('s');

% --- Động cơ 1 ---
% Lấy động tham số K và a bằng cách gọi trực tiếp hàm motor1_tf
global ul;
ul = 1; K1 = motor1_tf(0, 0); % Khi w=0, ul=1 -> dwdt = K1
ul = 0; a1 = -motor1_tf(0, 1); % Khi w=1, ul=0 -> dwdt = -a1
G1 = K1 / (s + a1);

% Lấy động thông số PID bằng cách đọc text file pid_motor_left.m
txt1 = fileread('pid_motor_left.m');
Kp1 = str2double(char(regexp(txt1, 'kp1\s*=\s*([\d.]+)', 'tokens', 'once')));
Ki1 = str2double(char(regexp(txt1, 'ki1\s*=\s*([\d.]+)', 'tokens', 'once')));
Kd1 = str2double(char(regexp(txt1, 'kd1\s*=\s*([\d.]+)', 'tokens', 'once')));
if isnan(Kd1), Kd1 = 0; end % Đề phòng không tìm thấy

C1 = pid(Kp1, Ki1, Kd1);

% Hàm truyền hệ kín của Động cơ 1 (Closed-loop)
T1 = feedback(C1 * G1, 1);

% --- Động cơ 2 ---
% Lấy động tham số K và a bằng cách gọi trực tiếp hàm motor2_tf
global ur;
ur = 1; K2 = motor2_tf(0, 0);
ur = 0; a2 = -motor2_tf(0, 1);
G2 = K2 / (s + a2);

% Lấy động thông số PID bằng cách đọc text file pid_motor_right.m
txt2 = fileread('pid_motor_right.m');
Kp2 = str2double(char(regexp(txt2, 'kp2\s*=\s*([\d.]+)', 'tokens', 'once')));
Ki2 = str2double(char(regexp(txt2, 'ki2\s*=\s*([\d.]+)', 'tokens', 'once')));
Kd2 = str2double(char(regexp(txt2, 'kd2\s*=\s*([\d.]+)', 'tokens', 'once')));
if isnan(Kd2), Kd2 = 0; end

C2 = pid(Kp2, Ki2, Kd2);

% Hàm truyền hệ kín của Động cơ 2 (Closed-loop)
T2 = feedback(C2 * G2, 1);

%% 3. Mô phỏng đáp ứng với Step
% Hàm step() mặc định cho tín hiệu biên độ 1 (unit step)
% Để mô phỏng theo target_speed, ta nhân hệ thống với target_speed
System1 = target_speed * T1;
System2 = target_speed * T2;

% Thời gian mô phỏng
t = 0:0.01:5; % 5 giây

% Lấy dữ liệu đáp ứng
[y1, t1] = step(System1, t);
[y2, t2] = step(System2, t);

%% 4. Tính toán các thông số chất lượng (OS, Settling Time, SSE)
% Dùng stepinfo để lấy Settling Time và Overshoot
info1 = stepinfo(T1);
info2 = stepinfo(T2);

% Tính Steady-State Error (SSE)
% SSE = Target - Giá trị xác lập (Final Value)
% Giá trị xác lập có thể tính bằng target_speed * dcgain(T1)
sse1 = target_speed - target_speed * dcgain(T1);
sse2 = target_speed - target_speed * dcgain(T2);

%% 5. In kết quả ra Command Window
fprintf('========================================\n');
fprintf('       KẾT QUẢ ĐÁP ỨNG TỐC ĐỘ\n');
fprintf('Target Speed: %.2f\n', target_speed);
fprintf('========================================\n\n');

fprintf('--- Động cơ 1 ---\n');
fprintf('Độ vọt lố (Overshoot) : %.2f %%\n', info1.Overshoot);
fprintf('Thời gian xác lập (Ts): %.4f s\n', info1.SettlingTime);
fprintf('Sai số xác lập (SSE)  : %.4f\n\n', sse1);

fprintf('--- Động cơ 2 ---\n');
fprintf('Độ vọt lố (Overshoot) : %.2f %%\n', info2.Overshoot);
fprintf('Thời gian xác lập (Ts): %.4f s\n', info2.SettlingTime);
fprintf('Sai số xác lập (SSE)  : %.4f\n', sse2);
fprintf('========================================\n');

%% 6. Vẽ đồ thị (Plot)
figure('Name', 'Motor Speed Response', 'NumberTitle', 'off', 'Position', [100 100 800 500]);
hold on; grid on;

% Vẽ đường Target Speed
plot([t(1) t(end)], [target_speed target_speed], 'k--', 'LineWidth', 1.5, 'DisplayName', 'Target Speed');

% Vẽ đáp ứng Động cơ 1
plot(t1, y1, 'r', 'LineWidth', 2, 'DisplayName', 'Motor 1 Response');

% Vẽ đáp ứng Động cơ 2
plot(t2, y2, 'b', 'LineWidth', 2, 'DisplayName', 'Motor 2 Response');

% Trang trí đồ thị
xlabel('Time (seconds)', 'FontWeight', 'bold');
ylabel('Speed', 'FontWeight', 'bold');
title(sprintf('Speed Response of Two Motors (Target = %.1f)', target_speed), 'FontSize', 14);
legend('Location', 'best', 'FontSize', 11);
ylim([0 target_speed*1.5]); % Mở rộng trục y để nhìn rõ overshoot

%% 7. Xuất file báo cáo cho AI Agent
fileID = fopen('motor_simulation_results.txt', 'w');
fprintf(fileID, '=== BÁO CÁO MÔ PHỎNG MOTOR_RESPONSE.M ===\n');
fprintf(fileID, 'Target Speed: %.2f\n\n', target_speed);

fprintf(fileID, '--- Động cơ 1 (Bánh Trái) ---\n');
fprintf(fileID, 'Hàm truyền: K1 = %.4f, a1 = %.4f\n', K1, a1);
fprintf(fileID, 'PID       : Kp1 = %.4f, Ki1 = %.4f, Kd1 = %.4f\n', Kp1, Ki1, Kd1);
fprintf(fileID, 'Đáp ứng   : Overshoot = %.2f %%, Settling Time = %.4f s, SSE = %.4f\n\n', info1.Overshoot, info1.SettlingTime, sse1);

fprintf(fileID, '--- Động cơ 2 (Bánh Phải) ---\n');
fprintf(fileID, 'Hàm truyền: K2 = %.4f, a2 = %.4f\n', K2, a2);
fprintf(fileID, 'PID       : Kp2 = %.4f, Ki2 = %.4f, Kd2 = %.4f\n', Kp2, Ki2, Kd2);
fprintf(fileID, 'Đáp ứng   : Overshoot = %.2f %%, Settling Time = %.4f s, SSE = %.4f\n', info2.Overshoot, info2.SettlingTime, sse2);
fclose(fileID);
