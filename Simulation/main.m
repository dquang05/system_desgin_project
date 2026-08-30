%% Initializing
clc;
clear all;
close all;
global ul ur R v_input w_input d;
%% Map
saban;
hold on
xlim([-3100,100]);
ylim([-200,1650]);
grid on
%% Mechanial parameters
b = 246;                            % Axial distance between 2 driving wheels
d = 176;                            % Distance from the center of the sensor array to the center of the drive shaft
r = 40;                           % Radius of each drive wheel
%% Initial setup
xM(1) = 0;                          % Horizontal position of the center of the 2 driving wheels (Vehicle's center)
yM(1) = 0;                          % Vertical position of the center of the 2 driving wheels (Vehicle's center)
phi(1) = pi;                        % Deflection angle of the center of the 2 driving wheels
pre_pos = [xM(1); yM(1); phi(1)];
xC(1)=xM(1) + d*cos(phi(1));        % Horizontal coordinates of the center of the sensor array (Tracking point)
yC(1)=yM(1) + d*sin(phi(1));        % Vertical coordinates of the center of the sensor array (Tracking point)
%% Modeling
h1 = plot(1,1, 'black');
h2 = plot(1,1, 'black');    
h3 = plot(1,1, 'black');
h4 = plot(1,1, 'black');
obit = animatedline('color',[0.4660 0.6740 0.1880],'LineWidth',2);
%% Tracking line 
pre_err = 0;
pPart = 0;
iPart = 0;
dPart = 0;
%% PID for motors
% Left motor
pre_nl = 0;
er1l = 0;
int_el = 0;
% Right motor
pre_nr = 0;
er1r = 0;
int_er = 0;
%% Initial constant
color = 0;
tsampsys = 0.2;                     % System sampling time
tsamppid = 0.03;                    % Motor settling time
R = 800;
line = 1;
stop = 0;
jj = round(tsampsys/tsamppid);
if jj == 0
   jj = 1;
end
%% Line tracking
pre_err = 0; % Initialize previous error
iPart = 0;   % Initialize integral part
tsampsys = 0.2; 
for i =1:200
    t(i)=i*tsampsys;
    % Get e2
    [line, i, j, e(i), stop, color] = get_e2(line, xC(i), yC(i), phi(i), i, j, color);
    fprintf('Step %d: e2 = %.2f', i, e(i));
    if (stop == 1)
        break;
    end
    % Tracking line
    [wl(i), wr(i), vl(i), vr(i), iPart, dPart] = tracking_line(e(i), pre_err, iPart, tsampsys);
    for j=1:jj
        % Pid_motor_left
        [int_el, vl(i), pre_nl] = pid_motor_left(er1l, wl(i), pre_nl, int_el);
        % Pid_motor_right
        [int_er, vr(i), pre_nr] = pid_motor_right(er1r, wr(i), pre_nr, int_er);
        % Embedded the system
        v(i)=(vl(i)+vr(i))/2;
        v_input=v(i);
        w_input=(vr(i)-vl(i))/b;
        % Determine instant coordinate
        [tt,y] = ode45(@dynamic_tf,[0 tsamppid],pre_pos);
        xM(i+1)= y(length(y),1);
        yM(i+1)= y(length(y),2);
        phi(i+1)= y(length(y),3);
        pre_pos=[ xM(i+1); yM(i+1); phi(i+1)];
        xC(i+1) = xM(i+1) + d*cos(phi(i+1));
        yC(i+1) = yM(i+1) + d*sin(phi(i+1));
        fprintf('Step %d: xC = %.2f, yC = %.2f, phi = %.2f\n', i, xC(i), yC(i), phi(i));
        % Plot coordinate and trajectory of robot
        delete(h1);
        delete(h2);
        delete(h3);
        delete(h4);
        xs = linspace(xC(i) + d*sin(phi(i)-pi/2), xC(i) + 0*d*sin(phi(i)-pi/2),2);
        ys = linspace(yC(i) - d*cos(phi(i)-pi/2), yC(i) - 0*d*cos(phi(i)-pi/2),2);
        xm = linspace(xC(i) + d*sin(phi(i)-pi/2)- b*sin(phi(i)), xC(i) + d*sin(phi(i)-pi/2) + b*sin(phi(i)),2);
        ym = linspace(yC(i) - d*cos(phi(i)-pi/2)+ b*cos(phi(i)), yC(i) - d*cos(phi(i)-pi/2) - b*cos(phi(i)),2);
        xa = linspace(xC(i) - 80*sin(phi(i)), xC(i) + 80*sin(phi(i)),2);
        ya = linspace(yC(i) + 80*cos(phi(i)), yC(i) - 80*cos(phi(i)),2);

        h1=plot(xs, ys, 'red','LineWidth',2);
        h2 = rectangle('Position',[xM(i)-110, yM(i)-110, 220, 220], 'Curvature',[1,1], 'EdgeColor','black','LineWidth',1);
        h3=plot(xa, ya, 'blue','Linewidth',2);
        addpoints(obit,xC(i), yC(i));
        drawnow;
        movieVector(i) = getframe;

        pre_err = e(i);
    end
end
%% Graph of kinematic data, linear velocity, angular velocity, sensor error
vl(1) = 0; vr(1) = 0;
wl(1) = 0; wr(1) = 0;
vl(i) = 0; vr(i) = 0;
wl(i) = 0; wr(i) = 0;
wl = wl*10; wr = wr*10;
xlabel('mm');
xlim([-3100,100]);
ylim([-200,1650]);
title('Quỹ đạo di chuyển của robot trên sa bàn');
figure();
plot(t,e);
hold on;
y=zeros(1,length(t));
plot(t,y,'r');
grid on;
xlabel('Time(s)');
ylabel('Độ lệch tâm dò line so với line (mm)');
title('Sai số bám line theo thời gian');
figure();
plot(t,wl);
hold on;
plot(t,wr);
grid on;
legend('Bánh trái', 'Bánh phải');
xlabel('Thời gian (s)');
ylabel('Tốc độ góc (RPM)');
title('Tốc độ góc 2 bánh xe');
figure();
plot(t,vl);
hold on;
plot(t,vr);
grid on;
legend('Bánh trái', 'Bánh phải');
xlabel('Thời gian (s)');
ylabel('Tốc độ dài (mm/s)');
title('Tốc độ dài 2 bánh xe');
ylim([0 900]);

%% Export results for AI Agent
fileID = fopen('main_simulation_results.txt', 'w');
rms_error = rms(e);
[max_error, max_idx] = max(abs(e));
time_max_error = t(max_idx);
zero_crossings = sum(e(1:end-1) .* e(2:end) < 0);
IAE = sum(abs(e)) * tsampsys; % Tích phân sai số tuyệt đối

fprintf(fileID, '=== BÁO CÁO MÔ PHỎNG MAIN.M ===\n');
fprintf(fileID, '1. Đánh giá tổng quan (Độ bám):\n');
fprintf(fileID, '- Thời gian mô phỏng: %.2f s\n', t(end));
fprintf(fileID, '- RMS Error (Sai số hiệu dụng): %.4f mm\n', rms_error);
fprintf(fileID, '- IAE (Tổng sai số cộng dồn): %.4f\n\n', IAE);

fprintf(fileID, '2. Đánh giá Dao động (Lắc đuôi):\n');
fprintf(fileID, '- Số lần quét qua tâm vạch (Zero-crossings): %d lần\n', zero_crossings);
if zero_crossings > 10
    fprintf(fileID, '  -> [CẢNH BÁO]: Xe bị dao động zigzag. AI Agent nên gợi ý giảm Kp hoặc tăng Kd ở hàm tracking_line!\n\n');
else
    fprintf(fileID, '  -> [OK]: Xe chạy tương đối ổn định.\n\n');
end

fprintf(fileID, '3. Đánh giá Văng cua (Overshoot):\n');
fprintf(fileID, '- Lỗi văng xa nhất: %.4f mm\n', max_error);
fprintf(fileID, '- Thời điểm xảy ra lỗi lớn nhất: t = %.2f s\n', time_max_error);
fclose(fileID);
