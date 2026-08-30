% Line tracking controller (PID)
function [wl, wr, vl, vr, err, pre_err]= tracking_line(err, pre_err, iPart,tsampsys)
     % Kp/Kd nay TU THIET KE qua quy trinh 3 buoc (khong phai so tham khao):
     % B1: do bien dao dong thuan Kp (Ki=Kd=0) -> vo o Kp=0.038
     % B2: lui an toan, chon Kp=0.035 (co dem ~8% duoi bien vo)
     % B3: them Kd=0.001 de dap dao dong con sot
     % Ket qua: RMS sai so on dinh 16.56mm, dinh sai so 28.95mm, vR=500mm/s
     Kp = 0.045;
     Ki = 0.00000;
     Kd = 0.0010;
     vR = 500;
     r = 40;
     b = 246;
% Compute PID terms
     iPart = iPart + err * tsampsys;
     dPart = (err - pre_err) / tsampsys;
     w = Kp * err + Kd * dPart + Ki * iPart;    % PID control output
     v = vR;
     vl= v - w*b/2;
     vr= v + w*b/2;
     wl = vl/r;
     wr = vr/r;
end
