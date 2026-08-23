% Line tracking controller (PID)
function [wl, wr, vl, vr, err, pre_err]= tracking_line(err, pre_err, iPart,tsampsys)
     global Kp Ki Kd vR r b;
     tsampsys = 0.3;
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