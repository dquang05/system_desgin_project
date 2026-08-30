% PID left motor controller
function [int_el, vl, pre_nl] = pid_motor_left(er1l, wl, pre_nl, int_el)
    global ul;
kp1 = 1.0673;     % Số mới đã tune theo K,a mới
ki1 = 13.948;
kd1 = 0;

    r = 40;
    tsamppid = 0.03;
    
    er2l = er1l;
    er1l = wl*30/pi - pre_nl;
    int_el = int_el + er1l*tsamppid;
    der_el = (er1l - er2l)/tsamppid;
    ul = kp1*er1l + ki1*int_el + kd1*der_el;    
    if ul > 255
        ul = 255;
    end
    if ul < 0 
        ul = 0;
    end
    [tt,y] = ode45(@motor1_tf,[0 tsamppid],pre_nl);
    pre_nl = y(length(y),1);
    vl = pre_nl*pi/30*r;
end