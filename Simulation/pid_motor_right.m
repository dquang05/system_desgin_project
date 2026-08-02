% PID right motor controller
function [int_er, vr, pre_nr] = pid_motor_right(er1r, wr, pre_nr, int_er)
    global ur;
    kp2 = 1.5792;
    ki2 = 26.1527;
    kd2 = 0.0076;

    r = 65/2;
    tsamppid = 0.03;

    er2r = er1r;
    er1r = wr * 30 / pi - pre_nr; 
    int_er = int_er + er1r * tsamppid; 
    der_er = (er1r - er2r)/ tsamppid; 
    ur = kp2*er1r + ki2*int_er + kd2 * der_er;
    if ur > 255 
        ur = 255;
    end
    if ur < 0 
        ur = 0;
    end
    [tt,y] = ode45(@motor2_tf,[0 tsamppid],pre_nr);
    pre_nr = y(length(y),1);
    pre_nr = pre_nr*4/3;
    vr = pre_nr*pi/30*r;
end