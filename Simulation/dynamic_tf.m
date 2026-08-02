% Determine coordinate of robot
function dq=dynamic_tf(t,q)
    global d v_input w_input
            dq=zeros(3,1);
            dq(1)=cos(q(3))*v_input; %x
            dq(2)=sin(q(3))*v_input; %y
            dq(3)=w_input; %phi
 end
    