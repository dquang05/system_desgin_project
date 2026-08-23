% Hàm xác định hàm truyền động cơ 1
function dwdt=motor1_tf(t,w)
    global ul
    K = 14.85;
    a = 11.5;
    dwdt = (-a*w+K*ul); %G = K/(s + a)
 end
