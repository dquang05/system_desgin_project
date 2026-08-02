% Hàm xác định hàm truyền động cơ 2
function dwdt=motor2_tf(t,w)
    global ur
    K = 16.8;
    a = 12.94;
    dwdt = (-a*w+K*ur);
end
   