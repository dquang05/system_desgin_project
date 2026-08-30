% Hàm xác định hàm truyền động cơ 1
function dwdt=motor1_tf(t,w)
    global ul
    K = 12.05;      % Update số mới từ nhóm
    a = 10.64;    % Update số mới từ nhóm
    dwdt = (-a*w+K*ul);
end