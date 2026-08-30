% Hàm xác định hàm truyền động cơ 2
function dwdt=motor2_tf(t,w)
    global ur
    K = 9.866;    % Update số mới từ nhóm
    a = 8.922;     % Update số mới từ nhóm
    dwdt = (-a*w+K*ur);
end