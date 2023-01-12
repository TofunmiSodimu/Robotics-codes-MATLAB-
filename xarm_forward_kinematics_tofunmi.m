function frames = xarm_forward_kinematics_tofunmi(q, params, needle_len)
% Calculate the forward kinematics of xArm using the D-H convention
% q - 5x1 joint vector
% params - 5x3 D-H parameters [d, a, alpha]
% frames - 4x4x6 D-H frames and tool frame w.r.t the spatial frame

T = eye(4);
frames = zeros(4, 4, 6);
for i = 1:1:5
    st = sin(q(i)); ct = cos(q(i));
    sa = sin(params(i,3)); ca = cos(params(i,3));
    a = params(i,2);
    d = params(i,1);
    A = [ct  -st*ca   st*sa  a*ct;
        st   ct*ca  -ct*sa  a*st;
        0    sa      ca     d;
        0    0       0      1];

    T = T * A;

    if i == 4
        T = T * calculate_rotation_z(pi/2) * calculate_rotation_x(pi/2);
    end
    frames(:,:,i) = T;
end
T_gripper2pen = [eye(3) [0;0;needle_len]; 0 0 0 1];
frames(:,:,6) = frames(:,:,5)*T_gripper2pen;
end
