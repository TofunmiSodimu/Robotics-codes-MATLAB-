function J = xArm_Jacobian_Tofunmi(frames)
% Calculate the Jacobian matrix of the xArm, for velocities of the pen
% frames - 4x4x5 D-H frames w.r.t the spatial frame

% rotation = frames(1:3,1:3,:);  % rotation matrices of D-H frames
origin = frames(1:3,4,:);      % positions of origins of D-H frames
z = frames(1:3,3,:);  % the z-axis of D-H frames, aligned with joint axis

% linear velocity
Jv1 = cross([0;0;1], origin(:,:,6));
Jv2 = cross(z(:,:,1), (origin(:,:,6)-origin(:,:,1)));
Jv3 = cross(z(:,:,2), (origin(:,:,6)-origin(:,:,2)));
Jv4 = cross(z(:,:,3), (origin(:,:,6)-origin(:,:,3)));
Jv5 = cross(z(:,:,4), (origin(:,:,6)-origin(:,:,4)));

Jv = [Jv1 Jv2 Jv3 Jv4 Jv5];
% angular velocity 
Jw= [[0;0;1] z(:,:,1) z(:,:,2) z(:,:,3) z(:,:,4)];

J = [Jv;Jw];
end