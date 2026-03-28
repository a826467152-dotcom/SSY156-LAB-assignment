function R = euler_ZYZ_2R(phi)
%EULER_ZYZ_2R Computes the Rotation Matrix from Euler Angles ZYZ

%   phi: ZYZ Euler angles [phi, theta, psi] (3x1)
% Return:
%   R: Rotation matrix (3X3)

phi1=phi(1);
theta=phi(2);
psi=phi(3);

%TODO: calculate R (3x3) from euler angles (ZYZ) [phi,theta, psi]
P_phi = [cos(phi1), -sin(phi1), 0; sin(phi1), cos(phi1), 0; 0, 0, 1];
P_theta = [cos(theta), 0, sin(theta); 0, 1, 0; -sin(theta), 0, cos(theta)];
P_psi = [cos(psi), -sin(psi), 0; sin(psi), cos(psi), 0; 0, 0, 1];

R = P_phi * P_theta * P_psi;

end

