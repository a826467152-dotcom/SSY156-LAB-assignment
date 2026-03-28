function T = Tf(phi)
%TF Computes the Transformation T from derivatives euler ZYZ to angular
%velocities
%   phi: Euler angles vector ZYZ [phi, theta, psi] (3x1)

%TODO: Define the T matrix (3x3) as a function of phi and theta
% Extract the individual Euler angles (ZYZ)
    varphi = phi(1);
    theta  = phi(2);
    % psi = phi(3); % Note: psi is actually not needed for the T matrix!

    % Precompute sines and cosines for efficiency and readability
    s_phi = sin(varphi);
    c_phi = cos(varphi);
    s_th  = sin(theta);
    c_th  = cos(theta);

    % Construct the transformation matrix T
    T = [0, -s_phi,  c_phi * s_th;
         0,  c_phi,  s_phi * s_th;
         1,      0,          c_th];
end

