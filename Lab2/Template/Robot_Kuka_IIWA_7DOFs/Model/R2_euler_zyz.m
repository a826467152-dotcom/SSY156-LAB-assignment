function phi = R2_euler_zyz(R)
%R2_EULER_ZYZ Rotation to euler angles ZYZ
%   Transforms a rotation matrix R into euler angles (ZYZ) representation
%   R: Rotation matrix (3x3)
% Return:
%   phi: euler angles ZYZ (3x1) [phi, theta, psi]

%TODO: get the euler angles (3x1) as atan2 functions of the elements of R(i,j). 
% Extract theta
    theta = atan2(sqrt(R(1,3)^2 + R(2,3)^2), R(3,3));
    
    if abs(sin(theta)) > 1e-6
        phi_val = atan2(R(2,3), R(1,3));
        psi_val = atan2(R(3,2), -R(3,1));
    else
        % Singular case: theta is 0 or pi
        phi_val = 0; % Arbitrary choice
        psi_val = atan2(-R(1,2), R(1,1)); % Sum or difference of angles
    end
    phi = [phi_val; theta; psi_val];
end

