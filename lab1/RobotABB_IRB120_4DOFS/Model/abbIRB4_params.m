function p=abbIRB4_params
%ABBIRB4_PARAMS Creates an array of kinematic parameters
% Return:
%   p: array of kinematic parameters (5x1)

% Kinematic parameters
% TODO: Define the kinematic parameters.
L1=0.29;
L2=0.27;
L3=0.07;
L4=0.302;
L5=0.072;
L6=0.1; % EF offset
L7=sqrt(L3^2 + L4^2);% linear offset between cf 2 and cf 3
L8=L5 + L6;% linear offset between cf 3 and cf 4 (in this case,  consider cf_4=ef) (See Figure 1 in Lab Assignment document)
al=atan2(L4,L3);% angular offset between cf 2 and cf 3


% Parameters
p=[L1,L2,L7,L8,al];

end

