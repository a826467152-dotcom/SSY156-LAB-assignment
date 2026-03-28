function [P] = Pol5th3DOF( Pini, Pend, tini,tend,t )
%POL5TH Generates a 3DOF trajectory based on a 5th degree polynomial
%function
%   The function calculates a trajectory between the initial position to
%   the final position with a duration of tend-tini
%   Pini:   initial position (3x1)
%   Pend:   final position (3x1)
%   tini:   initial time
%   tend:   final time
%   t:      current time (simulation time)
% Return:
%   P:  position vector (3x1) 


% The polynomial functions are only valid in the range t=[tini-tend].
% Therefore, if t is outside that range it will produce wrong outputs. 
% Then, we need to avoid this by setting the output outside the given
% range, i.e., t<tini and t>tend
if t<tini
    P=Pini;
elseif t>tend
    P=Pend;
else
    
    % TODO: Define the polinomial function p(i)=a0+a1*t+a2*t^2+...+an*t^n for
    % each axis
    
% 1. Calculate the total duration
    T_total = tend - tini;
    
    % 2. Calculate normalized time tau in range [0, 1]
    tau = (t - tini) / T_total;
    
    % 3. Calculate the 5th-degree polynomial scaling factor (Minimum Jerk)
    % This satisfies position, velocity=0, and acceleration=0 at boundaries
    s = 10*tau^3 - 15*tau^4 + 6*tau^5;
    
    % 4. Calculate position for each axis
    % P(t) = Pini + (Pend - Pini) * s(tau)
    
    % x axis
    P(1,1) = Pini(1) + (Pend(1) - Pini(1)) * s;   
    % y axis
    P(2,1) = Pini(2) + (Pend(2) - Pini(2)) * s;
    % z axis
    P(3,1) = Pini(3) + (Pend(3) - Pini(3)) * s;
    
end



end


