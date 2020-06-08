function [f, F_x, F_u] = transitionFunction(x,u,b)
% [f, F_x, F_u] = transitionFunction(x,u,b) predicts the state x at time t given
% the state at time t-1 and the input u at time t. b is the distance between
% the wheels f the differential-drive robot. F_x denotes the Jacobian
% of the state transition function with respect to the state evaluated at
% the state and input provided. F_u denotes the Jacobian of the state
% transition function with respect to the input evaluated at the state and
% input provided.
% State and input are defined according to the book pp. 337

if nargin < 3 || isempty(b)
    b = .1;
end

del_s = (u(2) + u(1))/2;
del_t = (u(2) - u(1))/b;

f = x(:);
f(1:3,1) = [x(1); x(2); x(3)] + [del_s*cos(x(3)+del_t/2); del_s*sin(x(3)+del_t/2); del_t];


F_x = eye(size(x,1));
F_x(1,3) = -del_s*sin(x(3)+del_t/2); 
F_x(2,3) = del_s*cos(x(3) + del_t/2); 

F_u = zeros(size(x,1),2);
F_u(1:3,1:2) = [0.5*cos(x(3)+del_t/2)+(del_s/(2*b))*sin(x(3)+del_t/2), 0.5*cos(x(3)+del_t/2)-(del_s/(2*b))*sin(x(3)+del_t/2); ...
       0.5*sin(x(3)+del_t/2)-(del_s/(2*b))*cos(x(3)+del_t/2), 0.5*sin(x(3)+del_t/2)+(del_s/(2*b))*cos(x(3)+del_t/2); ...
       -1/b 1/b];