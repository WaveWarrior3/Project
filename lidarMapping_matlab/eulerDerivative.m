function [xdot] = eulerDerivative(x,xprev,dt)
xdot = (x-xprev)/dt;
end

