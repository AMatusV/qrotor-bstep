function ynew = RKStep(fname,tc,yc,fc,h,k)
%  function [tnew,ynew,fnew] = RKStep(fname,tc,yc,fc,h,k,n)
% [tnew,ynew,fnew] = RKStep(fname,tc,yc,fc,h,k)
% Single step of  the kth order Runge-Kutta method.
%
% fname is a string that names a function of the form f(t,y)
% where t is a scalar and y is a column d-vector.
%
% yc is an approximate solution to y'(t) = f(t,y(t)) at t=tc.
%
% fc = f(tc,yc).
%
% h is the time step. 
%
% k is the order of the Runge-Kutta method used, 1<=k<=5.
%
% The user must calculate:
% tnew=tc+h, ynew is an approximate solution at t=tnew, and 
% fnew = f(tnew,ynew)
% in his drives

n = length(yc);            % number of ode's
k1 = zeros(n,1);
k2 = zeros(n,1);
k3 = zeros(n,1);
k4 = zeros(n,1);
k5 = zeros(n,1);
k6 = zeros(n,1);
ynew = zeros(n,1);
fnew = zeros(n,1);

if k==1
   k1 = h.*fc;
   ynew = yc + k1;
elseif k==2
   k1 = h.*fc;
   k2 = h.*feval(fname,tc+h,yc+k1);
   ynew  = yc + (k1 + k2)./2;
elseif k==3 
   k1 = h.*fc;
   k2 = h.*feval(fname,tc+(h/2),yc+(k1./2));
   k3 = h.*feval(fname,tc+h,yc-k1+2.*k2);
   ynew  = yc + (k1 + 4.*k2 + k3)./6;
elseif k==4
   k1 = h.*fc;
   k2 = h.*feval(fname,tc+(h/2),yc+(k1./2));   
   k3 = h.*feval(fname,tc+(h/2),yc+(k2./2));  
   k4 = h.*feval(fname,tc+h,yc+k3);  
   ynew  = yc + (k1 + 2.*k2 + 2.*k3 + k4)./6;
elseif k==5
   k1 = h.*fc;
   k2 = h.*feval(fname,tc+(h/4),yc+(k1./4));   
   k3 = h.*feval(fname,tc+(3*h/8),yc+(3/32).*k1+(9/32).*k2);    
   k4 = h.*feval(fname,tc+(12/13)*h,yc+(1932/2197).*k1-(7200/2197).*k2+(7296/2197).*k3);               
   k5 = h.*feval(fname,tc+h,yc+(439/216).*k1 - 8.*k2 + (3680/513).*k3 -(845/4104).*k4);
   k6 = h.*feval(fname,tc+(1/2)*h,yc-(8/27).*k1 + 2.*k2 -(3544/2565).*k3 + (1859/4104).*k4 - (11/40).*k5);
   ynew  = yc + (16/135).*k1 + (6656/12825).*k3 + (28561/56430).*k4 - (9/50).*k5 + (2/55).*k6;
end
% tnew = tc+h;
% fnew = feval(fname,tnew,ynew);

