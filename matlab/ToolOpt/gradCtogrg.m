
function grad = gradCtogrg(x, f, fx)
% compute a forward difference gradient of the cost function J
%
% Gustavo Rodriguez-Gomez
%
% This code comes with no guarantee or warranty of any kind.
%
%
% inputs:
%         x, f = point and function
%		  f0   = f(x), preevaluated
%--------------------------------------------------------
%epsmaq = 1.1102230246d-16
const = 4.71216091533d-09;                  % sqrt(epsmaq/5)

hmin = 1.0d-10;

n=length(x);

%Ciclo para aproximar la matriz jacobiana
      for j = 1:n
         xj = x(j);
         absx = abs(xj);
         h = const*sqrt(absx);
         if (h < hmin)
%            h fuera de rango
             h = hmin;
         end

%        Escalamos h

         h = max(1.0d0, absx)*h;
%
%        Primero, nos aseguramos la representacion
%        exacta de h en el sistema de punto flotante
%
         temp = x(j) + h;
         h = temp - x(j);
%
%       perturbacion de la variable j-esima del vector x.
%
         x(j) = x(j) + h;
%        Evaluamos f(x+h)
         fxh = feval(f,x);

%        Aproximamos la matriz jacobiana
%        por medio de diferencia hacia adelante
         grad(j,:) = (fxh - fx) / h;
         x(j) = xj;
      end

