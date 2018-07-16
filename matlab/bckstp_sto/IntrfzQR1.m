function  [f, g] = IntrfzQR1(u)
% Función que da la interfaz entre el método
% del Gradiente Conjugado y el modelo
% Salidas:
%   f         valor de la función costo
%   g         gradiente de la función costo
% --------------------------------------------------
    global ContEvalf                 % contador del número de llamadas
    global ContIter                  % contador de iteraciones
    
    ContEvalf = ContEvalf + 1;
    [f, dummy] = funQR1(u);
    % gradiente aproximado por diferencias
    f0 = f;              % valor de inicio para f (funcion costo)
    g = gradCtogrg(u, @funQR1, f0);

