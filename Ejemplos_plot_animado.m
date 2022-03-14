%% Diseño e Innovación de Ingeniería 1
%  Luis Alberto Rivera
%  Ejemplos de gráficas con animación

% REVISEN LA FUNCIÓN "drawnow", Y PONGAN ATENCIÓN A LA PROPIEDAD "limitrate". Ésta limita
% el número de actualizaciones de la gráfica, haciendo la animación mucho más rápida.
% A continuación muestro varias formas de hacer la animación, con y sin "limitrate".
% Las primeras opciones pueden ser algo lentas...

%% Datos
N = 1000;
t = linspace(0,2*pi,N)';    % no es necesario trasponer, es para tener vectores columna
y = cos(10*t);

%% Ejemplo 1: Sin reservar memoria (no recomendado), sin handler, sin "limitrate"
y1 = [];
figure(1); clf;
% xlim([0,t(end)]); % se alterará al graficar cada vez, dado que y1 va creciendo
tic;
for n = 1:N
    y1 = [y1;y(n)];
    plot(t(1:n),y1);
    xlim([0,t(end)]); % Para ajustarlo cada vez
    drawnow
end
tiempo1 = toc

%% Ejemplo 2: reservando memoria, sin handler, sin "limitrate"
y2 = zeros(N,1);
figure(2); clf;
xlim([0,t(end)]);
tic;
for n = 1:N
    y2(n) = y(n);
    plot(t,y2);     % No es necesario volver a ajustar el rango del eje x
    drawnow
end
tiempo2 = toc

%% Ejemplo 3: con handler, sin "limitrate"
figure(3); clf;
h3 = plot(t,zeros(N,1));
xlim([0,t(end)]);

tic;
for n = 1:N
    h3.YData(n) = y(n);
    drawnow
end
tiempo3 = toc

%% Ejemplo 4: Sin reservar memoria (no recomendado), sin handler, con "limitrate"
y4 = [];
figure(4); clf;
% xlim([0,t(end)]); % se alterará al graficar cada vez, dado que y1 va creciendo
tic;
for n = 1:N
    y4 = [y4;y(n)];
    plot(t(1:n),y4);
    xlim([0,t(end)]);   % Para ajustarlo cada vez
    drawnow limitrate   % "limitrate" limita el número de actualizaciones. Vean el help
end
tiempo4 = toc

%% Ejemplo 5: reservando memoria, sin handler, con "limitrate"
y5 = zeros(N,1);
figure(5); clf;
xlim([0,t(end)]);
tic;
for n = 1:N
    y5(n) = y(n);
    plot(t,y5);     % No es necesario volver a ajustar el rango del eje x
    drawnow limitrate
end
tiempo5 = toc

%% Ejemplo 6: con handler, con "limitrate"
%  Se recomienda esta opción para los mini-proyectos, si el muestreo no necesita ser
%  demasiado rápido.
figure(6); clf;
h6 = plot(t,zeros(N,1));
xlim([0,t(end)]);

tic;
for n = 1:N
    h6.YData(n) = y(n);
    drawnow limitrate
end
tiempo6 = toc


%% Ejemplo 7: con handler y "limitrate", actualización cada K muestras
%  Se recomienda esta opción para los mini-proyectos, si la anterior limita mucho la
%  frecuencia de muestreo.

N = 1000;  % Aumentar el número de datos arriba, para apreciar mejor el efecto.
t = linspace(0,2*pi,N)';    % no es necesario trasponer, es para tener vectores columna
y = cos(10*t);
K = 10;      % Restricción: K debe ser factor de N. Se puede ajustar el código abajo para
            % eliminar esta restricción. 
        
figure(7); clf;
h7 = plot(t,zeros(N,1));
xlim([0,t(end)]);
buffer = zeros(K,1);
k = 1;

tic;
for n = 1:N
    buffer(k) = y(n);
    
    if(k == K)
        h7.YData((n-K+1):n) = buffer;   % Asume que K es factor de N. De lo contrario,
                                        % hay que hacer ajustes adicionales.
        drawnow limitrate
        k = 1;
    else
        k = k + 1;    
    end
end

tiempo7 = toc
