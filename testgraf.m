%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
N = 100;  % Aumentar el número de datos arriba, para apreciar mejor el efecto.
t = linspace(0,2*pi,N)';    % no es necesario trasponer, es para tener vectores columna
%y = cos(10*t);
K = 1;      % Restricción: K debe ser factor de N. Se puede ajustar el código abajo para
            % eliminar esta restricción. 
%valores = 0;
figure(7); clf;
h7 = plot(t,zeros(N,1));
xlim([0,t(end)]);
buffer = zeros(K,1);
k = 1;
conta = 0;
while(1)
    conta = conta +1;
    palabra = readline(TivaObj);
    valores = split(palabra,"&");
    numero = str2double(valores(1));
    buffer(k) = numero;
    if (conta>N)
        buffer(k)
for n = 1:N
    %data(1,n) = fscanf(TivaObj, '%f');
    %palabra = num2str(data(1,n));
    palabra = readline(TivaObj);
    valores = split(palabra,"&");
    numero = str2double(valores(1));
    k = 1;
    buffer(k) = numero;
    
    if(k == K)
        
        h7.YData((n-K+1):n) = buffer;   % Asume que K es factor de N. De lo contrario,
                                        % hay que hacer ajustes adicionales.
        drawnow limitrate
        k = 1;
    else
        k = k + 1;    
    end
end
h7 = plot(t,zeros(N,1));
xlim([0,t(end)]);
buffer = zeros(K,1);
end 
fclose(TivaObj);

