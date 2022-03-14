%% tests uart recibir
delete(instrfind);%evita problemas al abrir y cerrar el puerto
serialportlist("available")';
TivaObj = serial('COM5','Baudrate', 115200);
fopen(TivaObj);
% 
% for i = 1:n
%     data(1,i) = fscanf(TivaObj, '%f');
% end
% fclose(TivaObj);
% 
% stairs(data);
% ylabel('valor');
% xlabel('tiempo (ms)');
% axis()
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
N = 1000;  % Aumentar el número de datos arriba, para apreciar mejor el efecto.
t = linspace(0,2*pi,N)';    % no es necesario trasponer, es para tener vectores columna
%y = cos(10*t);
K = 10;      % Restricción: K debe ser factor de N. Se puede ajustar el código abajo para
            % eliminar esta restricción. 
        
figure(7); clf;
h7 = plot(t,zeros(N,1));
xlim([0,t(end)]);
buffer = zeros(K,1);
k = 1;
while(1)
for n = 1:N
    data(1,n) = fscanf(TivaObj, '%f');
    buffer(k) = data(1,n);
    
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

