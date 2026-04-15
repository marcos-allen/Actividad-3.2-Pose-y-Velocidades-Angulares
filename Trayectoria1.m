clear
close all
clc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% TIEMPO %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

tf = 8;              % Tiempo de simulacion en segundos (s)
ts = 0.1;            % Tiempo de muestreo en segundos (s)
t = 0: ts: tf;       % Vector de tiempo
N = length(t);       % Muestras

%%%%%%%%%%%%%%%%%%%%%%%% CONDICIONES INICIALES %%%%%%%%%%%%%%%%%%%%%%%%%%%%

x1 = zeros (1,N+1);  % Posición en el centro del eje que une las ruedas (eje x) en metros (m)
y1 = zeros (1,N+1);  % Posición en el centro del eje que une las ruedas (eje y) en metros (m)
phi = zeros(1, N+1); % Orientacion del robot en radianes (rad)

x1(1) = 0;    % Posicion inicial eje x
y1(1) = 0;   % Posicion inicial eje y
phi(1) = 0;   % Orientacion inicial del robot

%%%%%%%%%%%%%%%%%%%%%%%%%%%% PUNTO DE CONTROL %%%%%%%%%%%%%%%%%%%%%%%%%%%%

hx = zeros(1, N+1);  % Posicion en el punto de control (eje x) en metros (m)
hy = zeros(1, N+1);  % Posicion en el punto de control (eje y) en metros (m)

hx(1) = x1(1); % Posicion en el punto de control del robot en el eje x
hy(1) = y1(1); % Posicion en el punto de control del robot en el eje y


%%%%%%%%%%%%%%%%%%%%%% VELOCIDADES DE REFERENCIA %%%%%%%%%%%%%%%%%%%%%%%%%%

u = [0*ones(1,10)' % Velocidad lineal de referencia (m/s)
     5*ones(1,10)'
     0*ones(1,10)'
     5*ones(1,10)'
     0*ones(1,10)'
     5*ones(1,10)'
     0*ones(1,10)'
     5*ones(1,11)'];
w = [pi/2*ones(1,10)' % Velocidad angular de referencia (rad/s)
     0*ones(1,10)'
     -pi/2*ones(1,10)'
     0*ones(1,10)'
     -pi/2*ones(1,10)'
     0*ones(1,10)'
     -pi/2*ones(1,10)'
     0*ones(1,11)'];


%%%%%%%%%%%%%%%%%%%%%%%%% BUCLE DE SIMULACION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for k=1:N 
    
    % Mi phi actual ||| Mi phi hasta el momento anterior
    phi(k+1)=phi(k)+w(k)*ts; % Integral numérica (método de Euler)
    
    %%%%%%%%%%%%%%%%%%%%% MODELO CINEMATICO %%%%%%%%%%%%%%%%%%%%%%%%%
    %Aplicamos el modelo cinemático diferencial para obtener las
    %velocidades en x, y, phi
    xp1=u(k)*cos(phi(k+1)); 
    yp1=u(k)*sin(phi(k+1));
    phip = w(k);

    x1(k+1)=x1(k) + xp1*ts ; % Integral numérica (método de Euler)
    y1(k+1)=y1(k) + yp1*ts ; % Integral numérica (método de Euler)
    

    % Posicion del robot con respecto al punto de control
    hx(k+1)=x1(k+1); 
    hy(k+1)=y1(k+1);

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SIMULACION VIRTUAL 3D %%%%%%%%%%%%%%%%%%%%%%%%%%%%

% a) Configuracion de escena

scene=figure;  % Crear figura (Escena)
set(scene,'Color','white'); % Color del fondo de la escena
set(gca,'FontWeight','bold') ;% Negrilla en los ejes y etiquetas
sizeScreen=get(0,'ScreenSize'); % Retorna el tamaño de la pantalla del computador
set(scene,'position',sizeScreen); % Congigurar tamaño de la figura
camlight('headlight'); % Luz para la escena
axis equal; % Establece la relación de aspecto para que las unidades de datos sean las mismas en todas las direcciones.
grid on; % Mostrar líneas de cuadrícula en los ejes
box on; % Mostrar contorno de ejes
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)'); % Etiqueta de los eje

view([25 25]); % Orientacion de la figura
axis([-3 10 -3 10 0 2]); % Ingresar limites minimos y maximos en los ejes x y z [minX maxX minY maxY minZ maxZ]

% b) Graficar robots en la posicion inicial
scale = 4;
MobileRobot_5;
H1=MobilePlot_4(x1(1),y1(1),phi(1),scale);hold on;

% c) Graficar Trayectorias
H2=plot3(hx(1),hy(1),0,'g','lineWidth',2);

% d) Bucle de simulacion de movimiento del robot

step=1; % pasos para simulacion

for k=1:step:N

    delete(H1);    
    delete(H2);
    
    H1=MobilePlot_4(x1(k),y1(k),phi(k),scale);
    H2=plot3(hx(1:k),hy(1:k),zeros(1,k),'g','lineWidth',2);
    
    pause(ts);

end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PUZZLEBOT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

r = 0.05;            % Radio de las ruedas (m)
l = 0.17;            % Eje / Distancia entre ruedas (m)

% Calcular las velocidades de las ruedas a partir de las velocidades lineales y angulares
w_r = (u + (l/2) * w) / r; % Velocidad de la rueda derecha
w_l = (u - (l/2) * w) / r; % Velocidad de la rueda izquierda


%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Graficas %%%%%%%%%%%%%%%%%%%%%%%%%%%%
graph=figure;  % Crear figura (Escena)
set(graph,'position',sizeScreen); % Congigurar tamaño de la figura
subplot(711)
plot(t,u,color='#FF0000',LineWidth=2),grid('on'),xlabel('Tiempo [s]'),ylabel('v [m/s]');
subplot(712)
plot(t,w,color='#FF7F00',LineWidth=2),grid('on'),xlabel('Tiempo [s]'),ylabel('\omega [rad/s]');
subplot(713)
plot(t,x1(1:N),color='#FFFF00',LineWidth=2),grid('on'),xlabel('Tiempo [s]'),ylabel('x [m]');
subplot(714)
plot(t,y1(1:N),color='#00FF00',LineWidth=2),grid('on'),xlabel('Tiempo [s]'),ylabel('y [m]');
subplot(715)
plot(t,phi(1:N),color='#0000FF',LineWidth=2),grid('on'),xlabel('Tiempo [s]'),ylabel('\theta [rad]');
subplot(716)
plot(t,w_r,color='#9400D3',LineWidth=2),grid('on'),xlabel('Tiempo [s]'),ylabel('\omega_R [rad/s]');
subplot(717)
plot(t,w_l,color='#FF00FF',LineWidth=2),grid('on'),xlabel('Tiempo [s]'),ylabel('\omega_L [rad/s]');
