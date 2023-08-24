
%% Posición del robot manipulador totalmente vertical

% Es igual que como hemos sacado los parametros de DH

% Cargamos el UR5 que esta dentro de la ToolBox
ur5 = loadrobot("universalUR5");
ur5.DataFormat = 'row';

% Linea que no influye en este script, pero es por si alguna vez
% hace falta una rotación en el eje X
rotationX = @(t) [1 0 0; 0 cosd(t) -sind(t) ; 0 sind(t) cosd(t)] ;

% Ya si ponemos ur5 y le damos en el intro, nos tiene que sacar los valores
% que guarda este robot. Uno de ellos es el BodyNames, en este caso, el 
% robot esta formado por 10 cuerpos, el 1 es el Base_link y el 10, que es
% el último y se llama tool10.
% Para poder ver esto, hay que escribir ur5.BodyNames{10}, nosotros
% usaremos como end-effector este cuerpo.

% Cierra la figure que haya y abre otra
close(findobj('type','figure','name','Interactive Visualization'));

% Visualizar el robot en la posición totalmente vertical
% Para poder interactuar con el modelo del robot cargado, hay que meter el
% siguiente comando
ur5_i = interactiveRigidBodyTree(ur5);
q_home = [0 -90 0 -90 0 0]'*pi/180;  % Posición de partida
rotate3d off;
view(145,25)
lightangle(20,-160)
axis([-1 1 -1 1 -0.5 1])  % Los ejes 
hold on
zlim([-0.5 1.5])   % Los límites de visualización del eje Z
ur5_i.ShowMarker = false;   
ur5_i.Configuration = q_home;   % Visualizarlo con la cofiguración q_home 
q_home

