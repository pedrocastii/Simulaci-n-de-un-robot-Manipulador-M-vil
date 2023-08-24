%% Función Helper

function stateDot = exampleHelperTimeBasedTaskInputs(motionModel, timeInterval, initialTform, finalTform, t, state)
% This function is for internal use only and may be removed in a future
% release

%exampleHelperTimeBasedTaskInputs Pass time-varying inputs to the taskSpaceMotionModel derivative
%   Since the taskSpaceMotionModel derivative method is updated at an
%   instant in time, a wrapper function is needed to provide time-varying
%   tracking inputs. This function computes the value of the transform
%   trajectory at an instant in time, t, and provides that input to the
%   derivative of the associated taskSpaceMotionModel at that same instant.
%   The resultant state derivative can be passed to an ODE solver to
%   compute the tracking behavior.

% Copyright 2019 The MathWorks, Inc.

[refPose, refVel] = transformtraj(initialTform, finalTform, timeInterval, t);

stateDot = derivative(motionModel, state, refPose, refVel);

end
%% Robot con 6 DoF 

function robot = robotWith6DoFFloatingBase(dataformat)
%robotWith6DoFFloatingBase This function constructs a robot with a 6-DoF joint made up of 1-DoF joints

% Create the tree
robot = rigidBodyTree();
robot.BaseName = 'world';
if nargin < 1
    dataformat = 'column';
end

robot.DataFormat = dataformat;
robot.Gravity = [0 0 -9.81];

jointAxisName = {'X', 'Y', 'Z'};
jointAxisValue = eye(3);

% Initialize parent name
parentBodyName = robot.BaseName;

% Add the prismatic joints
for i = 1:numel(jointAxisName)-1
    bodyName = ['base' jointAxisName{i} 'TransBody'];
    jointName = ['base' jointAxisName{i} 'TransJoint'];
    rb = rigidBody(bodyName);
    rb.Mass = 0;
    rb.Inertia = [0 0 0 0 0 0];
    rbJoint = rigidBodyJoint(jointName, 'prismatic');
    rbJoint.JointAxis = jointAxisValue(i,:);
    rbJoint.PositionLimits = [-inf inf];
    rb.Joint = rbJoint;
    
    % Add to robot using previous body as parent
    robot.addBody(rb, parentBodyName);
    parentBodyName = rb.Name; % Update parent body name
end

% Add the revolute joints
for i = 3:numel(jointAxisName)
    bodyName = ['base' jointAxisName{i} 'RevBody'];
    jointName = ['base' jointAxisName{i} 'RevJoint'];
    rb = rigidBody(bodyName);
    rb.Mass = 0;
    rb.Inertia = [0 0 0 0 0 0];
    rbJoint = rigidBodyJoint(jointName, 'revolute');
    rbJoint.JointAxis = jointAxisValue(i,:);
    rbJoint.PositionLimits = [-inf inf];
    rb.Joint = rbJoint;
    
    % Add to robot using previous body as parent
    robot.addBody(rb, parentBodyName);
    parentBodyName = rb.Name; % Update parent body name
end

end
%% Creación de una base flotante para el robot manipulador UR5

% Se inicializan valores, y se carga el robot UR5

warning('off','robotics:robotmanip:joint:ResettingHomePosition') %Turn off some annoying warnings
manipulador = loadrobot("universalUR5",DataFormat='row');
warning('on','robotics:robotmanip:joint:ResettingHomePosition') %Turn warnings back o
show(manipulador);

% Creamos un robot con 3 dof, dos articulaciones prismáticas y una de
% revolución

WMM = robotWith6DoFFloatingBase('row');
addSubtree(spotWithFloatingBase, 'baseZRevBody', spotWithFixedBase, ReplaceBase=false);

% body1 = rigidBody('b1');
% body2 = rigidBody('b2');
% jnt1 = rigidBodyJoint('jnt1','prismatic');
% body1.Joint = jnt1;
% jnt2 = rigidBodyJoint('jnt2','prismatic');
% body2.Joint = jnt2;
% body3 = rigidBody('b3');
% jnt3 = rigidBodyJoint('jnt3','revolute');
% body3.Joint = jnt3;
% basename = WMM.BaseName;
% addBody(WMM,body1,basename);
% addBody(spotWithFloatingBase,body2,'b1');
% addBody(spotWithFloatingBase,body3,'b2');
% showdetails(spotWithFloatingBase) % Muestra las carácterísticas de el nuevo robot

% Añadimos el UR5 encima del robot creado con el comando subtree.

showdetails(spotWithFloatingBase) 

% Creamos un robot inteactivo, para poder visualizar distintas configuraciones del
% robot.
%Creamos tres configuraciones cogidas al azar, donde se cambia la posición
%y orientación de la plataforma, pero se mantiene el UR5 en la posición de
%reposo.
 
homeConfiguration = [0 0 0 0 0 0 0 -1.5708 0 -1.5708 0 0];
robotConfiguration = [0 -1.5708 0 -1.5708 0 0];
% robotConfiguration2 = [1 0 pi/2 0 -1.5708 0 -1.5708 0 0];
% robotConfiguration3 = [1 0 0 0 -1.5708 0 -1.5708 0 0];
baseLocation = [1 0.3 0.6 0 0 pi/8];
figure;
configuracionWMM = [baseLocation robotConfiguration]

WMM_i = interactiveRigidBodyTree(WMM);
rotate3d off;
view(145,25)
lightangle(20,-160)
axis([-1 1 -1 1 -0.5 1])
hold on
zlim([-0.5 1.5])
WMM_i.ShowMarker = false;
WMM_i.Configuration = configuracionWMM

% WMM_i.Configuration = robotConfiguration2;

% WMM_i.Configuration = robotConfiguration3;




