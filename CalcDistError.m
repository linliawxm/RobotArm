%Objective function to calculate the end effector distance from the
%clipboard coordinate system origin
%
%Input parameter:
% dh = [a1 a2 d1 d2 d3 offset1 offset2]
% a1 a2 d1 d2 d3: DH parameters
% offset1 offset2: kinematic joint coordinate offsets
% baseY: clipboard coordinate system translation on Y axis
% a3,alpha1,alpha2, alpha3, offset3 are treated as fixed value
% a3 = 0; alpha1 = 0; alpha2 = -90; alpha3 = 0; offset3 = 0;
function error = CalcDistError(dh)
    clear L
    deg = pi/180;

    %build parameter a
    a=[dh(1);dh(2);0];
    %build parameter alpha
    alpha=[0;-90;0];
    %build parameter d
    d=[dh(3);dh(4);dh(5)];
    %4th column are parameter alpha
    offset=[dh(6);dh(7);0];
    %define the links of robot
    L(1) = Revolute('d', d(1), 'a', a(1),'alpha', alpha(1)*deg, ... 
        'offset',offset(1)*deg ); 

    L(2) = Revolute('d', d(2), 'a', a(2), 'alpha', alpha(2)*deg, ...
        'offset',offset(2)*deg );

    L(3) = Revolute('d', d(3), 'a', a(3), 'alpha', alpha(3)*deg,  ...
         'offset',offset(3)*deg  );
    %Create the robot
    g8robot = SerialLink(L, 'name', 'G8 Robot Arm','base',transl(0,-0.08,0.047));
    figure(2)
    clf(2)
    g8robot.plot([0,0,0]);
    %Load measured distance data and 3 joint angles from file
    MeasData = load ('DistanceData.txt');
    %get the joint angles of each pose
    theta = deg2rad(MeasData(:,1:3));
    %get the measured distance of each pose
    MeasDist = MeasData(:,4)';

    %go through 9 poses to calculate the distance
    for i=1:9 
        poseT(i) = g8robot.fkine(theta(i,:));
        CalcDist(i) = sqrt((poseT(i).t(1))^2 + (poseT(i).t(2))^2 + (poseT(i).t(3))^2);
    end
    %calculate the avearge error between calculated distance and
    %measurement
    error = mean(abs(CalcDist - MeasDist))
end
