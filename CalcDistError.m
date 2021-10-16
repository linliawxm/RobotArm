%Objective function to calculate the end effector distance from the
%clipboard coordinate system origin
%Input parameter:
% DH parameters of 3 links robot
% dh = [a1 a2 a3;
%       alpha1 alpha2 alpha3; 
%       d1 d2 d3; 
%       offset1 offset2 offset3;
%       baseX baseY baseZ] 
function error = CalcDistError(dh)
    clear L
    deg = pi/180;
    
    %build parameter a
    a=[dh(1,1);dh(1,2);dh(1,3)];
    %build parameter alpha
    alpha=[dh(2,1);dh(2,2);dh(2,3)];
    %build parameter d
    d=[dh(3,1);dh(3,2);dh(3,3)];
    %4th column are parameter alpha
    offset=[dh(4,1);dh(4,2);dh(4,3)];
    baseX = dh(5,1);
    baseY = dh(5,2);
    baseZ = dh(5,3);
    
    %define the links of robot
    L(1) = Revolute('d', d(1), 'a', a(1),'alpha', alpha(1)*deg, ... 
        'offset',offset(1)*deg ); 

    L(2) = Revolute('d', d(2), 'a', a(2), 'alpha', alpha(2)*deg, ...
        'offset',offset(2)*deg );

    L(3) = Revolute('d', d(3), 'a', a(3), 'alpha', alpha(3)*deg,  ...
         'offset',offset(3)*deg  );
    
    %Create the robot
    g8robot = SerialLink(L, 'name', 'G8 Robot Arm','base',transl(baseX,baseY,baseZ));
    figure(2)
    clf(2)
    %Plot the robot
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
        %T.t = [x;y;z]
        CalcDist(i) = sqrt((poseT(i).t(1))^2 + (poseT(i).t(2))^2 + (poseT(i).t(3))^2);
    end
    %calculate the avearge error between calculated distance and
    %measurement
    error = mean(abs(CalcDist - MeasDist))
end
