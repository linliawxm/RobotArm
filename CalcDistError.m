function error = CalcDistError(dh)
    %dh = [a1 a2 d1 d2 d3 offset1 offset2 baseY], other parameters are fixed,
    %don't need to optimize
    clear L
    deg = pi/180;

    %Load measured distance data and 3 joint angles from file
    MeasData = load ('DistanceData.txt');

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
    g8robot = SerialLink(L, 'name', 'G8 Robot Arm','base',[0,dh(8),0]);

    theta = deg2rad(MeasData(:,1:3));
    MeasDist = MeasData(:,4)';

    %go through 9 poses to calculate the distance
    for i=1:9 
        poseT(i) = g8robot.fkine(theta(i,:));
        CalcDist(i) = sqrt((poseT(i).t(1))^2 + (poseT(i).t(1))^2 + (poseT(i).t(1))^2);
    end

    error = mean(abs(CalcDist - MeasDist))
end
