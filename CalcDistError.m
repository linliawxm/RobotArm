function error = CalcDistError(dh)
    clear L
    deg = pi/180;

    %Load measured distance data and 3 joint angles from file
    MeasData = load ('DistanceData.txt');

    %1st column are parameter a
    a=dh(:,1);
    %2nd column are parameter alpha
    alpha=dh(:,2);
    %3rd column are parameter alpha
    d=dh(:,3);
    %4th column are parameter alpha
    offset=dh(:,4);
    %define the links of robot
    L(1) = Revolute('d', d(1), 'a', a(1),'alpha', alpha(1)*deg, ... 
        'offset',offset(1)*deg ); 

    L(2) = Revolute('d', d(2), 'a', a(2), 'alpha', alpha(2)*deg, ...
        'offset',offset(2)*deg );

    L(3) = Revolute('d', d(3), 'a', a(3), 'alpha', alpha(3)*deg,  ...
         'offset',offset(3)*deg  );
    %Create the robot
    g8robot = SerialLink(L, 'name', 'G8 Robot Arm','base',[0,-0.08,0]);

    theta = deg2rad(MeasData(:,1:3));
    MeasDist = MeasData(:,4)';

    %go through 9 poses to calculate the distance
    for i=1:9 
        poseT(i) = g8robot.fkine(theta(i,:));
        CalcDist(i) = sqrt((poseT(i).t(1))^2 + (poseT(i).t(1))^2 + (poseT(i).t(1))^2);
    end

    error = mean(abs(CalcDist - MeasDist))
end
