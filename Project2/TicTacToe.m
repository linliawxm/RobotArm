classdef TicTacToe<handle
    % This is class for TicTacToe game with Robot Arm
    % Methods:
    %   createRobot       Create a robot model with SerialLink
    %   connectRobot      Connect to Arduino and servo
    %   drawGrid          draw game board's grid
    %   drawO             draw O on specific square
    %   drawX             draw X on specific square
    %   drawLine          draw a Line on the board, used by drawGrid
    %   turnPenUp         turn pen up
    %   turnPenDown       turn pen down
    %   movePenTo         move pen to specific location
    %   controlMotor      control servo motors angle same as model's q 
    %   computeNextMove   compute next move for the game
    %   checkForWin       check if the game is finished and who wins
    
    properties
        G8Robot SerialLink  %robot model object
        q = [0 0 0];     %robot pose angles, [0 0 pi/2] is initial pose
        T                   %robot current postion transformation matrix
        %Initial DH parameters, can be improved by optimization
        DhPara = [0.103 0.09 0;      %a1 a2 a3
                  0   -90   0;      %alpha1 alpha2 alpha3
                  0   0.02  0.029;   %d1 d2 d3
                  -19 18.5  0;      %offset1 offset2 offset2
                  0   -0.08 0.076]; %basex basey basez
         %definition of game board
        grid = [0.16 -0.04 0.04 -0.04;  %startX startY endX endY 
                0.16 -0.08 0.04 -0.08; 
                0.12  0.00 0.12 -0.12; 
                0.08  0.00 0.08 -0.12];
        center = [0.14 -0.02 0 1 1;   %Sqaure1(x y z,row column)
                  0.10 -0.02 0 2 1;   %Sqaure2
                  0.06 -0.02 0 3 1;   %Sqaure3
                  0.14 -0.06 0 1 2;   %Sqaure4
                  0.10 -0.06 0 2 2;   %Sqaure5
                  0.06 -0.06 0 3 2;   %Sqaure6
                  0.14 -0.10 0 1 3;   %Sqaure7
                  0.10 -0.10 0 2 3;   %Sqaure8
                  0.06 -0.10 0 3 3]   %Sqaure9
        r=0.01; %unit=meter
        %Hardware related variables
        arduinoHW               %arduino hardware object
        pin = {"D3" "D5" "D6"}  %servo motor control pin
        jointServo1             %joint1 servo object
        jointServo2             %joint2 servo object
        jointServo3             %joint3 servo object
        
        %Game related variables    
        board = ones(3)*255;    %Initial game board, each cell initial value is 255
        step = 0                %game steps, less than 10
        state = "initial"       %game states: ready,running, done, terminated
        player = "user"         %whose turn to play
        
        isRecordTraj = 1
        isDrawMoveTraj = 1
        isUseFileTraj = 1
        
    end
    
    properties (Constant)
        %definition of robot joint angle limits
        qlim = [-112 90; -90 112;0 165];
        %definition of servo Max Rotation degree
        servoRange = [202 202 165];
        %definition of servo Max PWM Signal Range
        servoPwmRange = [5.75*10^-4, 2.46*10^-3; %min max
                         5.75*10^-4, 2.46*10^-3;
                         6.40*10^-4, 2.25*10^-3];
       
        p000 = [0 0 0]           %Pose1
        p090 = [0 pi/2 0]        %Pose2
        p0_90 = [0 -pi/2 0]      %Pose3
        p900 = [pi/2 0 0]        %Pose4
        p990 = [pi/2 pi/2 0]     %Pose5
        p9_90 = [pi/2 -pi/2 0]   %Pose6
        p_900 = [-pi/2 0 0]      %Pose7
        p_990 = [ -pi/2 pi/2 0]  %Pose8
        p_9_90 = [-pi/2 -pi/2 0] %Pose9
        p009 = [0 0 pi/2]        %Pose10
    end
    
    methods        
        function obj = createRobot(obj)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here

            % Define links of robot
            L(1) = Revolute('d', obj.DhPara(3,1), ...   % link length (Dennavit-Hartenberg notation)
                'a', obj.DhPara(1,1), ...               % link offset (Dennavit-Hartenberg notation)
                'alpha', deg2rad(obj.DhPara(2,1)), ...       % link twist (Dennavit-Hartenberg notation)
                'qlim', obj.qlim(1,:), ...                      % minimum and maximum joint angle
                'offset',deg2rad(obj.DhPara(4,1))); 
            
            L(2) = Revolute('d', obj.DhPara(3,2), ...
                'a', obj.DhPara(1,2), ...
                'alpha', deg2rad(obj.DhPara(2,2)), ...
                'qlim', obj.qlim(2,:), ...
                'offset',deg2rad(obj.DhPara(4,2)));
            
            L(3) = Revolute('d', obj.DhPara(3,3), ...
                'a', obj.DhPara(1,3), ...
                'alpha', deg2rad(obj.DhPara(2,3)),  ...
                'qlim', obj.qlim(3,:), ...
                'offset',deg2rad(obj.DhPara(4,3)));

            %Build the robot model
           obj.G8Robot = SerialLink(L, 'name', 'G8 Robot Arm', ...
                'tool',transl(0,0,0), ...
                'base',transl(obj.DhPara(5,1), obj.DhPara(5,2),obj.DhPara(5,3)));
           obj.G8Robot.plot(obj.p000);
           obj.T = obj.G8Robot.fkine(obj.p000);
           obj.q = obj.p000;
        end
        
        function [result,warnMsg]= connectRobot(obj, port, pin)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            
            result = 0;
            try
                obj.arduinoHW = arduino(port, 'Uno', 'Libraries', 'Servo');
            catch ME
                warnMsg = ["Connect failure","Indentifier:",ME.identifier,"Message:",ME.message]
                result = 1;
                return;
            end
            
            try
                if isempty(obj.arduinoHW) == false
                    obj.jointServo1 = servo(obj.arduinoHW, pin{1}, ...
                        'MinPulseDuration', obj.servoPwmRange(1,1), 'MaxPulseDuration', obj.servoPwmRange(1,2));
                    if isempty(obj.jointServo1) == false
                        
                        %clip data between joint limits
                        angle1 = obj.clip(rad2deg(obj.q(1)),obj.qlim(1,1),obj.qlim(1,2));

                        %change degree to servo input range(0,1)
                        angle1 = (obj.qlim(1,2) - angle1)/obj.servoRange(1);
                        %clip the data between 0 and 1
                        angle1 = obj.clip(angle1,0,1);
                        writePosition(obj.jointServo1, angle1);
                    end
                    
                    obj.jointServo2 = servo(obj.arduinoHW, pin{2}, ...
                        'MinPulseDuration', obj.servoPwmRange(2,1), 'MaxPulseDuration',obj.servoPwmRange(2,2));
                    if isempty(obj.jointServo2) == false
                        
                        %clip data between joint limits
                        angle2 = obj.clip(rad2deg(obj.q(2)),obj.qlim(2,1),obj.qlim(2,2));

                        %change degree to servo input range(0,1)
                        angle2 = (obj.qlim(2,2) - angle2)/obj.servoRange(2);
                        %clip the data between 0 and 1
                        angle2 = obj.clip(angle2,0,1);
                        writePosition(obj.jointServo2, angle2);
                    end
                    obj.jointServo3 = servo(obj.arduinoHW, pin{3}, ...
                        'MinPulseDuration', obj.servoPwmRange(3,1), 'MaxPulseDuration', obj.servoPwmRange(3,2));
                    if isempty(obj.jointServo3) == false
                        
                        %clip data between joint limits
                        angle3 = obj.clip(rad2deg(obj.q(3)),obj.qlim(3,1),obj.qlim(3,2));

                        %change degree to servo input range(0,1)
                        angle3 = (obj.qlim(3,2) - angle3)/obj.servoRange(3);
                        %clip the data between 0 and 1
                        angle3 = obj.clip(angle3,0,1);
                        writePosition(obj.jointServo3, angle3);
                    end
                end
                
            catch ME
                warnMsg = ["Servo failure","Indentifier:",ME.identifier,"Message:",ME.message]
                result = 2;
                return;
            end
           warnMsg = 'Arduino connected'
           obj.q
           controlMotor(obj,obj.q);
        end
        
        function drawGrid(obj,gridPoints)
            %draw 4 lines
%             for i = 1:1:4
%                 obj.drawLine([gridPoints(i,1) gridPoints(i,2) 0],[gridPoints(i,3) gridPoints(i,4) 0]);
%             end
            obj.drawLine("Left",[gridPoints(1,1) gridPoints(1,2) 0],[gridPoints(1,3) gridPoints(1,4) 0]);
            obj.drawLine("Right",[gridPoints(2,1) gridPoints(2,2) 0],[gridPoints(2,3) gridPoints(2,4) 0]);
            obj.drawLine("Upper",[gridPoints(3,1) gridPoints(3,2) 0],[gridPoints(3,3) gridPoints(3,4) 0]);
            obj.drawLine("Lower",[gridPoints(4,1) gridPoints(4,2) 0],[gridPoints(4,3) gridPoints(4,4) 0]);
        end
        
        function drawO(obj,square) 
            %Draw a circle on the board
            if ~obj.isUseFileTraj
                %calculate the points of circle
                N = (0:0.5:40)'; 
                theta = ( N/N(end) )*2*pi;
                points = (obj.center(square,1:3) + obj.r*[cos(theta) sin(theta) zeros(size(theta))]);

                angle3 = ones([1 size(theta)])*65;
                angle3 = (obj.qlim(3,2) - angle3)/obj.servoRange(3);

                %Move to first point of circle
                obj.turnPenUp();
                obj.movePenTo([points(1,1), points(1,2), obj.T.t(3)]);
                obj.turnPenDown(angle3(1));

                qx = obj.q;
                Tx = obj.T;
                for i = 1:1:size(theta)
                    %Calculate current point position angles
                    Tx.t(1) = points(i,1);
                    Tx.t(2) = points(i,2);
                    q_temp = obj.G8Robot.ikine(Tx,'q0',qx,"mask",[1 1 1 0 0 0]);
                    if isempty(q_temp) == false
                       qx= q_temp;

                       %plot trajectory in the model
                       hold on
                       if obj.isDrawMoveTraj
                           plot3(points(i,1),points(i,2),points(i,3),'.','Color','blue');
                       end

                       %re-plot robot model
                       obj.G8Robot.plot(qx);
                       %control motors to target position
                       obj.controlMotor(qx);
                       %update the real motor
                       if isempty(obj.jointServo3) == false
                           writePosition(obj.jointServo3, angle3(i));
                       end

                       %update robot properties
                       obj.q = qx;
                       Tx = obj.G8Robot.fkine(qx);
                       obj.T = Tx;
                    else
                        %ikine optimization failed, skip this point
                    end
                    if obj.isRecordTraj
                        m(i,:) = [points(i,1) points(i,2) qx(1) qx(2) qx(3) angle3(i)];
                    end
                end

                if obj.isRecordTraj
                    filename = sprintf("Circle%d.txt", square);
                    writematrix(roundn(m,-4),filename);
                end
            else
                filename = sprintf("Circle%d.txt", square)
                m = readmatrix(filename);
                [rows, columns] = size(m);
                
                %Move to the start point
                turnPenUp(obj);
                startPoint = [m(1,1) m(1,2) obj.T.t(3)];
                movePenTo(obj,startPoint);
                angle3 = m(1,6);
                %clip the data between 0 and 1
                angle3 = obj.clip(angle3,0,1);
                turnPenDown(obj,angle3);
                
                for i = 1:rows
                    if obj.isDrawMoveTraj
                        %plot the trajectory
                        plot3(m(i,1),m(i,2),0,'.','Color','black');
                    end
                    hold on
                    %update the robot model
                    qx = [m(i,3),m(i,4),m(i,5)];
                    obj.G8Robot.plot(qx);
                    %Drive motor to target position
                    obj.controlMotor(qx);
                    %update the real motor
                    if isempty(obj.jointServo3) == false
                        writePosition(obj.jointServo3, m(i,6));
                    end
                    obj.q = qx;
                end
                obj.T = obj.G8Robot.fkine(qx);
            end
            obj.turnPenUp();
            
            %update board info
            obj.board(obj.center(square,4),obj.center(square,5)) = 0;
            obj.step = obj.step + 1;
        end
        
        function drawX(obj,square)
            %Draw a circle on the board
            
            if ~obj.isUseFileTraj
                %calculate the points of X
                x = obj.center(square,1)-obj.r:0.001:obj.center(square,1)+obj.r;
                y1 = x-obj.center(square,1)+obj.center(square,2);
                y2 = -x+obj.center(square,1)+obj.center(square,2);

                %Move to first point of first line
                obj.turnPenUp();
                obj.movePenTo([x(1), y1(1),obj.T.t(3)]);
                obj.turnPenDown();

                %draw first line of X
                qx = obj.q;
                Tx = obj.T;
                len_x = length(x);
                for i = 1:1:len_x
                    %Calculate current point position angles
                    Tx.t(1) = x(i);
                    Tx.t(2) = y1(i);
                    q_temp = obj.G8Robot.ikine(Tx,qx,"mask",[1 1 1 0 0 0]);
                    if isempty(q_temp) == false
                        qx= q_temp;

                        hold on
                        if obj.isDrawMoveTraj
                            plot3(x(i),y1(i),0,'.','Color','red');
                        end
                        %re-plot robot model
                        obj.G8Robot.plot(qx);
                        %control motors to target position
                        obj.controlMotor(qx);
                        %update the pen motor
                        if isempty(obj.jointServo3) == false
                            writePosition(obj.jointServo3, 0.5);
                        end
                       
                        %update robot properties
                        obj.q = qx;
                        Tx = obj.G8Robot.fkine(qx);
                        obj.T = Tx;
                    else
                        %ikine optimization failed, skip this point
                    end
                    if obj.isRecordTraj
                        m(i,:) = [x(i),y1(i) qx(1) qx(2) qx(3) 0.5];
                    end
                end

                %Move to first point of the second line
                obj.turnPenUp();
                obj.movePenTo([obj.center(square,1)-obj.r, obj.center(square,2)+obj.r, obj.T.t(3)]);
                obj.turnPenDown();
                %draw second line of X
                %draw first line of X
                qx = obj.q;
                Tx = obj.T;
                
                for i = 1:1:len_x
                    %Calculate current point position angles
                    Tx.t(1) = x(i);
                    Tx.t(2) = y2(i);
                    q_temp = obj.G8Robot.ikine(Tx,qx,"mask",[1 1 1 0 0 0]);
                    if isempty(q_temp) == false
                        qx= q_temp;

                        hold on
                        if obj.isDrawMoveTraj
                            plot3(x(i),y2(i),0,'.','Color','red');
                        end
                        %re-plot robot model
                        obj.G8Robot.plot(qx);
                        %control motors to target position
                        obj.controlMotor(qx);
                        %update the pen motor
                        if isempty(obj.jointServo3) == false
                            writePosition(obj.jointServo3, 0.5);
                        end
                        %update robot properties
                        obj.q = qx;
                        Tx = obj.G8Robot.fkine(qx);
                        obj.T = Tx;
                    else
                        %ikine optimization failed, skip this point
                    end
                    m(len_x+i,:) = [x(i),y2(i) qx(1) qx(2) qx(3) 0.5];
                end
                if obj.isRecordTraj
                    filename = sprintf("Cross%d.txt", square);
                    writematrix(roundn(m,-4),filename);
                end
            else
                filename = sprintf("Cross%d.txt", square)
                m = readmatrix(filename);
                [rows, columns] = size(m);
                
                %Move to the start point
                turnPenUp(obj);
                startPoint = [m(1,1) m(1,2) obj.T.t(3)];
                movePenTo(obj,startPoint);
                angle3 = m(1,6);
                %clip the data between 0 and 1
                angle3 = obj.clip(angle3,0,1);
                turnPenDown(obj,angle3);
                
                for i = 1:rows
                    if obj.isDrawMoveTraj
                        %plot the trajectory
                        plot3(m(i,1),m(i,2),0,'.','Color','black');
                    end
                    hold on
                    %update the robot model
                    qx = [m(i,3),m(i,4),m(i,5)];
                    obj.G8Robot.plot(qx);
                    %Drive motor to target position
                    obj.controlMotor(qx);
                    %update the real motor
                    if isempty(obj.jointServo3) == false
                        writePosition(obj.jointServo3, m(i,6));
                    end
                    obj.q = qx;
                end
                obj.T = obj.G8Robot.fkine(qx);
            end
            obj.turnPenUp();
            
            %update board info
            obj.board(obj.center(square,4),obj.center(square,5)) = 1;
            obj.step = obj.step + 1;
        end
  
        function turnPenUp(obj)
            %update the model
            if isempty(obj.G8Robot) == false
                obj.q(3) = 0;
                obj.G8Robot.plot(obj.q);
            end
            %update the real motor
            if isempty(obj.jointServo3) == false
                writePosition(obj.jointServo3, 1);
            end
        end
        
        function turnPenDown(obj,sq)
            if nargin > 1
                if sq > 1
                    sq = 1;
                elseif sq < 0
                    sq = 0;
                end
                defaultSq = sq;
            else
                defaultSq = 0.6;
            end
            %update the model
            if isempty(obj.G8Robot) == false
                obj.q(3) = defaultSq*(-pi/2);
                obj.G8Robot.plot(obj.q);
            end
            %update the real motor
            if isempty(obj.jointServo3) == false
                writePosition(obj.jointServo3, defaultSq);
            end
        end
        
        function a = clip(obj,x,low,high)
            a = min(max(x,low),high);
        end
        
        function movePenTo(obj, location)
            %Move pen from current location to target location(x,y)
            %Get start point T matrix
            obj.q
            Tf = obj.G8Robot.fkine(obj.q);
            
            %Set target position
            Tf.t(1) = location(1);
            Tf.t(2) = location(2);
            %flag = 0;
            %get the joint angle using ikine
            Tf.t
            obj.q
            test = 1
            qf = obj.G8Robot.ikine(Tf,'q0',obj.q,'mask',[1 1 1 0 0 0]);
            test = 2
            if (isempty(qf) == false)
                test = 3
                [qx,qd,qdd]=mtraj(@tpoly, obj.q, qf, 20);
                test = 4
                for i = 1:size(qx)
                    obj.q = qx(i,:);
                    obj.T = obj.G8Robot.fkine(obj.q);
                    if obj.isDrawMoveTraj
                        plot3(obj.T.t(1),obj.T.t(2),obj.T.t(3),'.','Color','yellow');
                    end
                    hold on
                    %Update on model
                    obj.G8Robot.plot(qx(i,:));

                    %Drive motor to target position
                    obj.controlMotor(qx(i,:));
                end
            else
                %flag = 1; %ikine failed
            end
        end
        
        function drawOldLine(obj, startPoint, endPoint)
            %draw a line on the board
            %startPoint: [x, y, z]
            %endPoint:   [x, y, z]
            
            %Move to the start point
            turnPenUp(obj);
            movePenTo(obj,startPoint);
            turnPenDown(obj);
            
            %Compute the trajectory
            x = linspace(startPoint(1),endPoint(1),40);
            y = linspace(startPoint(2),endPoint(2),40);
            
            %generate the trajectory points
            p = [x;y;ones(1,length(x))*obj.T.t(3)]'
            %draw the trajectory with p
            drawTraj(obj, p);
            
            %At the end of drawing, turn pen up
            turnPenUp(obj);
        end
        
        function drawLine(obj, lineType,startPoint, endPoint)
            %draw a line on the board
            %startPoint: [x, y, z]
            %endPoint:   [x, y, z]
            if ~obj.isUseFileTraj
%                 if lineType == "Left"
%                     startPoint = [obj.grid(1,1) obj.grid(1,2)];
%                     endPoint = [obj.grid(1,3) obj.grid(1,4)];
%                 elseif lineType == "Right"
%                     startPoint = [obj.grid(2,1) obj.grid(2,2)];
%                     endPoint = [obj.grid(2,3) obj.grid(2,4)];
%                 elseif lineType == "Upper"
%                     startPoint = [obj.grid(3,1) obj.grid(3,2)];
%                     endPoint = [obj.grid(3,3) obj.grid(3,4)];
%                 else %if lineType == "Lower"
%                     startPoint = [obj.grid(4,1) obj.grid(4,2)];
%                     endPoint = [obj.grid(4,3) obj.grid(4,4)];
%                 end

                %Compute the trajectory
                x = linspace(startPoint(1),endPoint(1),40);
                y = linspace(startPoint(2),endPoint(2),40);

                %generate the trajectory points
                p = [x;y;ones(1,length(x))*obj.T.t(3)]';
                if (lineType == "Right") || (lineType == "Left")
                    angle3 = linspace(45,100,40);
                else %(lineType == "Upper") || (lineType == "Lower")
                    angle3 = ones([1 40])*70;
                end
                angle3 = (obj.qlim(3,2) - angle3)/obj.servoRange(3);
                
                %Move to the start point
                turnPenUp(obj);
                movePenTo(obj,startPoint);
                turnPenDown(obj,angle3(1));
                
                %draw the trajectory with p
                %Initial transformation matrix and angles
                Tx = obj.T;
                qx = obj.q;
                %m = zeros([40 6]);
                %Loop each point for angles, plot robot and control the servo
                for i = 1:size(p)
                    %Move the next point
                    Tx.t(1) = p(i,1);
                    Tx.t(2) = p(i,2);
                    q_temp = obj.G8Robot.ikine(Tx,'q0',qx,'mask',[1 1 1 0 0 0]);
                    if isempty(qx) == false
                        qx = q_temp;
                        if obj.isDrawMoveTraj
                            %plot the trajectory
                            plot3(Tx.t(1),Tx.t(2),0,'.','Color','black');
                        end
                        hold on
                        %update the robot model
                        obj.G8Robot.plot(qx);

                        %Drive motor to target position
                        obj.controlMotor(qx);
                        %update the real motor
                        if isempty(obj.jointServo3) == false
                            writePosition(obj.jointServo3, angle3(i));
                        end

                        Tx = obj.G8Robot.fkine(qx);
                        %Update the robot joint angles and transformation matrix
                        obj.q = qx;
                        obj.T = Tx;
                    else
                        %skip this point, then try next point
                    end
                    m(i,:) = [x(i) y(i) qx(1) qx(2) qx(3) angle3(i)];
                end
                if obj.isRecordTraj
                    filename = sprintf("%sline.txt", lineType);
                    writematrix(roundn(m,-4),filename);
                end
            else
                filename = sprintf("%sline.txt", lineType)
                m = readmatrix(filename);
                [rows, columns] = size(m);
                
                %Move to the start point
                turnPenUp(obj);
                startPoint = [m(1,1) m(1,2) obj.T.t(3)]
                movePenTo(obj,startPoint);
                angle3 = m(1,6);
                %clip the data between 0 and 1
                angle3 = obj.clip(angle3,0,1);
                turnPenDown(obj,angle3);
                
                for i = 1:rows
                    if obj.isDrawMoveTraj
                        %plot the trajectory
                        plot3(m(i,1),m(i,2),0,'.','Color','black');
                    end
                    hold on
                    %update the robot model
                    qx = [m(i,3),m(i,4),m(i,5)];
                    obj.G8Robot.plot(qx);
                    %Drive motor to target position
                    obj.controlMotor(qx);
                    %update the real motor
                    if isempty(obj.jointServo3) == false
                        writePosition(obj.jointServo3, m(i,6));
                    end
                    obj.q = qx;
                end
                obj.T = obj.G8Robot.fkine(qx);
            end
            
            %At the end of drawing, turn pen up
            turnPenUp(obj);
        end
                
        function drawTraj(obj, p)
            %draw trajectory with P (N x 3) matrix, N points of trajectory,
            %one point (x, y, z)
            
            %Initial transformation matrix and angles
            Tx = obj.T;
            qx = obj.q;
            %Loop each point for angles, plot robot and control the servo
            for i = 1:size(p)
                %Move the next point
                Tx.t(1) = p(i,1);
                Tx.t(2) = p(i,2);
                q_temp = obj.G8Robot.ikine(Tx,'q0',qx,'mask',[1 1 1 0 0 0]);
                if isempty(qx) == false
                    qx = q_temp;
                    if obj.isDrawMoveTraj
                        %plot the trajectory
                        plot3(Tx.t(1),Tx.t(2),0,'.','Color','black');
                    end
                    hold on
                    %update the robot model
                    obj.G8Robot.plot(qx);

                    %Drive motor to target position
                    obj.controlMotor(qx);
                    
                    Tx = obj.G8Robot.fkine(qx);
                    %Update the robot joint angles and transformation matrix
                    obj.q = qx;
                    obj.T = Tx;
                else
                    %skip this point, then try next point
                end
            end
        end
        
        function controlMotor(obj,q)
            %control motor to requested angle
            % q: requested angle vector
            if isempty(obj.jointServo1) == false
                %clip data between joint limits
                angle1 = obj.clip(rad2deg(q(1)),obj.qlim(1,1),obj.qlim(1,2));

                %change degree to servo input range(0,1)
                angle1 = (obj.qlim(1,2) - angle1)/obj.servoRange(1);
                %clip the data between 0 and 1
                angle1 = obj.clip(angle1,0,1);
                %control servo to right angle
                writePosition(obj.jointServo1, angle1);
            end

            if isempty(obj.jointServo2) ==false
                %clip data between joint limits
                angle2 = obj.clip(rad2deg(q(2)),obj.qlim(2,1),obj.qlim(2,2));

                %change degree to servo input range(0,1)
                angle2 = (obj.qlim(2,2) - angle2)/obj.servoRange(2);
                %clip the data between 0 and 1
                angle2 = obj.clip(angle2,0,1);
                %control servo to right angle
                writePosition(obj.jointServo2, angle2);
            end
            
        end
        
        function [done,who] = checkForWin(obj)
            %Summary: check who is winning
            %Logic: each cell initialized as 255, if one line sum is 3,
            %means X wins, sum is 0 means O wins, otherwise no winning.
            
            %Initialize
            done = false;
            who = 'none';
            %calculate column line sum
            column_sum = sum(obj.board);
            %calculate row line sum
            row_sum = sum(obj.board,2);
            %calculate diagonal line sum
            diag1 = trace(obj.board);
            %calculate reverse diagonal line sum
            diag2 = trace(fliplr(obj.board));
            
            if (ismember(3,column_sum) || ismember(3,row_sum) || (diag1 ==3) || (diag2 ==3))
                done = true;
                who = 'X';
            elseif (ismember(0,column_sum) || ismember(0,row_sum) || (diag1 ==0) || (diag2 ==0))
                done = true;
                who = 'O';
            else
            end
            
            %check if board is full, no one wins means draw
            if done == false
               if obj.step >= 9
                  done = true;
                  who = 'draw';
               end
            end
            
            if done == true
               obj.state = "done"; 
            end
            
        end
        
        function position = computeNextMove(obj)
            %obj.board have the info of current game
            %get empty space index info(random case)
            empty = find(obj.board == 255);
            if isempty(empty) == false
                position = empty(randi(length(empty)));
            else
                position = 0;
            end
        end

    end
end
