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
        q = [0 0 0];     %robot pose angles(radian), [0 0 0] is initial pose
        T                %robot current postion transformation matrix
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
        %definition of servo Max PWM Signal Range
        servoPwmRange = [5.75*10^-4, 2.46*10^-3; %min max
                         5.75*10^-4, 2.46*10^-3;
                         6.40*10^-4, 2.25*10^-3];
        %definition of robot joint angle limits
        qlim = [-112 90; -90 112;0 165];
        %definition of servo Max Rotation degree
        servoRange = [202 202 165];
        
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
        function createRobot(obj)
            %Create robot model
            %Set Links paramters, then build robot using SerialLink, and
            %plot it with intial pose

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
            %Plot robot in figure
            obj.G8Robot.plot(obj.p000);
            %Update T and q of robot
            obj.T = obj.G8Robot.fkine(obj.p000);
            obj.q = obj.p000;
        end
        
        function plot(obj,q)
            %Plot robot model
            obj.G8Robot.plot(q);
            %Update T and q of robot
            obj.T = obj.G8Robot.fkine(q);
            obj.q = q;
        end
        
        function [result,warnMsg]= connectRobot(obj, port, pin)
            %Connect with Arduino hardware and set up servo            
            result = 0;
            if isempty(obj.arduinoHW)
                try
                    %Connect with arduino hardware
                    obj.arduinoHW = arduino(port, 'Uno', 'Libraries', 'Servo');
                catch ME
                    %Connection failed
                    warnMsg = ["Connect failure","Indentifier:",ME.identifier,"Message:",ME.message];
                    result = 1;
                    return;
                end
            end
            
            try
                if isempty(obj.arduinoHW) == false
                    obj.jointServo1 = servo(obj.arduinoHW, pin{1}, ...
                        'MinPulseDuration', obj.servoPwmRange(1,1), 'MaxPulseDuration', obj.servoPwmRange(1,2));
                    obj.jointServo2 = servo(obj.arduinoHW, pin{2}, ...
                        'MinPulseDuration', obj.servoPwmRange(2,1), 'MaxPulseDuration',obj.servoPwmRange(2,2));
                    obj.jointServo3 = servo(obj.arduinoHW, pin{3}, ...
                        'MinPulseDuration', obj.servoPwmRange(3,1), 'MaxPulseDuration', obj.servoPwmRange(3,2));
                    %keep robot arm pose same as model
                    controlMotor(obj,obj.q);
                    turnPenUp(obj);
                end
                
            catch ME
                %Servo connection failed
                warnMsg = ["Servo failure","Indentifier:",ME.identifier,"Message:",ME.message]
                result = 2;
                return;
            end
           warnMsg = 'Arduino connected'
        end
        
        function drawGrid(obj,gridPoints)
            %draw game board grid with 2 vertical lines and 2 horizontal
            %lines
            %for i = 1:1:4
                %Draw lines using calculated trajectory, adjustment for 3rd
                %joint
                %obj.drawOldLine([gridPoints(i,1) gridPoints(i,2) 0],[gridPoints(i,3) gridPoints(i,4) 0]);
            %end
            obj.drawLine("Left",[gridPoints(1,1) gridPoints(1,2) 0],[gridPoints(1,3) gridPoints(1,4) 0]);
            obj.drawLine("Right",[gridPoints(2,1) gridPoints(2,2) 0],[gridPoints(2,3) gridPoints(2,4) 0]);
            obj.drawLine("Upper",[gridPoints(3,1) gridPoints(3,2) 0],[gridPoints(3,3) gridPoints(3,4) 0]);
            obj.drawLine("Lower",[gridPoints(4,1) gridPoints(4,2) 0],[gridPoints(4,3) gridPoints(4,4) 0]);
        end
        
        function drawTrajWithFile(obj, filename, color)
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
                    plot3(m(i,1),m(i,2),0,'.','Color',color);
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
        
        function drawO(obj,square) 
            %Draw a circle on the board
            if ~obj.isUseFileTraj
                %calculate the points of circle
                N = (0:1:40)'; 
                theta = ( N/N(end) )*2*pi;
                points = (obj.center(square,1:3) + obj.r*[cos(theta) sin(theta) zeros(size(theta))]);

                angle3 = 0.111;
                %Move to first point of circle
                obj.turnPenUp();
                obj.movePenTo([points(1,1), points(1,2), obj.T.t(3)]);
                obj.turnPenDown(angle3);

                %draw first line trajectory with p
                m = drawTraj(obj, points, angle3, "blue");
                
                %save the trajectory into file
                if obj.isRecordTraj
                    filename = sprintf("Traj\\Circle%d.txt", square);
                    writematrix(roundn(m,-4),filename);
                end
            else
                %Draw circle trajectory using data from file
                filename = sprintf("Traj\\Circle%d.txt", square);
                drawTrajWithFile(obj, filename, "blue");
            end
            obj.turnPenUp();
            
            %update board info
            obj.board(obj.center(square,4),obj.center(square,5)) = 0;
            obj.step = obj.step + 1;
        end
        
        function drawX(obj,square)
            %Draw a cross on the board
            
            if ~obj.isUseFileTraj
                %calculate the points of X
                x = obj.center(square,1)-obj.r:0.001:obj.center(square,1)+obj.r;
                y1 = x-obj.center(square,1)+obj.center(square,2);
                y2 = -x+obj.center(square,1)+obj.center(square,2);
                angle3 = 0.111;
                m = zeros([ size(x)*2+2 6]);
                p = [x;y1;ones(1,length(x))*obj.T.t(3)]';
                
                %Move to first point of first line
                obj.turnPenUp();
                obj.movePenTo([x(1), y1(1),obj.T.t(3)]);
                obj.turnPenDown();

                %draw first line trajectory with p
                m(1:size(x),:) = drawTraj(obj, p, angle3, "red");
                

                %Move to first point of the second line
                obj.turnPenUp();
                m(size(x)+1,:) = m(size(x),:);
                m(size(x)+1,6) = 1;   %Move pen up, for recording
                obj.movePenTo([obj.center(square,1)-obj.r, obj.center(square,2)+obj.r, obj.T.t(3)]);
                obj.turnPenDown();
                m(size(x)+2,:) = [obj.T(1) obj.T(2) obj.q(1) obj.q(2) obj.q(3) 1]; %Move pen up, for recording
                
                %draw second line trajectory with p
                p = [x;y2;ones(1,length(x))*obj.T.t(3)]';
                m(size(x)+2:size(x)*2+2,:) = drawTraj(obj, p, angle3, "red");
                
                if obj.isRecordTraj
                    filename = sprintf("Traj\\Cross%d.txt", square);
                    writematrix(roundn(m,-4),filename);
                end
            else
                filename = sprintf("Traj\\Cross%d.txt", square);
                drawTrajWithFile(obj, filename, "red");
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
            qf = obj.G8Robot.ikine(Tf,'q0',obj.q,'mask',[1 1 1 0 0 0]);
            if (isempty(qf) == false)
                [qx,qd,qdd]=mtraj(@tpoly, obj.q, qf, 8);
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
        
        function drawLine(obj, lineType,startPoint, endPoint)
            %draw a line on the board
            %startPoint: [x, y, z]
            %endPoint:   [x, y, z]
            if ~obj.isUseFileTraj
                %Compute the trajectory
                x = linspace(startPoint(1),endPoint(1),40);
                y = linspace(startPoint(2),endPoint(2),40);

                %generate the trajectory points
                p = [x;y;ones(1,length(x))*obj.T.t(3)]';

                angle3 = 0.111;
                %Move to the start point
                turnPenUp(obj);
                movePenTo(obj,startPoint);
                turnPenDown(obj,angle3);
                
                %draw the trajectory with p
                m = drawTraj(obj, p, angle3, "black");
                if obj.isRecordTraj
                    filename = sprintf("Traj\\%sline.txt", lineType);
                    writematrix(roundn(m,-4),filename);
                end
            else
                filename = sprintf("Traj\\%sline.txt", lineType);
                drawTrajWithFile(obj, filename, "black");
            end
            
            %At the end of drawing, turn pen up
            turnPenUp(obj);
        end
                
        function m = drawTraj(obj, p, angle3, color)
            %draw trajectory with P (N x 3) matrix, N points of trajectory,
            %one point (x, y, z)
            [rows columns] = size(p)
            m = zeros([rows 6])
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
                        plot3(Tx.t(1),Tx.t(2),0,'.','Color',color);
                    end
                    hold on
                    %update the robot model
                    obj.G8Robot.plot(qx);

                    %Drive motor to target position
                    obj.controlMotor(qx);
                    %update 3rd joint servo
                    if isempty(obj.jointServo3) == false
                        writePosition(obj.jointServo3, angle3);
                    end
                        
                    Tx = obj.G8Robot.fkine(qx);
                    %Update the robot joint angles and transformation matrix
                    obj.q = qx;
                    obj.T = Tx;
                else
                    %skip this point, then try next point
                end
                m(i,:) = [p(i,1) p(i,2) qx(1) qx(2) qx(3) angle3];
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
           
            row_sum = transpose(sum(obj.board,2));
            r1 = row_sum(1); % position 1 4 7
            r2 = row_sum(2); % position 2 5 8
            r3 = row_sum(3); % position 3 6 9
            column_sum = sum(obj.board);
            c1 = column_sum(1); % position 1 2 3
            c2 = column_sum(2); % position 4 5 6
            c3 = column_sum(3); % position 7 8 9
            d1 = trace(obj.board); % position 1 5 9
            d2 = trace(fliplr(obj.board)); % position 3 5 7

            sum_set = [r1 r2 r3 c1 c2 c3 d1 d2];
            position_set = [1 4 7; 2 5 8; 3 6 9; 1 2 3; 4 5 6; 7 8 9; 1 5 9; 3 5 7; ];
            search_set = [obj.board(1,1) obj.board(1,2) obj.board(1,3);
                obj.board(2,1) obj.board(2,2) obj.board(2,3);
                obj.board(3,1) obj.board(3,2) obj.board(3,3);
                obj.board(1,1) obj.board(2,1) obj.board(3,1);
                obj.board(1,2) obj.board(2,2) obj.board(3,2);
                obj.board(1,3) obj.board(2,3) obj.board(3,3);
                obj.board(1,1) obj.board(2,2) obj.board(3,3);
                obj.board(3,1) obj.board(2,2) obj.board(1,3);];
            corner_set = [1 3 7 9];

            if obj.board(2,2)==255
                position = 5;
            %     elseif board(2,2) == 1
            %         position = corner_set(randi(4));
            else
                onex = []; % a set to store the row or column that has 1 x and two empty cells.
                for i=1:8
                    % two key cases two x or two o in one single line
                    % we will win
                    if sum_set(i) == 255
                        x = find(search_set(i,:) == 255);
                        position = position_set(i,x(1));
                        return
                    % defence,otherwise we will loose 
                    elseif sum_set(i) == 257
                        y = find(search_set(i,:) == 255);
                        position = position_set(i,y(1));
                        return
                    % check if there is a line with only one x in it, if so, record
                    % them in onex. Because they may win on this line later, image
                    % that we do not have a o in that line.So we want to put our o
                    % in this line if possible.
                    elseif sum_set(i) == 511
                        onex(end+1)= i;
                    end % end if                
                end % end for loop

                % for instance, this is a board and its corresponding cell number.             
                % | 1   255  255  |         1  4  7
                % |255   0   255  |         2  5  8
                % |255   1   255  |         3  6  9
                % onex = [1 3 4], row1 row3 colomn1 
                % they have only one x and two empty cells, value = 511 =
                % 255*2+1. for loop 1 to 3,use index onx(i) to find the
                % postition of empty cells.
                % onex_empty_flag = isempty(onex);

                % if onex is empty, which means most cells have already been
                % occupied, we need to randomly pick one empty cell.
                onexemp = isempty(onex);
                if isempty(onex) == true
                    position = empty(randi(length(empty)));
                else
                    % if onex is not empty, we check if there are empty corner
                    % cells on each line recored in onex. we increment the
                    % frequency of each corner cell from 0 when we find the line's
                    % empty corners (1 or 2) contain it.
                    f1 = 0;
                    f3 = 0;
                    f7 = 0;
                    f9 = 0;
                    for i = 1:length(onex)
                        % empty_index_set store the index of the empty cells
                        % inside one line, while onex(i) is the line index in
                        % the position set and search_set
                        empty_index_set= find(search_set(onex(i),:)==255);                    
                        % check empty corner cells
                        % check highest frequency of each corner cell and pick that
                        % one
                        len1 = length(empty_index_set);
                        for j = 1:len1
                            p = position_set(onex(i),empty_index_set(j));
                            if p == 1
                                f1 = f1 + 1;
                            elseif p == 3
                                f3 = f3+ 1;
                            elseif p == 7
                                f7 = f7 + 1;
                            elseif p == 9
                                f9 = f9 + 1;

                            end
                        end
                    end % end for loop
                    f = [f1 f3 f7 f9];
                    % one corner cells cannot be contained by more than two
                    % lines. so we firstly check frequency at 2, then if no
                    % corner is contained by two lines, check the frequency at
                    % 1.
                    f_index_set=find(f == 2);

                    if isempty(f_index_set) == true
                        f_index_set=find(f == 1);
                    end
                    selected_index = f_index_set(randi(length(f_index_set)));
                    position = corner_set(selected_index);


                end

            end
        end

    end
end

