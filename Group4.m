clc;
clear all;

% Installing connecting with ev3 robot
robot = legoev3('USB');

% Motor setup and initalisation with variables
gripper = motor(robot,'A');                               % Motor of the gripper
arm = motor(robot,'B');                                   % Motor of the arm
base = motor(robot,'C');                                  % Motor at the base

% sensor initalisation with variables
sens_base = touchSensor(robot,1);                         % Touch sensor at the Base
sens_arm = touchSensor(robot,3);                          % Touch sensor at the Arm
sens_gripper = sonicSensor(robot,2);                      % Ultra sonic sensor to measure distance

start(base)                                               % Base Motor startup
start(arm)                                                % Arm motor startup
start(gripper)                                            % Gripper Motor startup

% Station coordinates
a = [-90,0];                                              % Station A
b = [0,0];                                                % Station B
c = [90,0];                                               % Station C

% Check for station and to measure heights
signal = 0;
while(signal == 0)
    homing(sens_base,base,sens_arm,arm,gripper);          % robot homing function
    signal = 1;
end

% Task Sequence to be performed
picking = ['b','a','c','b','c','a'];                      % Defining sequence of Picking in a loop
placing =['a','c','b','c','a','b'];                       % Defining sequence of Placing in a loop

for loop = 1:1:6                                          % For loop of the picking and placing sequence 
    % Calling angle base functions based on the station
    if (picking(loop) == 'a')
        angle_base = a(1);   
    elseif (picking(loop) == 'b')
            angle_base = b(1);
    elseif (picking(loop) == 'c')
            angle_base = c(1);
    end
    pause(0.5)
  
    % Defining next picking station angle based on the placing station
    if (placing(loop) == 'a')
        if (picking(loop) == 'a')
            angle_pick = 0;
        elseif (picking(loop) == 'b')
                angle_pick = -90;
        elseif (picking(loop) == 'c')
                angle_pick = -180;
        end
  
   elseif (placing(loop) == 'b')
        if (picking(loop) == 'a')
             angle_pick = 90;
        elseif (picking(loop) == 'b')
                angle_pick = 0;
        elseif (picking(loop) == 'c')
                angle_pick = -90;
        end
           
    elseif (placing(loop) == 'c')
        if (picking(loop) == 'a')
            angle_pick = 180;
        elseif (picking(loop) == 'b')
                angle_pick = 90;
        elseif (picking(loop) == 'c')
                angle_pick = 0;
        end         
    end
    pause(0.5)
    
    move_base(base,angle_base);                           % Calling of Base location 
    pick_ball(arm,gripper,sens_arm,sens_gripper);         % Calling of pick function
    
    move_base(base,angle_pick);                           % Calling of Base location 
    place_ball(arm,gripper,sens_arm,sens_gripper);        % Calling of pick function
    
    homing(sens_base,base,sens_arm,arm,gripper);          % Calling of Home functionality
end

% Homing function to go to home position based on 2 touch sensors at base
% and arm to reach station A and then defining the rotation to move to
% Station B which is the home as per our defined objective
function homing(sens_base,base,sens_arm,arm,gripper)      
    while(readTouch(sens_arm)~=1)
          arm.Speed = -30;
    end
    arm.Speed=0;
    resetRotation(arm)

    while(readTouch(sens_base)~=1)
          base.Speed = 25;  
    end
    base.Speed=0;

    resetRotation(base)
    base_rotat=3*90;

    if(base_rotat >= (-readRotation(base)))
       while(base_rotat >=(-readRotation(base)))
             readRotation(base);
             base.Speed = -25;
       end
       base.Speed=0;
       resetRotation(base)
    end

    pause(0.5)
    resetRotation(base)
    resetRotation(arm)
    gripper.Speed = +5;
end

% Move base function to rotate the base between picking and placing station
function move_base(base,theta)                             
    base_rotat = 3 * theta;
    pidc(base_rotat,base);                                % Base Motor rotation with PI control
    resetRotation(base)
    base.Speed=0;
    pause(0.5)
end

% Pick Ball function for functioning of arm and gripper based on the requirement
% Includes reading of the distance at the particulat station and call inverse kinematics funtion to ontain arm angle 
function pick_ball(arm,gripper,sens_arm,sens_gripper)
    resetRotation(arm)
    h = readdis(sens_gripper);
    pause(0.5)
    angle_arm = double(inv_kin_cal(h));

    gripper.Speed = -10;                                  % gripper opening
    pause(0.4)
    gripper.Speed = 0;   
    
    armc(angle_arm, arm);                                 % Moving arm down with PI control
    arm.Speed = 0;

    gripper.Speed = 20;                                   % gripper closing
    pause(0.4)
    gripper.Speed = 0;
    
    while(readTouch(sens_arm)~=1)                         % Moving up of arm
          arm.Speed = -40;
    end
    arm.Speed = 0;
    resetRotation(arm)  
end

% Place Ball function for functioning of arm and gripper based on the requirement
% Includes reading of the distance at the particulat station and call inverse kinematics funtion to ontain arm angle
function place_ball(arm,gripper,sens_arm,sens_gripper)                 
    resetRotation(arm);
    h = readdis(sens_gripper);
    pause(0.5)
    angle_arm_f = double(inv_kin_cal(h));
    armd(angle_arm_f, arm);                               % Moving down of arm with PI control
    arm.Speed = 0;

    gripper.Speed = -10;                                  % gripper opening
    pause(0.4)
    gripper.Speed = 0;
    
    while(readTouch(sens_arm)~=1)                         % Moving up of arm
        arm.Speed = -30;
    end
    arm.Speed = 0;
    resetRotation(arm);
    
    gripper.Speed= 20;                                    % gripper closing
    pause(0.4)
    gripper.Speed=0;
end

% Distance function to read the ultrasonic distance at the station
function distance = readdis(sens_gripper)
    distance = readDistance(sens_gripper);    
end

% Inverse kinematics approach
function joint_angle = inv_kin_cal(height)                         
    offset = 60;
    link0 = 70;
    link1 = 50;
    link2 = 95;
    link3 = 185;
    link4 = 110;

    % gear ratio and correction factor due to imperfection in the gear functioning
    joint_angle = (asind((link4 - offset + height*1000 - link2*sind(45) - link1 - link0)/link3) + 45) * 5 *0.8;    
end

% PI functionality for roatating the base motor
function pidc(base_rotat,base)
    % Initializing PID coefficients
    % Using only PI control as derivative controller makes the operation complex and results in jerking of robot.
    kp = 0.001;
    ki = 0.005;    
    %introduce error terms
    error = base_rotat - readRotation(base);
    ie = 0;
    start(base)
    while (abs(error) > 5)
          readRotation(base);  
          PreviousE = error;
          error = base_rotat - readRotation(base)  ;
          de = error - PreviousE;

          % To remove stack up of error
          if((de > 20) && (ie > 0))
            ie = ie + 0;
          elseif((de < -20) && (ie < 0))
            ie = ie + 0;
          else
            ie = ie + error;
          end
      
          pidc = kp*error + ki*ie;

          % For Limiting the speed to avoid overshoot
          if(pidc > 25)
            base.Speed = 25 - (pidc*0.1);
          elseif(pidc < -25)
            base.Speed = -25 - (pidc*0.1);
          else 
            base.Speed = pidc;
          end
    start(base);
    end
    base.Speed = 0;
    start(base);
end

% PI functionality for roatating the arm motor during pick up operation
function armc(angle_arm, arm)
    % Initializing PID coefficients
    % Using only PI control as derivative controller makes the operation complex and results in jerking of robot.
    kp = 0.03;
    ki = 0.005;
    % Introduce error terms
    e = angle_arm ;
    ie = 0;
    start(arm)
    while (readRotation(arm) < angle_arm)
          PreviousE = e;
          e = angle_arm + readRotation(arm);
          de = e - PreviousE;
          % To remove stack up of error
          if((de > 20) && (ie > 0))
            ie = ie + 0;
          elseif((de < -20) && (ie < 0))
            ie = ie + 0;
          else
            ie = ie + e;
          end
          pida = kp*e + ki*ie;
          % For Limiting the speed to avoid overshoot
          if(pida > 25)
            arm.Speed = 25 - (pida * 0.1);
          elseif(pida < -25)
            arm.Speed = -25 - (pida * 0.1);
          else 
            arm.Speed = pida;
          end
          start(arm);
    end
    arm.Speed = 0;
    start(arm);
end

% PI functionality for roatating the arm motor during place up operation
function armd(angle_arm, arm)
    % Initializing PID coefficients
    % Using only PI control as derivative controller makes the operation complex and results in jerking of robot.
    kp = 0.03;
    ki = 0.005;
    
    %introduce error terms
    e = angle_arm ;
    ie = 0;
    start(arm)
    while (readRotation(arm) < angle_arm)
          PreviousE = e;
          e = angle_arm + readRotation(arm);
          de = e - PreviousE;

          % To remove stack up of error
          if((de > 20) && (ie > 0))
            ie = ie + 0;
          elseif((de < -20) && (ie < 0))
            ie = ie + 0;
          else
            ie = ie + e;
          end
          pida = kp*e + ki*ie;
          % For Limiting the speed to avoid overshoot
          if(pida > 25)
            arm.Speed = +25 - (pida * 0.1);
          elseif(pida < -25)
            arm.Speed = -25 - (pida * 0.1);
          else 
            arm.Speed = pida;
          end
          start(arm);
    end
    arm.Speed = 0;
    start(arm);
end