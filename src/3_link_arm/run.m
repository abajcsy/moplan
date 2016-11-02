%% This is a simple 3-link arm model created by designing an arm in
%% MapleSim5, exporting to C, and then importing into Matlab. 
%% Created by Travis DeWolf, 2011

clear; 
close all;

%% Constants
dt = .01; % timestep
du = .01; % muscle activation change

boneWidth = 4; 
muscleWidth = 2;

% arm constants
L1 = 3.1; 
L21 = 2.25; % from elbow to wrist
L22 = .45; % elbow backwards
L31 = 1; % wrist to finger
L32 = .5; % wrist backwards

u1 = 0; 
u2 = 0; 
u3 = 0; 
u = [u2 u1 u3]; % initial torques [elbow shoulder wrist]

% arm link lengths
Constants.L = [L1, L21+L22, L31+L32];
% mass of links
Constants.m = [1.93, 1,32, .35];
% joint mount location offset
Constants.p0 = [0, 0.45, 0.5];
% inertia of arm segment
Constants.Izz = [0.0141 0.012, 0.001];

%% Set up initial state of the model 

% [ShoulderAngle ShoulderVel ElbowAngle  ElbowVel WristAngle  WristVel]
armState = [pi/4 0 pi/4 0 pi/4 0];

% Load in the simulink model
mdl = load_system('MuscleArm_Subsystem');
set_param(mdl,'SaveFinalState','on', 'FinalStateName', ['armState'], 'LoadInitialState', 'on', ...
    'InitialState',['armState']);
[t,x,y] = sim('MuscleArm_Subsystem',[0 dt],[],[0 u]);

R = x(1:2:5);

ch = '';
keydown=0;

% Set up figure and character input
figure(1); clf; hold on; grid;
set(gca, 'NextPlot', 'replacechildren');
set(gcf,'doublebuffer','on');
set(gcf,'KeyPressFcn','keydown=1;');

t = 0; % set start time

while 1==1 
    t = t+dt;
    
    u = [u2 u1 u3]; % joint torques [elbow shoulder wrist]
    
    set_param(mdl, 'InitialState',['armState']);
    [t1,x,y] = sim('MuscleArm_Subsystem',[t t+dt],[],[t u]);
    
    dR = x(1,2:2:6);
    R = x(1,1:2:5);
    eMR = y(1,4:6);
    
    %% Keyboard Control
    ch = get(gcf,'CurrentCharacter');
    if keydown==1
      switch(ch)
         
        case '1' % shoulder angle velocity increase
            u1=u1+du;
        case '2' % shoulder angle velocity increase
            u1=u1-du;
        case 'q' % set to 0! 
            u1 = 0; 
        
        case '3' % elbow angle velocity increase
            u2=u2+du;
        case '4' % elbow angle velocity increase
            u2=u2-du;
        case 'w' % set to 0! 
            u2 = 0; 
            
        case '5' % wrist angle velocity increase
            u3=u3+du;
        case '6' % wrist angle velocity increase
            u3=u3-du;
        case 'e' % set to 0! 
            u3 = 0; 
           
      end
      keydown=0;
    end
    
       
    %% Plot the arm and activations
    
    % y output is in form [elbow shoulder wrist] angles
    elbow = y(1,1);
    shoulder = y(1,2);
    wrist = y(1,3);
    S1 = sin(shoulder); 
    C1 = cos(shoulder);
    C12 = cos(shoulder+elbow);
    S12 = sin(shoulder+elbow);
    C123 = cos(shoulder+elbow+wrist);
    S123 = sin(shoulder+elbow+wrist);
    
    x1 = L1*C1; 
    y1 = L1*S1; %xy of elbow
 
    x21 = x1+L21*C12; 
    y21 = y1 + L21*S12; %xy of wrist
    x22 = x1-L22*C12; 
    y22 = y1 - L22*S12;
    
    x31 = x21 + L31*C123;
    y31 = y21 + L31*S123; %xy of finger
    x32 = x21 - L32*C123; 
    y32 = y21 - L32*S123;
    
    clf; hold on; axis([-8 10 -2 8]); grid;
    plot([0 x1],[0 y1],'LineWidth',boneWidth);  %plot shoulder to elbow
    plot([x1 x21],[y1,y21],'LineWidth',boneWidth); %plot elbow to wrist    
    plot([x1 x22],[y1,y22],'LineWidth',boneWidth); 
    plot([x21 x31],[y21 y31],'LineWidth',boneWidth); % plot wrist to finger
    plot([x21 x32],[y21,y32],'LineWidth',boneWidth); 
%         plot(x31,y31,'ks','LineWidth',boneWidth);  %plot finger
    plot(0,0,'ko','LineWidth',8); % plot shoulder ball
    plot(x1,y1,'ko','LineWidth',5); % plot elbow ball
    plot(x21,y21,'ko','LineWidth',5); % plot wrist ball
        
    %% print the torques
    text(8,2.5,sprintf('Elbow=%3.2f',y(1,1)*180/pi));
    text(8,2,sprintf('Shoulder=%3.2f',y(1,2)*180/pi));
    text(8,3,sprintf('Wrist=%3.2f',y(1,3)*180/pi));
    text(8,4,sprintf('ShoulderVel=%3.5f',x(1,2)));
    text(8,4.5,sprintf('ElbowVel=%3.5f',x(1,4)));
    text(8,5,sprintf('WristVel=%2.5f',x(1,6)));
    text(8,7,sprintf('ShoulderT=%2.4f',u1));
    text(8,6.5,sprintf('ElbowT=%2.4f',u2));
    text(8,6,sprintf('WristT=%2.4f',u3));
    
    drawnow();
end


