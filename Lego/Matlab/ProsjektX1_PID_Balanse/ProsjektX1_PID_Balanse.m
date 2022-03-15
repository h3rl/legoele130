%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% Prosjekt0X_.....
%
% Hensikten med programmet er aa ....
% Foelgende sensorer brukes:
% - Lyssensor
% - ...
% - ...
%
% Foelgende motorer brukes: 
% - motor A
% - ...
% - ...
%
%--------------------------------------------------------------------------


%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
%                EXPERIMENT SETUP AND DATA FILENAME 
%
% Alltid lurt aa rydde workspace opp foerst
clear all; close all
% Skal prosjektet gjennomfoeres online mot EV3 eller mot lagrede data?
online = true;
% Spesifiser et beskrivende filnavn for lagring av maaledata
filename = '.mat';

%--------------------------------------------------------------------------

% +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
%                      INITIALIZE EQUIPMENT
% Initialiser styrestikke, sensorer og motorer.
%
% Spesifiser hvilke sensorer og motorer som brukes.
% I Matlab trenger du generelt ikke spesifisere porten de er tilkoplet.
% Unntaket fra dette er dersom bruke 2 like sensorer, og Da maa 
% du initialisere 2 sensorer med portnummer som argument.
% Eksempel:
% mySonicSensor_1 = sonicSensor(mylego,3);
% mySonicSensor_2 = sonicSensor(mylego,4);

% For ryddig og oversiktlig kode, er det lurt aa slette
% de sensorene og motoren som ikke brukes.

if online  
    if ismac
        % Mac-bruker:
        % - Laster foerst Java-bibliotek til styrestikke
        % - Sletter deretter gamle instanser
        % - Lager nye instanser av EV3 og styrestikke
        HebiJoystick.loadLibs();
        clear mylego joystick
        mylego = legoev3('USB');
        joystick = HebiJoystick(1);
    else
        % PC-bruker:
        % - Sletter foerst gamle instanser
        % - Lager nye instanser av EV3 og styrestikke
        clear mylego joymex2
        mylego = legoev3('USB');
        joymex2('open',0);
    end
    
    %% init sensorer og motorer
    myGyroSensor  = gyroSensor(mylego);
    resetRotationAngle(myGyroSensor);

    % motorer
    motorA = motor(mylego,'A');
    motorA.resetRotation;
    motorB = motor(mylego,'B');
    motorB.resetRotation;

    %% init values

    sampletime = 22; % ms
    wheeldiameter = 42; % mm
    gyrotype = 0;
    kp = 0.6;
    ki = 14;
    kd = 0.005;
    gain_angular_velocity = 1.3;
    gain_angle = 25;
    gain_motor_speed = 75;
    gain_motor_pos = 350;

    dt = (sampletime-2)/1000;
    radius = wheeldiameter/2000;
    max_index = 7;
    enc_val = zeros(max_index);

    motor_refpos = 0;
    NowOutOfBound = false;
    PrevOutOfBound = false;
    OutOfBoundCount = 0;
    OutOfBound = 0;
    angle = 0;
    mean_angle = 0;

    speed = 0;
    steering = 0;
    max_acceleration = 0;

    onerad = 180/pi;

    %% Kalibrere gyro
    playTone(myev3,440,0.1,10);
    pause(0.1);
    gyro_mean = 0;
    for i = 1:100
        gyro_mean = gyro_mean + readRotationRate(myGyroSensor);
        pause(0.005);
    end
    gyro_mean = gyro_mean/100;
    pause(0.1);
    playTone(myev3,440,0.1,10);
    pause(0.1);
    playTone(myev3,440,0.1,10);
end

disp('Equipment initialized.')

% setter skyteknapp til 0, og tellevariabel k=1
JoyMainSwitch=0;
k=1;

while ~JoyMainSwitch
    %+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    %                       GET TIME AND MEASUREMENT
    % Faa tid og maalinger fra sensorer, motorer og joystick
    %
    % For ryddig og oversiktlig kode, er det lurt aa slette
    % de sensorene og motoren som ikke brukes.

    if online
        if k==1
            tic
            Tid(1) = 0;
        else
            Tid(k) = toc;
        end

        % sensorer
        GyroAngle(k) = readRotationAngle(myGyroSensor);
        GyroRate(k) = readRotationRate(myGyroSensor);
        
        % motorer

        VinkelPosMotorA(k) = double(motorA.readRotation);
        VinkelPosMotorB(k) = double(motorB.readRotation);
        
        % Data fra styrestikke. Utvid selv med andre knapper og akser
        if ismac
            %skalering = 100;       % konvertering fra 1 til 100%
            JoyMainSwitch = button(joystick,1);
            %JoyForover(k) = -skalering*axis(joystick,2);
        else
            %skalering = 100/2^15;  % konvertering fra 2^15 til 100%
            joystick      = joymex2('query',0);
            JoyMainSwitch = joystick.buttons(1);
            %JoyForover(k) = -skalering*double(joystick.axes(2)); 
        end
        
    else
        % online=false
        % Naar k er like stor som antall elementer i datavektoren Tid,
        % simuleres det at bryter paa styrestikke trykkes inn.
        if k==numel(Tid)
            JoyMainSwitch=1;
        end
        
        % simulerer EV3-Matlab kommunikasjon i online=false
        % pause(0.01)

    end
    %--------------------------------------------------------------
    
    
    % +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    %             CONDITIONS, CALCULATIONS AND SET MOTOR POWER
    % Gjoer matematiske beregninger og motorkraftberegninger 
    % hvis motor er tilkoplet. 
    % Kaller IKKE paa en funksjon slik som i Python
     

    motor_refpos = motor_refpos + (Ts(k-1)*speed*0.002);%spesial
    
    enc_index = enc_index + 1;
    if enc_index == max_index
        enc_index = 0;
    end
    compare_index = enc_index + 1;
    if compare_index == max_index
        compare_index = 0;
    end
    avg_VinkelPos = (VinkelPosMotorA(k)+VinkelPosMotorB(k))/2;
    enc_val(enc_index) = avg_VinkelPos

    motorspeed = (enc_val(enc_index)-enc_val(compare_index))/(Ts(k-1)*max_index)


    robotspeed = radius*motorspeed/onerad;%spesial
    robotposition = radius*avg_VinkelPos/onerad;%spesial



    % read gyrorate
    gyrorate = 0;
    for j=1:5
        gyrorate = gyrorate + readRotationRate(myGyroSensor);
    end
    gyrorate = gyrorate/5;

    % calc angles and angvel(readgyro)
    mean_angle = (1-Ts(k-1)*0.2)*mean_angle + Ts(k-1)*0.2*gyrorate;
    
    angular_velocity = gyrorate - mean_angle; %spesial
    angle = angle + Ts(k-1)*angular_velocity; %spesial


    % calc weighted sum
    Error(k) = gain_angle * angle + ...
                gain_angular_velocity*angular_velocity + ...
                gain_motor_speed * robotspeed + ...
                gain_motor_pos*(robotposition-motor_refpos);

    if k == 1
        Error(k) = 0;
        acc_error(k) = 0;
    else
        Ts(k-1) = Tid(k) - Tid(k-1);
        
        P(k) = Kp * Error(k);

        I(k) = EulerForward(I(k-1),Ki*Error(k-1),Ts(k-1));
        %I(k) = Clamp(I(k),outMin,outMax);

        %alpha = 0.85;
        %Error_f(k) = IIR_filter(Error_f(k-1),Error(k),alpha);
        D(k) = Kd*Derivation(Error(k-1:k),Ts(k-1));

        Output(k) = basis + P(k)+I(k)+D(k);
        Output(k) = Clamp(Output(k),outMin,outMax);
    end

    if abs(Output(k)) > 100
        NowOutOfBound = true;
    end
    if NowOutOfBound && PrevOutOfBound
        OutOfBoundCount = OutOfBoundCount + 1;
    else
        OutOfBoundCount = 0;
    end

    if OutOfBoundCount > 20
        pause(0.1);
        stop(motorA);          
        stop(motorB);
%         playTone(myev3,800,0.1,10);
%         playTone(myev3,600,0.1,10);
%         playTone(myev3,300,0.1,10);
        JoyMainSwitch = true;
        continue;
    else
         OutOfBound = NowOutOfBound;
    end
    

    
    PowerA(k) = - Output(k)*0.021/radius;
    PowerB(k) = - Output(k)*0.021/radius;

    if ~calcdrift || GyroAngle(k) > balanceoffset - 45 && GyroAngle(k) < balanceoffset + 45
    if online
        % Setter powerdata mot EV3
        % (slett de motorene du ikke bruker)
        motorA.Speed = PowerA(k);
        motorB.Speed = PowerB(k);
        
        start(motorA)
        start(motorB)
    end
    end
    %--------------------------------------------------------------
   
    
    
    
    %++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    %                  PLOT DATA
    % Denne plasseres enten i while-lokka eller rett etterpaa. 
    % Dette kan enkelt gjoeres ved flytte de 5 nederste linjene 
    % før 'end'-kommandoen nedenfor opp før denne seksjonen. 
    % Alternativt så kan du lage en egen .m-fil for plottingen som du
    % kaller på. 
    %
    % Husk at syntaksen plot(Tid(1:k),data(1:k))
    % gir samme opplevelse i online=0 og online=1 siden
    % hele datasettet (1:end) eksisterer i den lagrede .mat fila
    
    % aktiver fig1
    
    if plotting
        PlotPID;
    end
    %--------------------------------------------------------------

    % For aa flytte PLOT DATA etter while-lokken, er det enklest aa
    % flytte de neste 5 linjene (til og med "end") over PLOT DATA.
    %
    % Oppdaterer tellevariabel
    k=k+1;
end
 
% +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
%           CLOSE JOYSTICK AND EV3
% Lukker forbindelsen til baade styrestikke og EV3.
% Det er ikke flere maater aa stoppe motorene paa

if online
    % For ryddig og oversiktlig kode, er det lurt aa slette
    % de sensorene og motoren som ikke brukes.
    stop(motorA);           
    stop(motorB);          

    clear mylego
    if ismac
        clear joystick
    else
        clear joymex2
    end
    k = k-1;
end

PlotPID;
%------------------------------------------------------------------





