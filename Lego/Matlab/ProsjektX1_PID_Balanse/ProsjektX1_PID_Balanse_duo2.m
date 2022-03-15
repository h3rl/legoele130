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
    
    % sensorer
    Gyro(1)  = gyroSensor(mylego,2);
    Gyro(2)  = gyroSensor(mylego,3);
    if docalc
        resetRotationAngle(Gyro(1));
        resetRotationAngle(Gyro(2));
    end
    %kalkulere drift over 100 målinger.
%     for i = 1:101
%        DriftMeasurement(i) = readRotationRate(Gyro(1));
%        if mod(i,10) == 0
%             fprintf('%i%%\n',i)
%        end
%        pause(0.004) %in seconds
%     end
%     offs_driftrate = mean(DriftMeasurement);
    
    % motorer
    motorA = motor(mylego,'A');
    motorA.resetRotation;
    motorB = motor(mylego,'B');
    motorB.resetRotation;
else
    % Dersom online=false lastes datafil.     
    load(filename)
end

disp('Equipment initialized.')
%pause(2);
%----------------------------------------------------------------------
    

%----------------------------------------------------------------------

% setter skyteknapp til 0, og tellevariabel k=1
JoyMainSwitch=0;
k=1;
alloc = 300;
Tid = zeros(alloc);
Ts = zeros(alloc);
GyroRateRaw = [zeros(alloc);zeros(alloc)];
GyroRate = zeros(alloc);
P = zeros(alloc);
I = zeros(alloc);
D = zeros(alloc);
Error = zeros(alloc);
Error_f = zeros(alloc);
Output = zeros(alloc);


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
        %GyroAngle(k) = readRotationAngle(myGyroSensor);
        %GyroRate(k) = readRotationRate(Gyro(1));
        GyroRateRaw(1,k) = readRotationRate(Gyro(1));
        GyroRateRaw(2,k) = -readRotationRate(Gyro(2));
        GyroRate(k) = (GyroRateRaw(1,k)+GyroRateRaw(2,k))/2;
        %GyroAngle(k) = readRotationAngle(Gyro(2));

        % motorer
%         VinkelPosMotorA(k) = double(motorA.readRotation);
%         VinkelPosMotorB(k) = double(motorB.readRotation);
        
        % Data fra styrestikke. Utvid selv med andre knapper og akser
        if ismac
            %skalering = 100;       % konvertering fra 1 til 100%
            JoyMainSwitch = button(joystick,1);
            %JoyForover(k) = -skalering*axis(joystick,2);
        else
            %skalering = 100/2^15;  % konvertering fra 2^15 til 100%
            joystick      = joymex2('query',0);
            JoyMainSwitch = joystick.buttons(1);
        end
    end
    %--------------------------------------------------------------
    
    
    % +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    %             CONDITIONS, CALCULATIONS AND SET MOTOR POWER
    % Gjoer matematiske beregninger og motorkraftberegninger 
    % hvis motor er tilkoplet. 
    % Kaller IKKE paa en funksjon slik som i Python
     
    %settings
    
    if k == 1
        Kp = 1.7;
        Ki = 2;
        Kd = 0.025;

        I(k) = 0;
        Error(k) = 0;
        Error_f(k) = 0;
        Output(k)=0;
        setpoint = 0;
    else
      

        %GyroAngle(k) = IIR_filter(GyroAngle(k-1),GyroAngle(k),0.95);
        GyroRate(k) = IIR_filter(GyroRate(k-1),GyroRate(k),0.85);
        
        Ts(k-1) = Tid(k) - Tid(k-1);
% 
%         Tc = 0.5;
%         a = Tc/(Tc+Ts(k-1));
% 
%         %                  (GyroAngle(k) + GyroRate(k)*Ts(k-1)) 
%         %Angle(k) = (a)*EulerForward(Angle(k-1),GyroAngle(k),Ts(k-1)) + (1-a)*(GyroRate(k))

        Error(k) = setpoint - GyroRate(k);
        
        P(k) = Kp * Error(k);

        I(k) = EulerForward(0.9*I(k-1),Ki*Error(k-1),Ts(k-1));
        %I(k) = Clamp(I(k),outMin,outMax);

        %alpha = 0.85;
        %Error_f(k) = IIR_filter(Error_f(k-1),Error(k),alpha);

        D(k) = Kd*Derivation(Error(k-1:k),Ts(k-1));

        Output(k) = P(k)+I(k)+D(k);
        Output(k) = Clamp(Output(k),-100,100);
    end

    % Setter powerdata mot EV3
    % (slett de motorene du ikke bruker)
    motorA.Speed = -Output(k);
    motorB.Speed = -Output(k);
    
    start(motorA)
    start(motorB)
    %-------------------------------------------------------------


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





