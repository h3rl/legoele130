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

plotting = false;
calibrate = true;
calcdrift = true;

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
    myGyroSensor  = gyroSensor(mylego);
    if calibrate
        resetRotationAngle(myGyroSensor);
        if calcdrift
            %kalkulere drift over 100 målinger.
            for i = 1:101
               DriftMeasurement(i) = readRotationAngle(myGyroSensor);
               if mod(i,10) == 0
                    fprintf('%i%%\n',i)
               end
               pause(0.001) %in seconds
            end            
            driftoffset = mean(DriftMeasurement)
        end
    end
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
    

%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
%                       SPECIFY FIGURE SIZE
if plotting
    fig1=figure;
    screen = get(0,'Screensize');
    set(fig1,'Position',[1,1,0.5*screen(3), 0.5*screen(4)])
    set(0,'defaultTextInterpreter','latex');
    set(0,'defaultAxesFontSize',14)
    set(0,'defaultTextFontSize',16)
end
%----------------------------------------------------------------------

% setter skyteknapp til 0, og tellevariabel k=1
JoyMainSwitch=0;
JoyOffsUp = 0;
JoyOffsDown = 0;
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
        %GyroAngle(k) = readRotationAngle(myGyroSensor);
        GyroAngle(k) = readRotationRate(myGyroSensor);
        
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
            if ~JoyOffsUp
                JoyOffsUp = joystick.buttons(6);
            else
                JoyOffsUp = false;
            end
            if ~JoyOffsDown
                JoyOffsDown = joystick.buttons(4);
            else
                JoyOffsDown = false;
            end
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
     
    %settings
    outMin = -100;
    outMax = 100;

    basis = 0;
    Kp = 2.5;
    Ki = 0;
    Kd = 0.015;
    % Ønsket posisjon er 90deg

    setpoint = driftoffset;
    
    if k == 1
        I(k) = 0;
        Error(k) = 0;
        Error_f(k) = 0;
        Output(k)=0;
        setpoint = 0;
    else

        GyroAngle(k) = IIR_filter(GyroAngle(k-1),GyroAngle(k),0.8);
        %GyroRate(k) = Derivation(GyroAngle(k-1:k),Ts(k-1));
        
        Ts(k-1) = Tid(k) - Tid(k-1);
        
        assert(Ts(k-1) > 0)

        Error(k) = setpoint - GyroAngle(k);
        
        P(k) = Kp * Error(k);

        I(k) = EulerForward(0.9*I(k-1),Ki*Error(k-1),Ts(k-1));
        %I(k) = Clamp(I(k),outMin,outMax);

        alpha = 0.85;
        Error_f(k) = IIR_filter(Error_f(k-1),Error(k),alpha);

        D(k) = Kd*Derivation(Error_f(k-1:k),Ts(k-1));

        Output(k) = basis + P(k)+I(k)+D(k);
        Output(k) = Clamp(Output(k),outMin,outMax);
    end
    
    PowerA(k) = - Output(k);
    PowerB(k) = - Output(k);

    if online
        % Setter powerdata mot EV3
        % (slett de motorene du ikke bruker)
        motorA.Speed = PowerA(k);
        motorB.Speed = PowerB(k);
        
        start(motorA)
        start(motorB)
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





