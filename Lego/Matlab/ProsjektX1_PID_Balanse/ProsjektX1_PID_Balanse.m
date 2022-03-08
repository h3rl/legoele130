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
online = false;
% Spesifiser et beskrivende filnavn for lagring av maaledata
filename = 'P02_singlepeak.mat';
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
%     myColorSensor = colorSensor(mylego);
%     myTouchSensor = touchSensor(mylego);
%     mySonicSensor = sonicSensor(mylego);
    myGyroSensor  = gyroSensor(mylego);
    resetRotationAngle(myGyroSensor);
     
    % motorer
    motorA = motor(mylego,'A');
    motorA.resetRotation;
    motorB = motor(mylego,'B');
    motorB.resetRotation;
%     motorC = motor(mylego,'C');
%     motorC.resetRotation;
%     motorD = motor(mylego,'D');
%     motorD.resetRotation;
else
    % Dersom online=false lastes datafil.     
    load(filename)
end

disp('Equipment initialized.')
%----------------------------------------------------------------------
    

%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
%                       SPECIFY FIGURE SIZE
fig1=figure;
screen = get(0,'Screensize');
set(fig1,'Position',[1,1,0.5*screen(3), 0.5*screen(4)])
set(0,'defaultTextInterpreter','latex');
set(0,'defaultAxesFontSize',14)
set(0,'defaultTextFontSize',16)
%----------------------------------------------------------------------

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
%         Lys(k) = double(readLightIntensity(myColorSensor,'reflected'));
%         LysDirekte(k) = double(readLightIntensity(myColorSensor));
%         Bryter(k)  = double(readTouch(myTouchSensor));
%         Avstand(k) = double(readDistance(mySonicSensor));
        GyroAngle(k) = double(readRotationAngle(myGyroSensor));
        GyroRate(k)  = double(readRotationRate(myGyroSensor));
           
        % motorer
        VinkelPosMotorA(k) = double(motorA.readRotation);
        VinkelPosMotorB(k) = double(motorB.readRotation);
%         VinkelPosMotorC(k) = double(motorC.readRotation);
%         VinkelPosMotorD(k) = double(motorC.readRotation);
        
        % Data fra styrestikke. Utvid selv med andre knapper og akser
        if ismac
            skalering = 100;       % konvertering fra 1 til 100%
            JoyMainSwitch = button(joystick,1);
            JoyForover(k) = -skalering*axis(joystick,2);
        else
            skalering = 100/2^15;  % konvertering fra 2^15 til 100%
            joystick      = joymex2('query',0);
            JoyMainSwitch = joystick.buttons(1);
            JoyForover(k) = -skalering*double(joystick.axes(2)); 
        end
        
    else
        % online=false
        % Naar k er like stor som antall elementer i datavektoren Tid,
        % simuleres det at bryter paa styrestikke trykkes inn.
        if k==numel(Tid)
            JoyMainSwitch=1;
        end
        
        % simulerer EV3-Matlab kommunikasjon i online=false
        pause(0.01)

    end
    %--------------------------------------------------------------

       
    
    
    % +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    %             CONDITIONS, CALCULATIONS AND SET MOTOR POWER
    % Gjoer matematiske beregninger og motorkraftberegninger 
    % hvis motor er tilkoplet. 
    % Kaller IKKE paa en funksjon slik som i Python
    
    angvel_der = Derivation(GyroRate(k-1:k),Tid(k-1))
    angvel_hp = FIR_filter()
    
    PowerA(k) = 0; % 0.7*JoyForover(k);
    PowerB(k) = 0;
%     PowerC(k) = ...
%     PowerD(k) = ...    

    if k == 1 % Konstanter som skal settes en gang
        
        %settings
        Proportional_on_measurement = false;
        Output_max = 100;
        Output_min = 0;
        
        Kp = 90;
        Ki = 10;
        Kd = 0;
        setpoint = 100;
        inp = 15;
        lastinp = 0;
        sample_time = 0.01;
        
        error(1:2) = 0;
        
        dt = 0.1
    else
        dt = dt + Tid(k) - Tid(k-1);
    end
    
    Desired(k) = setpoint
    
    %inp = lest_verdi;
    assert(dt > 0)
    
    error = setpoint - inp;%[setpoint-inp error(1:end-1)]

    P = Kp * error(1);
    
    I = Ki * error(1) * dt;
    %I = min(Output_max,max(Output_min,I)) % basic clamp
    
    D = -Kd*(inp-lastinp)/dt;
    
    output = P+I+D;
    %output = min(Output_max,max(Output_min,output)) % basic clamp
    
    inp = inp + output*0.01 +randn*k^0.5;
    lastinp = inp;
    lastout = output;
    
    Outputs(k) = inp;
    
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
    figure(fig1);
    clf;
    
    subplot(1,1,1);
    hold on;
    plot(Tid(1:k),Outputs(1:k));
    plot(Tid(1:k),Desired(1:k));
    hold off;
    title('PID');
    xlabel('Tid [sek]');
    
    % tegn naa (viktig kommando)
    drawnow
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
%     stop(motorC);           
%     stop(motorD);           

    clear mylego
    if ismac
        clear joystick
    else
        clear joymex2
    end 
end
%------------------------------------------------------------------





