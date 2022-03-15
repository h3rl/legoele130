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
filename = 'yadana.mat';
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
        Lys(k) = double(readLightIntensity(myColorSensor,'reflected'));
           
        % motorer
        VinkelPosMotorA(k) = double(motorA.readRotation);
        VinkelPosMotorB(k) = double(motorB.readRotation);
        
        % Data fra styrestikke. Utvid selv med andre knapper og akser
        if ismac
            skalering = 100;       % konvertering fra 1 til 100%
            JoyMainSwitch = button(joystick,1);
            JoyForover(k) = -skalering*axis(joystick,2);
            JoySideways(k) = -skalering*axis(joystick,1);
        else
            skalering = 100/2^15;  % konvertering fra 2^15 til 100%
            joystick      = joymex2('query',0);
            JoyMainSwitch = joystick.buttons(1);
            JoyForover(k) = -skalering*double(joystick.axes(2));
            JoySideways(k) = -skalering*double(joystick.axes(1));
        end
        
    else
        % online=false
        % Naar k er like stor som antall elementer i datavektoren Tid,
        % simuleres det at bryter paa styrestikke trykkes inn.
        if k==numel(Tid)
            JoyMainSwitch=1;
        end
        
        % simulerer EV3-Matlab kommunikasjon i online=false
%         pause(0.01)

    end
    %--------------------------------------------------------------

       
    
    
    % +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    %             CONDITIONS, CALCULATIONS AND SET MOTOR POWER
    % Gjoer matematiske beregninger og motorkraftberegninger 
    % hvis motor er tilkoplet. 
    % Kaller IKKE paa en funksjon slik som i Python
    
    
    a=0.12;
    b=0.1;
    PowerA(k) = a*JoyForover(k) - b*JoySideways(k);
    PowerB(k) = a*JoyForover(k) + b*JoySideways(k);
    
    Referanse = Lys(1);
    e(k) = Referanse - Lys(k);
    
    
    MAE(k) = sum(abs(e))/(k);
    
    if k~=1
        Ts(k-1) = Tid(k) - Tid(k-1);

        IAE(k)= EulerForward(IAE(k-1),abs(e(k)),Ts(k-1));

        TV_A(k) = abs(PowerA(k)-PowerA(k-1)) + TV_A(k-1);
        TV_B(k) = abs(PowerB(k)-PowerB(k-1)) + TV_B(k-1);
    else
       IAE(k) = 0;
       TV_A(k)=0;
       TV_B(k)=0;
    end

    Middelverdi = ones(1,numel(Lys)) * mean(Lys);
    Standardavvik = ones(1,numel(Lys)) * std(Lys);
    Referanse_ = ones(1,numel(Lys)) * Referanse;
    
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
    
    plotting = true;
    
    if plotting
    
        % aktiver fig1
        figure(fig1)
        clf(fig1)
        
        subplot(3,2,1)
        hold on;
        plot(Tid(1:k),Lys(1:k),"b","DisplayName", "Lys");
        plot(Tid(1:k),Referanse_(1:k),"r","DisplayName", "Referanse");
        hold off;
        subtitle('Lys')
        xlabel('Tid [sek]')

        subplot(3,2,2) 
        plot(Tid(1:k),e(1:k),"b","DisplayName", "Avvik e(k)");
        subtitle('Avvik e(k)')
        xlabel('Tid [sek]')

        subplot(3,2,3)
        hold on;
        plot(Tid(1:k),PowerA(1:k),"b","DisplayName", "PowerA");
        plot(Tid(1:k),PowerB(1:k),"r","DisplayName", "PowerB");
        hold off;
        subtitle('PowerA (venstre) og PowerB (høyre)')
        xlabel('Tid [sek]')

        subplot(3,2,4)
        plot(Tid(1:k),IAE(1:k), "b","DisplayName", "IAE");
        subtitle('IAE, integral abs(e(k))')
        xlabel('Tid [sek]')

        subplot(3,2,5)
        hold on;
        plot(Tid(1:k),TV_A(1:k), "b","DisplayName", "TV_A venstre");
        plot(Tid(1:k),TV_B(1:k), "r","DisplayName", "TV_B høyre");
        hold off;
        subtitle('TV_A(k) og TV_B(k), Total variation')
        xlabel('Tid [sek]')

        subplot(3,2,6)
        plot(Tid(1:k),MAE(1:k));
        subtitle('MAE, sum av e(k)')
        xlabel('Tid [sek]')
        
        % tegn naa (viktig kommando)
        drawnow
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





