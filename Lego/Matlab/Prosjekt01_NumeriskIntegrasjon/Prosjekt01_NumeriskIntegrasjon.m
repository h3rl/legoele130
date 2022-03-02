%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% Prosjekt01_NumeriskIntegrasjon.m
%
% Hensikten med programmet er aa ....
% Foelgende sensorer brukes:
% - Lyssensor
%
%
% Kommando for å lagre data til fil:
% save('P00_Measurement_1.mat','Tid','Lys','VinkelPosMotorA','JoyForover')
%--------------------------------------------------------------------------


%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
%                EXPERIMENT SETUP AND DATA FILENAME 
%
% Alltid lurt aa rydde workspace opp foerst
clear all; close all
% Skal prosjektet gjennomfoeres online mot EV3 eller mot lagrede data?
online = false;
% Spesifiser et beskrivende filnavn for lagring av maaledata
filename = 'P01_Measurement_sin.mat';
%filename = 'P01_Measurement_3_step.mat';
%--------------------------------------------------------------------------


% +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
%                      INITIALIZE EQUIPMENT
% Initialiser styrestikke, sensorer og motorer.

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
    myColorSensor = colorSensor(mylego);
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
        %pause(0.01)

    end
    %--------------------------------------------------------------

       
    
    
    % +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    %             CONDITIONS, CALCULATIONS AND SET MOTOR POWER
    % Gjoer matematiske beregninger og motorkraftberegninger 
    % hvis motor er tilkoplet. 
    % Kaller IKKE paa en funksjon slik som i Python
    
    nullflow = 50;
    
    Flow(k) = Lys(k) - nullflow;
    
    if k~=1
        T_s(k-1) = Tid(k) - Tid(k-1);
        volum_eulers(k) = EulerForward(volum_eulers(k-1),Flow(k-1),T_s(k-1));
        volum_trapes(k) = Trapes(volum_trapes(k-1),Flow(k-1:k),T_s(k-1));
    else
        volum_eulers(k) = 0;
        volum_trapes(k) = 0;
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
    figure(fig1)
    
    subplot(2,1,1)
    plot(Tid(1:k),Flow(1:k));
    xlabel('Tid [sek]')
    ylabel('[Cl/s]')
    title('Flowrate')
    
    subplot(2,1,2)
    hold on;
    %plot(Tid(1:k),volum_trapes(1:k));
    plot(Tid(1:k),volum_eulers(1:k),"r");
    hold off;
    xlabel('Tid [sek]')
    ylabel('[Cl]')
    title('Volum')
    
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

    clear mylego
    if ismac
        clear joystick
    else
        clear joymex2
    end 
end
%------------------------------------------------------------------





