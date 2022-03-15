%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% Hensikten med programmet er aa ....
% Foelgende sensorer brukes:
% - Lyssensor
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
%filename = 'P02_Measurement_step.mat';
filename = 'P03_obl1.mat';
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
    

    
    
    Avstand(k) = Lys(k) + randn;
    
    %   inital values
    Fart(k) = 0;
    Akselerasjon(k) = 0;
    Akselerasjon_IIR(k)=0;
    Avstand_IIR(k) = Avstand(k);
    Fart_IIR(k) = 0;
    Akselerasjon_IIR(k) = 0;
    
    alpha = 0.3;
    
    if k ~= 1
        Ts(k-1) = Tid(k) - Tid(k-1);
        
        Avstand_IIR(k) = IIR_filter(Avstand_IIR(k-1),Avstand(k),alpha);
        
        Fart(k-1) = Derivation(Avstand(k-1:k),Ts(k-1));
        Fart_IIR(k-1) = Derivation(Avstand_IIR(k-1:k),Ts(k-1));
    end
    if k~= 1 && k~=2
            Akselerasjon(k-2) = Derivation(Fart(k-2:k-1),Ts(k-2));
            Akselerasjon_IIR(k-2) = Derivation(Fart_IIR(k-2:k-1),Ts(k-2));
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
    clf(fig1)
    subplot(3,1,1)
    hold on;
    plot(Tid(1:k),Avstand(1:k),"r","DisplayName", "Avstand");
    plot(Tid(1:k),Avstand_IIR(1:k),"b","DisplayName", "Avstand IIR");
    subtitle(sprintf("Avstandsmåling rådata (b) og IIR-filtrert avstandsmåling alfa=%.2f (r)",alpha));
    hold off;
    
    xlabel('Tid [sek]');
    ylabel('Avstand [m]');
    
    subplot(3,1,2)
    hold on;
    plot(Tid(1:k-1),Fart(1:k-1),"r","DisplayName", "Fart");
    plot(Tid(1:k-1),Fart_IIR(1:k-1),"b","DisplayName", "Fart IIR");
    subtitle("Hastighetsberegninger, ubrukelig rådata(b) og filtrert(r)");
    hold off;
    xlabel('Tid [sek]');
    ylabel('Fart [m/s]');
    
    subplot(3,1,3)
    hold on;
    %plot(Tid(1:k-1),Akselerasjon(1:k-1),"r","DisplayName", "Aks");
    plot(Tid(1:k-1),Akselerasjon_IIR(1:k-1),"b","DisplayName", "Aks IIR");
    subtitle("Akselerasjonsberegninger, ubrukelig rådata(b) og filtrert(r)");
    hold off;
    
    xlabel('Tid [sek]');
    ylabel('Aks [m/s^2]');
    
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





