clear all
close all

% Initialize joystick
joymex2('open',0);

% Setter figurenhetene
set(groot, 'defaultFigureUnits','normalized')

figure(1);
set(1,'position',[0.1 0.1 0.7 0.5])

% Lager håndtak (handles) til hvert plot
subplot(1,1,1)
p1=plot(0,0,'b'); hold on;
p2=plot(0,0,'r');
p3=plot(0,0,'g');
p4=plot(0,0,'k');
legend('axes(1)','axes(2)','axes(3)','axes(4)','Location','Northwest')
title(['Alle aksene'])
xlabel('sekund')

k=1;
tid(k)=0;
axes_vector(1:4,k)=0;
tic
k=k+1;

joystick = joymex2('query',0);
JoyMainSwitch = joystick.buttons(1);

while ~JoyMainSwitch
    tid(k)=toc;
    
    % Leser joystick
    joystick = joymex2('query',0);
    
    axes_vector(:,k)=joystick.axes';

    % oppdaterer håndtak (går fortere enn å plotte på ny)
    set(p1,'Xdata',tid,'Ydata',axes_vector(1,:));
    set(p2,'Xdata',tid,'Ydata',axes_vector(2,:));
    set(p3,'Xdata',tid,'Ydata',axes_vector(3,:));
    set(p4,'Xdata',tid,'Ydata',axes_vector(4,:));
    
    drawnow
    k=k+1;
    joystick = joymex2('query',0);
    JoyMainSwitch = joystick.buttons(1);
end

% Clear MEX-file to release joystick
clear joymex2
