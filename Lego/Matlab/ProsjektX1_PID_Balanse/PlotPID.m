%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
%                       SPECIFY FIGURE SIZE
fig1=figure;
screen = get(0,'Screensize');
set(fig1,'Position',[1,1,0.5*screen(3), 0.5*screen(4)])
set(0,'defaultTextInterpreter','latex');
set(0,'defaultAxesFontSize',14)
set(0,'defaultTextFontSize',16)
%----------------------------------------------------------------------


figure(fig1);
clf;

subplot(2,1,1);
hold on;
%plot(Tid(1:k),GyroAngle(1:k),"DisplayName","GAng");
plot(Tid(1:k),GyroRate(1:k),"DisplayName","GRat");
plot(Tid(1:k),GyroRateRaw(1,1:k),"DisplayName","GRat1");
plot(Tid(1:k),GyroRateRaw(2,1:k),"DisplayName","GRat2");
%plot(Tid(1:k),Angle(1:k),"DisplayName","Ang")
%plot(Tid(1:k),-Output(1:k), "magenta", "DisplayName","PID");
hold off;
subtitle('GyroAngle');

subplot(2,1,2);
hold on;
plot(Tid(1:k),P(1:k), "r","DisplayName","P");
plot(Tid(1:k),I(1:k), "g","DisplayName","I");
plot(Tid(1:k),D(1:k), "b", "DisplayName","D");
hold off;
subtitle('PID');