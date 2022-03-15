
clear all; close all
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

myGyroSensor  = gyroSensor(mylego,2);
resetRotationAngle(myGyroSensor);
myGyroSensor  = gyroSensor(mylego,3);
resetRotationAngle(myGyroSensor);
%{
still_before = 10;
still_after = 5;
calibration_done = false;
m = -1;
n = 1;
while ~calibration_done
    rates(n) = readRotationRate(myGyroSensor);
    if n > max(still_after,still_before)
        if m > 0 && n > m + still_after
            if sum(rates(n-still_after:n)) == 0
                calibration_done = true;
            else
                m = -1;
            end
        elseif sum(rates(n-still_before:n)) == 0
            resetRotationAngle(myGyroSensor);
            m = n;
        end
    end
    n = n+1;
end
%}
disp('Calibrated!')