% Rewinder simulator
%
% Description:
%   Rewinder controller (Automation Direct P1AM-100) outputs 0-10V
%   valve amplifier command. In the actual system the Wandfluh amplifier
%   maps 0-5V to to move the carriage west, 5-10V to move the carriage
%   east. Thus, this is a +/-5 volt flow command input.
%
%   The tranfer function from this command input to LVDT output is known
%   from the system identification result.
%
%   This simulator acts on the input side like the Wandfluh amplifier: it
%   accepts 0-10V input from the P1AM, maps it to +/-5 Volts.
%
%   Given this command, it then processes the command through the
%   plant model to produce an simulated LVDT output voltage.
%
%   In the actual system the LVDT is 4-20mA through 500 ohms to produce
%   a 2-10V signal to the controller, or 8 volts range on a 2-volt bias.
%   However, the control scheme numericaly differentiates the LVDT input
%   so the bias is not a significant detail.
%
%   In the actual system the Edge Guide sub-system is entirely decoupled 
%   from the carriage and the LVDT signal, but the controller accepts
%   it as position state feedback while the numerically differentiated
%   LVDT feedback is the velocity state.
%
%   This state relationship is true for a paper edge that traverses the
%   edge guide sensor as a result of the carriage lateral motion. The
%   edge guide also picks-up edge deviations on the roll.
%
%   Simulation the Edge Guide here:
%
%     The controller is seeking to regulate Edge Guide to zero.
%     In a tightly-coupled state feedback simulation, gain-map Vlvdt
%     to Veg (edge guide). Let the above-described LVDT output feedback
%     to the controller where it will be differentiated and processed
%     as the rate (tach) feedback. The simulated Edge-guide voltage will
%     output from the simulator as +-10V.
%
%     Initialization.
%
%     This simulator acts as the actual LVDT so initialize with a mid-range
%     bias voltage of say 5 V. Add the response voltage out of the model
%     described above to this bias. Again, the controller differentiates
%     it so it doesn't matter on this point, but it represents a mid-range
%     condition compared to the 2-10V LVDT signal in the actual system.
%
%     For the Edge Guide sim, this is about zero so do not apply the bias.
%     map the signed model Vlvdt output through the lvdt-to-edge guide
%     gain adjustment and feed out the second analog output.
%
%     The P1AM controller accepts this as EdgeGuide input.
%
%     When the P1AM loop is closed this is the voltage regulated to zero.
%
%
%     Testing the simulator:
%       command input drives rate, so a position step response here is 
%       a result of an impulse. The "test" input parameter below bipasses
%       the command input read from the PP1AM and instead issues an
%       N-sample impulse
clear
close all

% program control button: click to stop
ButtonHandle = uicontrol('Style', 'PushButton', ...
                         'String', 'Stop loop', ...
                         'Callback', 'delete(gcbf)');

% initialize strip chart plot
plt = plot(0,0,'ro-');
ax = gca;

% initialize national instruments device
dq = daq("ni");
addinput(dq,"Dev1", "ai0","Voltage");
addoutput(dq,"Dev1","ao0","Voltage");
addoutput(dq,"Dev1","ao1","Voltage");



% plant model coefficients
%b1 = 9.216250582849553e-04;
%b0 = 8.479686771438059e-04;
%a1 = -1.778800783071405;
%a0 = 0.778800783071405;
b1 = 9e-04;
b0 = 9e-04;
a1 = -1.8;
a0 = 0.8;





% variable initialization
tstart = tic;
tnow=0;
tlast=0;
k=1;

% impulse test mode
testmode=1

u(k)=1.0;  % will get over-written if not in test mode. Otherwise this is pulse

while 1, %tnow < 4,

    data  = read(dq);

    tm(k) = tnow;

    if testmode==0, 
        u(k)  = data.Dev1_ai0;
    elseif k>1
        u(k)=0;
    end


    % Plant model: difference equation from u to y.
    if k>2
        y(k) = b1*u(k-1) + b0*u(k-2) - a1*y(k-1) - a0*y(k-2);
    elseif k==2
        y(k) = b1*u(k-1) - a1*y(k-1);
    else  % k=1
        y(k)=0;
    end

    plt(1).XData = tm;
    plt(1).YData = u;

    ax.XLim = [max(0,tnow-1) max(tnow,1)];
    drawnow limitrate

    while tnow-tlast < 0.01
        tnow  = toc(tstart);
    end

    % output on ao0 and ao1
    write(dq,[u(k) u(k)]);

    % propagate state
    k = k+1;
    tlast=tnow;

  if ~ishandle(ButtonHandle)
    disp('Loop stopped by user');
    break;
  end    
end

for k=2:length(tm),
    td(k-1) = tm(k)-tm(k-1);
end

figure('Name','Samples vs Time')
plot(tm, u,'bo-',tm, y,'ro-');
ylabel("Voltage (V)");

figure('Name','Samples')
plot(u,'bo-')
hold
plot(y,'ro-')
ylabel("Voltage (V)");

figure('Name','Sample interval')
plot(td);

daqreset; % releases device

