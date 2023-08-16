% another view of the edge guide plant
clear
close all
Kv = 20; % Experimentally determined plant gain
a=25;    % Experimentally determined plant bandwidth (rad/sec)

% start with unity gain and maximum crossover frequency
% We'll compute gain at this crossover frequency
wmax=10;

Kl = 8/0.1;  % LVDT volts-per-meter

% EdgeGuide sensor gain
egz = 0.125*0.0254; % (m) Edge guide active zone +- about null (0.125" each way)
egv = 10;     % +/- voltage range
Keg = egv/egz % EdgeGuide volts per meter

% mapping LVDT gain to EdgeGuide gain
Klg = Kl/Keg;


smax = 0.1/4;  % meters per second max slew rate
umax = 5;  % volts max output to valve amp

% Kill the integrator with a zero sufficiently low in frequency
% relative to the second plant pole such that a phase hump is sufficient
% to offset the precipitous phase drop-off due to transport lag.

% this zero is going to be slow for little or no transport lag, but
% this is not realistic. Transport lag is tenths of a second for
% typical operating speeds. in the 1-to-10 radian per-second crossover
% region this is 30 and more degrees of phase lost du to transport lag.
% Therefore we design with a slow zero knowing that transprt lag will
% be effectively pulling-in the plant pole.

% Ignoring transport lag consider you're designing for a phase margin of 
% 85 degrees.
dpm = 85*pi/180; % desired phase bump
% resultant alpha
alpha = (1/sin(dpm) - 1)/(1 + 1/sin(dpm))
Kt = 1;  % tach feedback gain
% compensator zero
b = alpha*(a+Kt*Kv);

% Unity feedback gain
num = Kv*[1 b];
den = conv([1 0 0],[1 (a+Kt*Kv)]);
G = tf(num,den);
W=[0.01:.001:1000];
[M,PP]=bode(G,W);
M=squeeze(M);
PP=squeeze(PP);

% Knowing where we wan max crossover, establish max gain.
Kpmax = 1/M(find(W==wmax));

% For a given transport lag time we accept the delay will take
% "maxlag" of phase margin. We determine a new crossover frequency
% where the Time delay results in this degree of phase loss.
Td=3;

if( Td > 0)
    maxlag=30;  % maximum transport lag (degrees)
    wc=(maxlag/57.3)/Td;
    wc=maxlag/(57.3*Td)
else
    wc = wmax;
end

% Given we kill the integrating pole at a low relative frequency, 'b' and
% our wmax is lower in frequency than the second plant pole, we are
% scaling our gain for unity-gain crossover in the -20dB/decade
% region of our magnitude plot. We start and -40 and past the plant pole
% we're -40 plus unmodelled poles.

% The lag-time specified 'wc' is a fraction of design wmax, for which we
% have set Kpmax. Therefore we scale-down Kp in this -20dB per decade
% slope region.

Kp=Kpmax*10^(log10(wc/wmax))

% With Kp established we can now model the system, including the delay.
num = Kp*Kv*[1 b];
G = tf(num,den,'InputDelay',Td)
[M,P]=bode(G,W);
M=squeeze(M);
P=squeeze(P);


fig = figure('Name','Open-Loop Frequency Response with Transport Lag Illustrated');
set(fig, 'Position', [10 10 1200 700])
subplot(2,1,1)

semilogx(W,20*log10(M));

grid on
xlim([0.001 1000]);
str = sprintf('Open-Loop Frequency Response, Compensated. Td=%2.1f seconds', Td);
title(str)
ylabel('Gain (dB)');
subplot(2,1,2)
td(1)=0.01;
td(2)=0.1;
td(3)=0.5;
td(4)=1;
td(5)=2.0;
td(6)=3.0;

title('Tranport Delay Phase margin loss included in blue. Example Phase loss for Td range for reference')
fig = semilogx(W,PP,'r:',W,P,W,-57.3*W*td(1),W,-57.3*W*td(2),W,-57.3*W*td(3),W,-57.3*W*td(4),W,-57.3*W*td(5),W,-57.3*W*td(6));
grid on
xlim([0.001 1000]);
ylim([-180 0])
yticks([-180:10:0]);
legend('No delay phase model','Td Modelled','Transport lag for Tdelay=0.01s','0.1s','0.5s','1s','2s','3s','Location','northwest')
ylabel('relative phase (degrees)');
xlabel('Angular Frequency (rad/sec)');
str = sprintf('Phase response: Notice how the Td range example reduce the red-dotted phase bump. "Td modelled" result for specified Td=%2.1f',Td);
title(str)

H=feedback(G,1);

figure('Name', 'Closed Loop Transfer Function')
bode(H);

if Td==0,
    figure('Name', 'Root Locus')
    rlocus(G);
end

maxphiloc=find(P==max(P))

cog=M(maxphiloc)

Kpnext=Kp/cog

tstop = 40
set_param('simrewinder_scaled','AlgebraicLoopSolver','Auto')
sim('simrewinder_scaled', tstop);

% get simulink output
s_lvdt  = get(logsout,'lvdt');
s_vd    = get(logsout,'vdot');
s_u     = get(logsout,'u');

tm    = s_lvdt.Values.Time;
lvdt  = s_lvdt.Values.Data;
vdot  = s_vd.Values.Data;
u     = s_u.Values.Data;

fig = figure('Name','X and Edge Errors');
set(fig, 'Position', [10 10 1200 700])

subplot(3,3,1)
plot(tm,lvdt,'r');
title('LVDTv');
ylabel('V')

subplot(3,3,2)
plot(tm,vdot,'b');
title('Vdot');
ylabel('V/s')

subplot(3,3,3)
plot(tm,u,'r');
title('Valve cmd');
ylabel('V')