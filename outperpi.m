% another view of the edge guide plant
clear
close all
Kv = 20; %10;
a=25  %10;


Kpmax=7;
wmax=2;

Td=0

if( Td > 0)
    maxlag=30;  % maximum transport lag (degrees)
    wc=(maxlag/57.3)/Td;
    wc=maxlag/(57.3*Td)
else
    wc = wmax;
end

Kp=Kpmax*10^(log10(wc/wmax))


Kl = 8/0.1;  % LVDT volts-per-meter
% EdgeGuide sensor gain
egz = 0.125*0.0254; % (m) Edge guide active zone +- about null (0.125" each way)
egv = 10;     % +/- voltage range
Keg = egv/egz % EdgeGuide volts per meter

Klg = Kl/Keg;


smax = 0.1/4;  % meters per second max slew rate
umax = 5;  % volts max output to valve amp

dpm = 85*pi/180; % desired phase bump
alpha = (1/sin(dpm) - 1)/(1 + 1/sin(dpm))

Kt = 1;  % tach feedback gain

b = alpha*(a+Kt*Kv);


nump=Kv;
denp=conv([1 0],[1 a]);
figure('Name', 'Plant Model')
bode(nump,denp);

num = Kp*Kv*[1 b];
den = conv([1 0 0],[1 (a+Kt*Kv)]);
W=[0.01:.001:1000];
G = tf(num,den,'InputDelay',Td)
figure('Name', 'Open Loop Transfer Function')
[M,P]=bode(G,W);
M=squeeze(M);
P=squeeze(P);


subplot(2,1,1)
semilogx(W,20*log10(M));
grid on
xlim([0.001 1000]);
subplot(2,1,2)
fig = semilogx(W,P,W,-57.3*W*.3-180,W,-57.3*W*1-180,W,-57.3*W*2-180,W,-57.3*W*3-180,W,-57.3*W*4-180,W,-57.3*W*4-180);
grid on
xlim([0.001 1000]);
ylim([-270 -90])
yticks([-270:10:-90]);

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