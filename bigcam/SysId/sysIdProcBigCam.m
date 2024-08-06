% sysIdProc.m
% script useful for ME4231 motor frequency response alalysis
% and generally any such plant.
%
% Process details:
% 1) drive a sinusoid of frequency F amplitude A into plant.
% 2) Log response to A in a vector. 
%       - First element of vector is F.
%       - Second element of vector is input amplitude.
% 3) Repeat for amplitude A for any range of frequencies F
%     saving each file with unique name and list these files as 'files1'
%
% 4) Repeat above process for a second amplitude B over a range of 
%    frequencies same as the first run (although they don;t need to be
%    same)
%    
%    List these files as 'files2' below

clear
close all

dcoffset=5;

% files 1 contains the Va input (1/2V amplitude) response data
% files1 =   {
%     's01.txt',
%     's10.txt',
%     's20.txt',
%     's30.txt',
%     's40.txt',
%     's50.txt',
%     's60.txt',
%     's70.txt',
%     's80.txt',
%     's90.txt',
%     's100.txt',
%     's110.txt',
%     's120.txt',
%     's130.txt',
%     's140.txt',
% 
% };

% files1 =   {
%     's01.txt',
%     's11.txt',
%     's21.txt',
%     's31.txt',
%     's41.txt',
%     's51.txt',
%     's61.txt',
%     's71.txt',
%     's81.txt',
%     's91.txt',
%     's101.txt',
%     's111.txt',
%     's121.txt',
%     's131.txt',
%     's141.txt'
% };


files1 =   {
     's00d.txt',
     's10d.txt',
     's20d.txt',
     's30d.txt',
     's40d.txt',
     's50d.txt',
     's60d.txt'
     };


[ampoutA, freqa, ampinA] = ... 
        SysIdAmplitudeFromFilteredRmsBigCam(files1, dcoffset, 1);

figure('name','SysID Response')
ratioA = ampoutA./ampinA;
semilogx((2*pi*freqa),(20*log10(ratioA)),'b-o');

xlim([0.01 10]);
ylim([80 120]);
legend('Va input (1V)','Vb input (mix of 2 and 3 V')

ylabel('20Log(Vout/Vin)  (dB)')
xlabel('Angular Frequency (radians)')
title('Rack Experimental Frequency Response')

grid on
