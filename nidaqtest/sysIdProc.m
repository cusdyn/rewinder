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
files1 =   {
    'sid010_100.txt',
    'sid0125_100.txt',
    'sid025_100.txt', 
    'sid050_100.txt',
    'sid100_100.txt',
    'sid1125_100.txt',    
    'sid125_100.txt',
    'sid150_100.txt',
    'sid175_100.txt'
};
% files 1 contains the Vb input (1V amplitude) response data
% files2 =   {
%     'sid100_200.txt',
%     'sid100_300.txt',
%     'sid100_400.txt',
%     'sid100_150.txt',
%     'sid100_125.txt',
%     'sid100_075.txt',
%     'sid100_050.txt'
% };

files2 =   {
    'sid020_300.txt'
    'sid025_200.txt',
    'sid040_200.txt',
    'sid050_200.txt',
    'sid075_200.txt',
    'sid0875_200.txt',
    'sid100_200.txt',
    'sid125_200.txt',
    'sid150_200.txt',
    'sid175_200.txt',
    'sid200_200.txt',
    'sid2125_200.txt',
    'sid225_200.txt',
    'sid300_200.txt',
    'sid350_200.txt',
    'sid400_200.txt',
    'sid410_200.txt',     
    'sid450_200.txt',    
    'sid500_200.txt',
    'sid600_200.txt',
     'sid800_200.txt'
};

[ampoutA, freqa, ampinA] = ... 
        SysIdAmplitudeFromFilteredRms(files1, dcoffset, 0);
[ampoutB, freqb, ampinB] = ...
        SysIdAmplitudeFromFilteredRms(files2, dcoffset, 0);

figure('name','SysID Response')
ratioA = ampoutA./ampinA;
ratioB = ampoutB./ampinB;
semilogx((2*pi*freqa),(20*log10(ratioA)),'b-o', ... 
         (2*pi*freqb),(20*log10(ratioB)),'r-o');
xlim([0.01 100]);
ylim([-45 30]);
legend('Va input (1V)','Vb input (mix of 2 and 3 V')

ylabel('20Log(Vout/Vin)  (dB)')
xlabel('Angular Frequency (radians)')
title('Rack Experimental Frequency Response')

grid on
