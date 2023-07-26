function  [amplitude, freq, inamp] = SysIdAmplitudeFromFilteredRms(files,dcoffset,plt)
    N = length(files);
    freq = zeros(N,1);
    inamp = zeros(N,1);
    amplitude = zeros(N,1);

    for i=1:N
       d =  importdata(files{i});
       freq(i) = d(1);
       inamp(i) = d(2);       
       y = d(3:length(d))-dcoffset;

       M   = length(y);
       
       % high-pass filter...
       Fs = 400;  % Hz sampling frequency
       Fc = freq(i)/2;   % Hz cuttoff frequency
       N  = 2;    % filter order

       t=[0:1/Fs:(M-1)/Fs];

       [b,a] = butter(N,Fc/Fs,'high');
       yf = filter(b,a,y);

       % ignore filter settling samples: 1 seconds worth
       ignore = Fs*10;

       yfi=yf(ignore:M);
       C=length(yfi);
       tfi=t(ignore:M);
       if(plt==1),
           figure('Name','y raw and filtered')
           plot(t,y,'r',t,yf,'b',tfi,yfi,'g')
       end
        
        
       
    
       % accumulate sum of squares of waveform
       ssq = 0;
       for j=1:C,
           ssq = ssq + yfi(j)^2;
       end
        
       % rms result
       rms = sqrt(ssq/C);

       % amplitude from RMS
       amplitude(i) = rms*sqrt(2);
   end

end

