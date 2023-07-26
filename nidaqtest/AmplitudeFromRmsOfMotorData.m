function  [amplitude, freq] = AmplitudeFromRmsOfMotorData(files)
    N = length(files);
    freq = zeros(1,N);
    amplitude = zeros(1,N);
    for i=1:N
       d =  importdata(files{i});
       freq(i) = d(1);
       y = d(2:length(d))-5;

       M   = length(y);
       
       % high-pass filter...
       Fs = 400;  % Hz sampling frequency
       Fc = freq(i)/2;   % Hz cuttoff frequency
       N  = 2;    % filter order

       t=[0:1/Fs:(M-1)/Fs];

       [b,a] = butter(N,Fc/Fs,'high');
       yf = filter(b,a,y);
       figure('Name','y raw anf filtered')
       plot(t,y,'r',t,yf,'b')

        
       % accumulate sum of squares of waveform
       ssq = 0;
       for j=1:M,
           ssq = ssq + yf(j)^2;
       end
        
       % rms result
       rms = sqrt(ssq/M);

       % amplitude from RMS
       amplitude(i) = rms*sqrt(2);
   end

end

