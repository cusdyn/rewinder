clear
close all
load  dout.txt
load  vout.txt
load  tdout.txt

h=1/100;  % sampling interval (seconds)
N=length(dout);

t=0:h:h*(N-1);

dv = zeros(1,N);
dvf = zeros(1,N);

for k=2:N,
    dv(k) = (dout(k)-dout(k-1))/h;
end

flen=10
for k=flen:N,
    dvf(k) = sum(dv(k-flen+1:k))/flen;
end



figure('Name',"Samples")
%plot(t,dout,'b-o',t,vout,'r-+',t,dv,'g')
%plot(t,dout,'b-o',t,vout,'r-+')
plot(t,dout,'b-o');

figure('Name',"dlvdt")
plot(t,dv,'r-o',t,dvf,'b');


%figure('Name',"timediff")
%plot(t(2:N),tdout(2:N),'b-o')