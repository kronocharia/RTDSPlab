%not as good as firfilter.m

F = [240 440 2000 2450];  %tap frequencies
A = [0 1 0];                    %tap gain, assume its working on
                                    %pairs of entries, with an implicit 0
                                    %at the start and end
pass_ripple = (10^(0.4/20)-1)/(10^(0.4/20)+1); 
stop_ripple = 10^(-50/20);
DEV = [stop_ripple pass_ripple stop_ripple];
Fs = 8000;                          %Fsamp

[m,f,a,W] = firpmord(F,A,DEV,Fs);   %tells us what order and shit it needs to be

b = firpm(m+10,f,a, W);                %coeffs of the FIR filter
[h,w] = freqz(b,1,512);         %freqz(numerator, denominator, resolution)
plot(f/2 * Fs,a,w/2/pi * Fs,abs(h)) %draws the something....

figure;                             %create new figure window
freqz(b,1,512, Fs);                 %plot magnitude and phase of filter design

tmp = (b(1));
str = ['double b[] = {' num2str(tmp)];
for i=2:length(b)
    tmp = (b(i));
    str = [str, ', ', num2str(tmp)];
end;
str = [str, '}'];
save fir_coeff.txt str -ascii -double   %save coefficient b to txt file