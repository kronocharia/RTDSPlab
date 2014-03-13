%{
F = [245 440 2000 2200];  %tap frequencies
A = [0 1 0];                    %tap gain, assume its working on
                                    %pairs of entries, with an implicit 0
                                    %at the start and end
pass_ripple = (10^(0.4/20)-1)/(10^(0.4/20)+1); 
stop_ripple = 10^(-50/20);
DEV = [stop_ripple pass_ripple stop_ripple];
Fs = 8000;                          %Fsamp

[m,f,a,W] = firpmord(F,A,DEV,Fs);   %tells us what order and shit it needs to be

b = firpm(m,f,a, W);                %coeffs of the FIR filter
[h,w] = freqz(b,1,512);         %freqz(numerator, denominator, resolution)
plot(f/2 * Fs,a,w/2/pi * Fs,abs(h)) %draws the something....

figure;                             %create new figure window
freqz(b,1,512, Fs);                 %plot magnitude and phase of filter design

%}
order = 4;
fs = 8000                   %sampling frequency
%pass_ripple = (10^(0.5/20)-1)/(10^(0.5/20)+1); 
%stop_gain = 10^(-25/20);

pass_ripple = 0.4;      %in dB
stop_attenuation = 25;
edge_freq = [2*280/fs 2*470/fs]  %filter normalised edge frequencies

%[b,a] = ellip(order/2,pass_ripple,stop_attenuation,edge_freq); %original
%num coeffs =5
[b,a] = ellip(3,pass_ripple,stop_attenuation,edge_freq);
%[b,a] = ellip(n,Rp,Rs,Wp,'ftype')
%----order n 'ftype' digital elliptic filter with normalized passband edge frequency Wp, 
%----Rp dB of ripple in the passband, and a stopband Rs dB down from the peak value in the passband.
%---- returns b,a
Hd=dfilt.df2(b,a);
fvtool(Hd)
%plot stuff
figure;                             %create new figure window
freqz(b,a,512, fs);                 %plot magnitude and phase of filter design

%turns coeffs to string
tmp = (b(1));
str_b = ['double b[] = {' num2str(tmp)];
for i=2:length(b)
    tmp = (b(i));
    str_b = [str_b, ', ', num2str(tmp)];
end;
str_b = [str_b, '};'];
str_len_b = ['#define N ', num2str(length(b))];     %length of b

tmp = (a(1));
str_a = ['double a[] = {' num2str(tmp)];
for i=2:length(a)
    tmp = (a(i));
    str_a = [str_a, ', ', num2str(tmp)];
end;
str_a = [str_a, '};'];

fileID = fopen('RTDSP/coeff_jerry.txt' , 'w+');
%%fprintf(fileID, '%s\n%s\n%s\n', str_b, str_a, str_len_b);   %assume a and b same length
fprintf(fileID, 'double b[] = {');
fprintf(fileID, ' %.15e,',b);
fprintf(fileID, '};\ndouble a[] = {');
fprintf(fileID, ' %.15e,',a);
fprintf(fileID, '};\n#define N %d', length(b));
fclose(fileID);
%save fir_coeff.txt str -ascii -double   %save coefficient b to txt file