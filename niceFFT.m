function [freq, pow] = niceFFT(signal, fs, do_plot)

if nargin < 3
  do_plot = 0;
end

if(size(signal,2) > size(signal,1))
  signal = signal';
end
  
N = length(signal);

signal = signal.* hann(N);

NFFT = 2^nextpow2(N); % Next power of 2 from length of y
Y = fft(signal,NFFT)/N;
pow = 2*abs(Y(1:NFFT/2+1));
freq = fs/2*linspace(0,1,NFFT/2+1);

% Plot single-sided amplitude spectrum.
if(do_plot)
  figure()
  plot(freq,pow)
end