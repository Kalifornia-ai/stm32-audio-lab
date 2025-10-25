fs = 8000;
x  = load('tone500.txt');

% deinterleave stereo -> left channel
xL = x(1:2:end);

N = length(xL);
t = (0:N-1)/fs;

figure(1); clf;

% Time-domain
subplot(2,1,1);
plot(t, xL, 'LineWidth', 1);
xlabel('Time [s]');
ylabel('Amplitude');
title('Time-Domain Signal (500 Hz tone)');
grid on;

% FFT
X = abs(fft(xL));
f = (0:N-1)*fs/N;

% normalize, dB
YdB = 20*log10(X(1:N/2)/max(X));

% clip floor for nice plotting
YdB(YdB < -100) = -100;

subplot(2,1,2);
semilogx(f(1:N/2), YdB, 'LineWidth', 1);
xlabel('Frequency [Hz]');
ylabel('Magnitude [dB]');
title('Log-Magnitude Spectrum (500 Hz tone)');
grid on;
xlim([100 4000]);     % just look up to Nyquist
ylim([-100 5]);       % show headroom above 0 dB
