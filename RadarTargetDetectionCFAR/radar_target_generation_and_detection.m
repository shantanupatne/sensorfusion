clear;
clc;

%% Radar Specifications 
%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Frequency of operation = 77GHz
% Max Range = 200m
% Range Resolution = 1 m
% Max Velocity = 100 m/s
%%%%%%%%%%%%%%%%%%%%%%%%%%%
maxRange = 200;
resolution = 1;
maxVel = 100;
c = 3e8;

%speed of light = 3e8
%% User Defined Range and Velocity of target
% *%TODO* :
% define the target's initial position and velocity. Note : Velocity
% remains contant
ran = 123;
vel = -56;


%% FMCW Waveform Generation

% *%TODO* :
%Design the FMCW waveform by giving the specs of each of its parameters.
% Calculate the Bandwidth (B), Chirp Time (Tchirp) and Slope (slope) of the FMCW
% chirp using the requirements above.
B = c / (2 * resolution);
Tchirp = 5.5 * 2 * maxRange / c;
slope = B / Tchirp;

%Operating carrier frequency of Radar 
fc = 77e9;             %carrier freq
                                                          
%The number of chirps in one sequence. Its ideal to have 2^ value for the ease of running the FFT
%for Doppler Estimation. 
Nd = 128;                   % #of doppler cells OR #of sent periods % number of chirps

%The number of samples on each chirp. 
Nr = 1024;                  %for length of time OR # of range cells

% Timestamp for running the displacement scenario for every sample on each
% chirp
t = linspace(0, Nd * Tchirp, Nr * Nd); %total time for samples



% *%TODO* :
%For each time stamp update the Range of the Target for constant velocity. 
r_t = ran + (vel * t);
td = 2 * r_t / c;


%% Signal generation and Moving Target simulation
% Running the radar scenario over the time. 
%Creating the vectors for Tx, Rx and Mix based on the total samples input.
Tx = cos(2 .* pi .* (fc .* t + slope .* ((t.^2) ./ 2)));
Rx = cos(2 * pi .* (fc .* (t - td) + slope .* ((t - td).^2) ./ 2));
Mix = Tx .* Rx;

%% RANGE MEASUREMENT


 % *%TODO* :
%reshape the vector into Nr*Nd array. Nr and Nd here would also define the size of
%Range and Doppler FFT respectively.
Mix = reshape(Mix, [Nr, Nd]);

 % *%TODO* :
%run the FFT on the beat signal along the range bins dimension (Nr) and
%normalize.
sig_fft = fft(Mix, Nr) ./ Nr;
 % *%TODO* :
% Take the absolute value of FFT output
sig_fft = abs(sig_fft);

 % *%TODO* :
% Output of FFT is double sided signal, but we are interested in only one side of the spectrum.
% Hence we throw out half of the samples.
sig_fft = sig_fft(1 : Nr / 2);

%plotting the range
figure ('Name','Range from First FFT')
% *%TODO* :
% plot FFT output 

plot(sig_fft);
xlabel("Range")
title("Range from FFT")
axis ([0 200 0 1]);



%% RANGE DOPPLER RESPONSE
% The 2D FFT implementation is already provided here. This will run a 2DFFT
% on the mixed signal (beat signal) output and generate a range doppler
% map.You will implement CFAR on the generated RDM


% Range Doppler Map Generation.

% The output of the 2D FFT is an image that has reponse in the range and
% doppler FFT bins. So, it is important to convert the axis from bin sizes
% to range and doppler based on their Max values.

Mix=reshape(Mix,[Nr,Nd]);

% 2D FFT using the FFT size for both dimensions.
sig_fft2 = fft2(Mix,Nr,Nd);

% Taking just one side of signal from Range dimension.
sig_fft2 = sig_fft2(1:Nr/2,1:Nd);
sig_fft2 = fftshift (sig_fft2);
RDM = abs(sig_fft2);
RDM = 10*log10(RDM) ;

%use the surf function to plot the output of 2DFFT and to show axis in both
%dimensions
doppler_axis = linspace(-100,100,Nd);
range_axis = linspace(-200,200,Nr/2)*((Nr/2)/400);
figure;
surf(doppler_axis, range_axis, RDM);
title("2D FFT");


%% CFAR implementation

%Slide Window through the complete Range Doppler Map

% *%TODO* :
%Select the number of Training Cells in both the dimensions.
Tr = 12;
Td = 5;
% *%TODO* :
%Select the number of Guard Cells in both dimensions around the Cell under 
%test (CUT) for accurate estimation
Gr = 4;
Gd = 2;

% *%TODO* :
% offset the threshold by SNR value in dB
offset = 13;

% *%TODO* :
%Create a vector to store noise_level for each iteration on training cells
noise_level = zeros(1,1);
r_size = 2 * (Tr + Gr) + 1;
d_size = 2 * (Td * Gd) + 1;

% *%TODO* :
%design a loop such that it slides the CUT across range doppler map by
%giving margins at the edges for Training and Guard Cells.
%For every iteration sum the signal level within all the training
%cells. To sum convert the value from logarithmic to linear using db2pow
%function. Average the summed values for all of the training
%cells used. After averaging convert it back to logarithimic using pow2db.
%Further add the offset to it to determine the threshold. Next, compare the
%signal under CUT with this threshold. If the CUT level > threshold assign
%it a value of 1, else equate it to 0.

% *%TODO* :
% The process above will generate a thresholded block, which is smaller 
%than the Range Doppler Map as the CUT cannot be located at the edges of
%matrix. Hence,few cells will not be thresholded. To keep the map size same
% set those values to 0. 

num_training_cells = r_size * d_size - (2 * Gr + 1) * (2 * Gd + 1);
signal_cfar = zeros(Nr/2, Nd);
for i = 1 + Tr + Gr : (Nr/2 - (Gr + Tr))
    for j = 1 + Td + Gd : (Nd - (Gd + Td))
        sum_window_cells = sum(db2pow(RDM(i - (Tr + Gr) : i + Tr + Gr, j - (Td + Gd) : j + Td + Gd)), 'all');
        sum_guard_cell  = sum(db2pow(RDM(i - Gr : i + Gr, j - Gd : j + Gd)), 'all');
        noise_level = sum_window_cells - sum_guard_cell;

        threshold = noise_level / num_training_cells;
        threshold = pow2db(threshold) + offset;

        signal = RDM(i,j);
        signal_cfar(i, j) = signal > threshold;
    end
end

% *%TODO* :
%display the CFAR output using the Surf function like we did for Range
%Doppler Response output.
figure;
surf(doppler_axis, range_axis, signal_cfar, 'EdgeColor','none', 'FaceColor','interp');
title("CFAR Detection")
colorbar;


 
 