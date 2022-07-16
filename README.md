# Radar Target Generation and Detection Project

## Project Layout

- Configure the FMCW waveform based on the system requirements
- Define the range and velocity of target and simulated its displacement
- For the same simulation loop, process the  transmit and receive signal to determine the beat signal
- Perform Range FFT on the received signal to determine the Range
- Perform the CFAR processing on the output of 2nd FFT to display the target

![image](https://user-images.githubusercontent.com/31724244/179364293-8d091862-676d-4b2d-b290-ab9eaf0a4a76.png)

### 1. System requirements

I define the initial position of the target vehicle is *80m* ahead and its constant velocity is *-25m/s*.

```matlab
range = 100;         % target initial position (m)
velocity = -20;     % target initial velocity (m/s)
```

System requirements define the design of a radar.

![image](https://user-images.githubusercontent.com/31724244/179364304-e5fd26b5-48ba-4bdc-9c9f-ea93da6b457f.png)


The above requirements can then be convereted in radar chirp configuration based on: `Bsweep`, `Tchirp`, and `Slope`. 

```matlab
c = 3e8;                    % speed of light (m/s)
B = c / (2 * range_res);    % Sweep bandwidth
Tchirp = 5.5 * 2 * range_max / c;   % Sweep time
slope = B / Tchirp;         % Slope of chirp signal
```

### 2. Chirp Signal Configuration & Target Generation

FMCW transmit and received signals are defined using the wave equations, where the alpha `a` is the `Slope`. The received signal is nothing but the time delayed version of the transmit signal.

On mixing these two signals, we get the beat signal by *element-by-element multiplication* of two signal matrices: `times(Tx, Rx)` or `Tx.*Rx`.

![image](https://user-images.githubusercontent.com/31724244/179364535-bb3b68e5-6b11-4cc5-9712-c4f0bbe605f5.png)

The loop iterates all timestamps in the `t`, calculates the `range(t)` using constant velocity model and `Tx(t)`/`Rx(t)` using wave propagation equations. At the end, generate the beat signal by mixing the Tx and Rx. The following code shows represents the equation as above:

```matlab
for i=1:length(t)
    % For each time stamp update the Range of the Target for constant velocity.
    range_t(i) = range + velocity * t(i);
    td(i) = (2 * range_t(i)) / c;

    % For each time sample we need update the transmitted and received signal.
    Tx(i) = cos(2 * pi * (fc * t(i) + (slope * t(i)^2) / 2));
    Rx(i) = cos(2 * pi * (fc * (t(i) - td(i)) + (slope * (t(i) - td(i))^2) / 2));

    % Now by mixing the Transmit and Receive generate the beat signal
    % This is done by element wise matrix multiplication of Transmit and Receiver Signal
    Mix(i) = Tx(i) .* Rx(i);
end
```

### 3. FFT operation

I implement the 1D FFT on the mixed signal for range FFT. The output shows the peak at the range of the vehicle. The FFT gives a mean of *101m* with *+/-2m* std. 

![image](https://user-images.githubusercontent.com/31724244/179364846-3ed6501a-248f-4118-93cd-699681ba9ad5.png)


Then by implementing the 2D FFT (range+doppler) on the beat signal, we can extract both **Range** and **Doppler** information - a Range-Doppler Map. 

![image](https://user-images.githubusercontent.com/31724244/179364969-4aad7716-4121-48db-9678-eeffd796f773.png)


### 4. 2D CFAR

I chose to use 8 training cells and 4 guard cells in range and 7 training cells and 2 guard cells in doppler. Offset to the CFAR threshold was set to 10dB.

```matlab
Tr = 8; % training cells for range
Td = 7; % training cells for doppler

Gr = 4; % guard cells for range
Gd = 2; % guard cells for doppler

offset = 10;
```

A for-loop iterates the RDM, doing the following operations:
-Determine the number of Training cells for each dimension. Similarly, pick the number of guard cells.
-Slide the cell under test across the complete matrix. Make sure the CUT has margin for Training and Guard cells from the edges.
-For every iteration sum the signal level within all the training cells. To sum convert the value from logarithmic to linear using db2pow function.
-Average the summed values for all of the training cells used. After averaging convert it back to logarithmic using pow2db.
-Further add the offset to it to determine the threshold.
-Next, compare the signal under CUT against this threshold.
-If the CUT level > threshold assign it a value of 1, else equate it to 0.

![image](https://user-images.githubusercontent.com/31724244/179365219-8b30e41c-78df-4fc2-8d1b-dcb1f6aa0aa1.png)

The final 2D CFAR result shown below as a single peak of target detection with range at 100m and doppler as 19.7m/s within +-1m/s std
![image](https://user-images.githubusercontent.com/31724244/179365563-29928e88-b2fe-4772-ba32-d185e2242034.png)

