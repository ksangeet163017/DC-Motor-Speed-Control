clc;
clear;
close all;

% Define the parameters for the DC motor (example values)
K = 0.01;  % Motor constant (Nm/A)
J = 0.01;  % Moment of inertia (kg.m^2)
B = 0.1;   % Damping coefficient (Nm.s)

% Open-loop Transfer Function (without PID)
s = tf('s');
open_loop_tf = K / (J*s + B);  % Open-loop transfer function

% PID controller (example values)
Kp = 100;
Ki = 200;
Kd = 10;
pid_controller = pid(Kp, Ki, Kd);

% Closed-loop Transfer Function with PID
closed_loop_tf = feedback(pid_controller * open_loop_tf, 1);  % Feedback loop for closed-loop response

% Set video frame rate
frame_rate = 30;      % Frames per second

% Create Video Writer Object for Side-by-Side Comparison (AVI format)
video = VideoWriter('dc_motor_comparison_full_simulation_16x9_1920x1080.avi', 'Motion JPEG AVI');
video.FrameRate = frame_rate;
open(video);

% Generate Full Simulation Animation
fig = figure;
set(fig, 'Position', [100, 100, 1920, 1080]);  % 16:9 aspect ratio (1920x1080 resolution)

% Plot Open-Loop Response (Left)
subplot(1, 2, 1);  % Create the left plot (open-loop)
hold on;
grid on;
title('Open-Loop Response (No PID)');
xlabel('Time (seconds)');
ylabel('Speed Response');
[y1, t1] = step(open_loop_tf, 10);  % Open-loop system response for 10 seconds
h1 = plot(t1, y1, 'r', 'LineWidth', 2);  % Open-loop response in red

% Plot Closed-Loop Response (Right)
subplot(1, 2, 2);  % Create the right plot (closed-loop)
hold on;
grid on;
title('Closed-Loop Response with PID');
xlabel('Time (seconds)');
ylabel('Speed Response');
[y2, t2] = step(closed_loop_tf, 10);  % Closed-loop system response for 10 seconds
h2 = plot(t2, y2, 'b', 'LineWidth', 2);  % Closed-loop response in blue

% Add legends for clarification
subplot(1, 2, 1);
legend('Open-Loop Response (No PID)');
subplot(1, 2, 2);
legend('Closed-Loop Response with PID');

% Determine the maximum time length (make sure both simulations run until the end)
max_time = max(t1(end), t2(end));  % Take the larger of the two times (for synchronization)

% Loop to capture frames for the comparison
num_frames = frame_rate * max_time;  % Calculate the total number of frames based on the max time and frame rate

for i = 1:num_frames
    % Calculate the current time
    current_time = i / frame_rate;  % Time for the current frame
    
    % Update the plot with the corresponding data up to the current time
    subplot(1, 2, 1);
    set(h1, 'XData', t1(t1 <= current_time), 'YData', y1(t1 <= current_time));  % Update open-loop response
    
    subplot(1, 2, 2);
    set(h2, 'XData', t2(t2 <= current_time), 'YData', y2(t2 <= current_time));  % Update closed-loop response
    
    % Capture frame
    drawnow;  % Ensure the plot is updated
    frame = getframe(fig);
    
    % Resize the frame to match the expected video size (1920x1080 for 16:9 ratio)
    frame_resized = imresize(frame.cdata, [1080, 1920]);

    % If the frame is valid, write to video
    if ~isempty(frame_resized)
        writeVideo(video, frame_resized);
    end
end

% Finish recording and close the video
close(video);  % Finish recording
disp('Side-by-Side comparison video with 16:9 aspect ratio and full simulation duration saved as dc_motor_comparison_full_simulation_16x9_1920x1080.avi');
