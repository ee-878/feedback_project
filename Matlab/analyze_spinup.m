%%
clear all
close all
clc


file = "/Users/evan/Documents/Arduino/feedback_project/motor_spin_up_7V_time.csv";
data = importfile(file);
data = data(:, 2:5);
data = data(all(~strcmp(data,""), 2),:); % remove rows with empty cells

% data format:
% angle (deg), pulses, speed (deg/s), % time (microseconds (E-6))

numeric_data = [];
for i = 1:size(data,1)
    numeric_data(i,:) = str2num(char(strrep(data(i,:),',','.')'));
end

numeric_data(:,4) = numeric_data(:,4)./(1E6); % seconds

% low pass filter of speed
% a = 100;
% for k = 2:length(numeric_data(:,3))
%     dt = numeric_data(k,4) - numeric_data(k-1,4);
%     y_dot = a*numeric_data(k,3) - a*numeric_data(k-1,3);
%     numeric_data(k,3) = numeric_data(k-1,3) + numeric_data(k,3)*dt;
% end

[b, a] = butter(2, 0.035);
numeric_data(:,3) = filtfilt(b,a,numeric_data(:,3));

figure
plot(numeric_data(:,4), numeric_data(:,3));
title('Angular Speed');
xlabel('Time (s)');
ylabel('Degrees/s');