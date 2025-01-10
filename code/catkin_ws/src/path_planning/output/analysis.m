clc;
clear;

%% Set up the Import Options and import the data
opts = delimitedTextImportOptions("NumVariables", 2);

% Specify range and delimiter
opts.DataLines = [2, Inf];
opts.Delimiter = ",";

% Specify column names and types
opts.VariableNames = ["totalSegmentLengthTimestamp", "VarName2"];
opts.VariableTypes = ["double", "double"];

% Specify file level properties
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";

% Import the data
totalsegmentlength = readtable("E:\study_file\EPFL\SP\code\catkin_ws\src\path_planning\output\comparison\horizon_jp0.5_gain_6.0_0.5_1.5_4.0\total_segment_length.csv", opts);
totalsegmentlength_2 = readtable("E:\study_file\EPFL\SP\code\catkin_ws\src\path_planning\output\comparison\horizon_jp0.5_gain_6.0_0.5_1.5_4.0\total_segment_length.csv", opts);

%% Clear temporary variables
clear opts

y_1 = totalsegmentlength.totalSegmentLengthTimestamp;
x_1 = totalsegmentlength.VarName2;

figure(1);
subplot(2, 1, 1);
plot(x_1, y_1);
title('Exploration efficiency', 'FontSize', 14);
xlabel('Time (s)', 'FontSize', 12);
ylabel('Total Segment Length (m)', 'FontSize', 12);

grid on;

y_diff = diff(y_1);  % First order difference of the y_data
x_diff = x_1(2:end); % The x-axis values shift by one (due to the diff operation)

subplot(2, 1, 2);  % This is the second plot in the subplot
plot(x_diff, y_diff, 'r-');
xlabel('Time (s)', 'FontSize', 12);
ylabel('Exploration Rate (m/s)', 'FontSize', 12);
grid on;

max(y_diff)

%% %% Set up the Import Options and import the data
opts = delimitedTextImportOptions("NumVariables", 1);

% Specify range and delimiter
opts.DataLines = [1, Inf];
opts.Delimiter = ",";

% Specify column names and types
opts.VariableNames = "VarName1";
opts.VariableTypes = "double";

% Specify file level properties
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";

% Import the data
planningduration = readtable("E:\study_file\EPFL\SP\code\catkin_ws\src\path_planning\output\comparison\horizon_jp0.5_gain_6.0_0.5_1.5_4.0\planning_duration.csv", opts);
evaluationduration = readtable("E:\study_file\EPFL\SP\code\catkin_ws\src\path_planning\output\comparison\horizon_jp0.5_gain_6.0_0.5_1.5_4.0\evaluation_duration.csv", opts);


%% Clear temporary variables
clear opts

figure(2);

subplot(2, 1, 1);
plot(evaluationduration.VarName1);
title('Algorithm efficiency', 'FontSize', 14);
xlabel('Iteration', 'FontSize', 12);
ylabel('evaluation time cost(s)', 'FontSize', 12);

mean(evaluationduration.VarName1)
std(evaluationduration.VarName1)
min(evaluationduration.VarName1)
max(evaluationduration.VarName1)
sum(evaluationduration.VarName1)
grid on;

subplot(2, 1, 2);
plot(planningduration.VarName1);
xlabel('Iteration', 'FontSize', 12);
ylabel('planning time cost(s)', 'FontSize', 12);

mean(planningduration.VarName1)
std(planningduration.VarName1)
min(planningduration.VarName1)
max(planningduration.VarName1)
sum(planningduration.VarName1)

grid on;

%% Set up the Import Options and import the data
opts = delimitedTextImportOptions("NumVariables", 2);

% Specify range and delimiter
opts.DataLines = [2, Inf];
opts.Delimiter = ",";

% Specify column names and types
opts.VariableNames = ["energyConsumptionTimestamp", "VarName2"];
opts.VariableTypes = ["double", "double"];

% Specify file level properties
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";

% Import the data
energyconsumption_h = readtable("E:\study_file\EPFL\SP\code\catkin_ws\src\path_planning\output\comparison\horizon_jp0.5_gain_6.0_0.5_1.5_4.0\energy_consumption.csv", opts);
energyconsumption_v = readtable("E:\study_file\EPFL\SP\code\catkin_ws\src\path_planning\output\comparison\vicinity_jp0.5_6.0\energy_consumption.csv", opts);


%% Clear temporary variables
clear opts

% Load data
y_h = energyconsumption_h.energyConsumptionTimestamp;
x_h = energyconsumption_h.VarName2;
y_v = energyconsumption_v.energyConsumptionTimestamp;
x_v = energyconsumption_v.VarName2;

% Create figure and plot
figure(3);
plot(x_h, y_h, '-b', 'LineWidth', 1.5); % Horizon-first in blue
hold on; % Hold the plot for adding the second line
plot(x_v, y_v, '-r', 'LineWidth', 1.5); % Vicinity-first in red

% Add title, labels, grid, and legend
title('Energy Consumption Comparison', 'FontSize', 14);
xlabel('Time (s)', 'FontSize', 12);
ylabel('Energy consumption(J)', 'FontSize', 12);
grid on;
legend({'Horizon-first', 'Vicinity-first'}, 'FontSize', 12, 'Location', 'best');

% Release hold
hold off;

%% comparison 
% Specify range and delimiter
opts = delimitedTextImportOptions("NumVariables", 2);
opts.DataLines = [2, Inf];
opts.Delimiter = ",";

% Specify column names and types
opts.VariableNames = ["totalSegmentLengthTimestamp", "VarName2"];
opts.VariableTypes = ["double", "double"];

% Specify file level properties
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";

% Import the data
totalsegmentlength_1 = readtable("E:\study_file\EPFL\SP\code\catkin_ws\src\path_planning\output\comparison\horizon_jp0.5_gain_6.0_0.5_1.5_4.0\total_segment_length.csv", opts);
totalsegmentlength_2 = readtable("E:\study_file\EPFL\SP\code\catkin_ws\src\path_planning\output\comparison\jp0.5_gain_0.5_0.0_1.5_0.0\total_segment_length.csv", opts);

%% Clear temporary variables
clear opts

% Load data
y_1 = totalsegmentlength_1.totalSegmentLengthTimestamp;
x_1 = totalsegmentlength_1.VarName2;

y_2 = totalsegmentlength_2.totalSegmentLengthTimestamp;
x_2 = totalsegmentlength_2.VarName2;

figure(1);

% Subplot 1: Total Segment Length
subplot(2, 1, 1);
plot(x_1, y_1, '-b', 'LineWidth', 0.5); % First dataset in blue
hold on;
plot(x_2, y_2, '-r', 'LineWidth', 0.5); % Second dataset in red
hold off;
title('Total Segment Length', 'FontSize', 12);
xlabel('Time (s)', 'FontSize', 12);
ylabel('Total Segment Length (m)', 'FontSize', 12);
legend({'lambda\_1=6.0', 'lambda\_1=0.5'}, 'FontSize', 12, 'Location', 'best');
grid on;

% Subplot 2: Exploration Rate
y_diff_1 = diff(y_1); % First order difference for dataset 1
x_diff_1 = x_1(2:end);

y_diff_2 = diff(y_2); % First order difference for dataset 2
x_diff_2 = x_2(2:end);

subplot(2, 1, 2);
plot(x_diff_1, y_diff_1, '-b', 'LineWidth', 0.5); % First dataset in blue
hold on;
plot(x_diff_2, y_diff_2, '-r', 'LineWidth', 0.5); % Second dataset in red
hold off;
title('Exploration Rate', 'FontSize', 12);
xlabel('Time (s)', 'FontSize', 12);
ylabel('Exploration Rate (m/s)', 'FontSize', 12);
legend({'lambda\_1=6.0', 'lambda\_1=0.5'}, 'FontSize', 12, 'Location', 'best');
grid on;

%% %% Set up the Import Options and import the data
opts = delimitedTextImportOptions("NumVariables", 1);

% Specify range and delimiter
opts.DataLines = [1, Inf];
opts.Delimiter = ",";

% Specify column names and types
opts.VariableNames = "VarName1";
opts.VariableTypes = "double";

% Specify file level properties
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";

% Import the data
planningduration_1 = readtable("E:\study_file\EPFL\SP\code\catkin_ws\src\path_planning\output\comparison\horizon_jp0.5_gain_6.0_0.5_1.5_4.0\planning_duration.csv", opts);
evaluationduration_1 = readtable("E:\study_file\EPFL\SP\code\catkin_ws\src\path_planning\output\comparison\horizon_jp0.5_gain_6.0_0.5_1.5_4.0\evaluation_duration.csv", opts);

planningduration_2 = readtable("E:\study_file\EPFL\SP\code\catkin_ws\src\path_planning\output\comparison\jp0.5_gain_0.5_0.0_1.5_0.0\planning_duration.csv", opts);
evaluationduration_2 = readtable("E:\study_file\EPFL\SP\code\catkin_ws\src\path_planning\output\comparison\jp0.5_gain_0.5_0.0_1.5_0.0\evaluation_duration.csv", opts);

%% Clear temporary variables
clear opts

% Load data
evaluation_1 = evaluationduration_1.VarName1;
evaluation_2 = evaluationduration_2.VarName1;

planning_1 = planningduration_1.VarName1;
planning_2 = planningduration_2.VarName1;

% Create figure
figure(2);

% Subplot 1: Evaluation time cost comparison
subplot(2, 1, 1);
plot(evaluation_1, '-b', 'LineWidth', 0.5); % First dataset in blue
hold on;
plot(evaluation_2, '-r', 'LineWidth', 0.5); % Second dataset in red
hold off;
title('Evaluation Time Cost', 'FontSize', 12);
xlabel('Iteration', 'FontSize', 12);
ylabel('Evaluation Time Cost (s)', 'FontSize', 12);
legend({'lambda\_1=6.0', 'lambda\_1=0.5'}, 'FontSize', 12, 'Location', 'best');
grid on;

% Subplot 2: Planning time cost comparison
subplot(2, 1, 2);
plot(planning_1, '-b', 'LineWidth', 0.5); % First dataset in blue
hold on;
plot(planning_2, '-r', 'LineWidth', 0.5); % Second dataset in red
hold off;
title('Planning Time Cost', 'FontSize', 12);
xlabel('Iteration', 'FontSize', 12);
ylabel('Planning Time Cost (s)', 'FontSize', 12);
legend({'lambda\_1=6.0', 'lambda\_1=0.5'}, 'FontSize', 12, 'Location', 'best');
grid on;

% Print statistics for evaluation data
disp('Statistics for Evaluation Time Cost:');
disp(['jump\_size = 1.0m: Mean = ', num2str(mean(evaluation_1)), ', Std Dev = ', num2str(std(evaluation_1)), ...
    ', Min = ', num2str(min(evaluation_1)), ', Max = ', num2str(max(evaluation_1)), ', Sum = ', num2str(sum(evaluation_1))]);
disp(['jump\_size = 0.2m: Mean = ', num2str(mean(evaluation_2)), ', Std Dev = ', num2str(std(evaluation_2)), ...
    ', Min = ', num2str(min(evaluation_2)), ', Max = ', num2str(max(evaluation_2)), ', Sum = ', num2str(sum(evaluation_2))]);

% Print statistics for planning data
disp('Statistics for Planning Time Cost:');
disp(['jump\_size = 1.0m: Mean = ', num2str(mean(planning_1)), ', Std Dev = ', num2str(std(planning_1)), ...
    ', Min = ', num2str(min(planning_1)), ', Max = ', num2str(max(planning_1)), ', Sum = ', num2str(sum(planning_1))]);
disp(['jump\_size = 0.2m: Mean = ', num2str(mean(planning_2)), ', Std Dev = ', num2str(std(planning_2)), ...
    ', Min = ', num2str(min(planning_2)), ', Max = ', num2str(max(planning_2)), ', Sum = ', num2str(sum(planning_2))]);
