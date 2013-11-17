clear
clc
format long g
clf
hold all

files= dir('*.csv');
num_files = length(files);
for file_id=1:1 %num_files
     
    sampled_data = csvread(files(file_id).name,1,0);

    %assign columns into seperate variable names - this is made quickly using the
    %header names and find-replace command
    i=1;
    timestamp=sampled_data(:,i);i=i+1;e0_pos=sampled_data(:,i);i=i+1;e0_vel=sampled_data(:,i);i=i+1;e0_eff=sampled_data(:,i);i=i+1;e0_pos_cmd=sampled_data(:,i);i=i+1;e1_pos=sampled_data(:,i);i=i+1;e1_vel=sampled_data(:,i);i=i+1;e1_eff=sampled_data(:,i);i=i+1;e1_pos_cmd=sampled_data(:,i);i=i+1;s0_pos=sampled_data(:,i);i=i+1;s0_vel=sampled_data(:,i);i=i+1;s0_eff=sampled_data(:,i);i=i+1;s0_pos_cmd=sampled_data(:,i);i=i+1;s1_pos=sampled_data(:,i);i=i+1;s1_vel=sampled_data(:,i);i=i+1;s1_eff=sampled_data(:,i);i=i+1;s1_pos_cmd=sampled_data(:,i);i=i+1;w0_pos=sampled_data(:,i);i=i+1;w0_vel=sampled_data(:,i);i=i+1;w0_eff=sampled_data(:,i);i=i+1;w0_pos_cmd=sampled_data(:,i);i=i+1;w1_pos=sampled_data(:,i);i=i+1;w1_vel=sampled_data(:,i);i=i+1;w1_eff=sampled_data(:,i);i=i+1;w1_pos_cmd=sampled_data(:,i);i=i+1;w2_pos=sampled_data(:,i);i=i+1;w2_vel=sampled_data(:,i);i=i+1;w2_eff=sampled_data(:,i);i=i+1;w2_pos_cmd=sampled_data(:,i);i=i+1;

    %plot(timestamp, s0_vel, '', 'DisplayName', strcat('actual ',num2str(file_id)));
    %plot(timestamp, s0_pos_cmd, '--', 'DisplayName', strcat('command ',num2str(file_id)))
    
    plot(timestamp, w2_vel, '', 'DisplayName', strcat('actual ',num2str(file_id)));
    plot(timestamp, w2_pos_cmd, '--', 'DisplayName', strcat('command ',num2str(file_id)))
    
    %plot(timestamp, e0_vel, '', 'DisplayName', strcat('actual ',num2str(file_id)));
    %plot(timestamp, e0_pos_cmd, '--', 'DisplayName', strcat('command ',num2str(file_id)))
    
    % neutralize
    %e1_pos_cmd = e1_pos_cmd - max(e1_pos);
    %e1_pos = e1_pos - max(e1_pos);
    %plot(timestamp, e1_vel, '', 'DisplayName', strcat('actual ',num2str(file_id)));
    %plot(timestamp, e1_pos_cmd, '--', 'DisplayName', strcat('command ',num2str(file_id)))
    
    %plot(timestamp, w0_vel, '', 'DisplayName', strcat('actual ',num2str(file_id)));
    %plot(timestamp, w0_pos_cmd, '--', 'DisplayName', strcat('command ',num2str(file_id)))
    
    %plot(timestamp, w1_vel, '', 'DisplayName', strcat('actual ',num2str(file_id)));
    %plot(timestamp, w1_pos_cmd, '--', 'DisplayName', strcat('command ',num2str(file_id)))
    
    %plot(timestamp, w2_vel, '', 'DisplayName', strcat('actual ',num2str(file_id)));
    %plot(timestamp, w2_pos_cmd, '--', 'DisplayName', strcat('command ',num2str(file_id)))
    
end
     
legend('-DynamicLegend')
xlabel('Time')
ylabel('Command')
title('Position-control response of joint left\_w1 on Baxter')