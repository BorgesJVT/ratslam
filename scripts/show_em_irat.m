%% Show Experience Map

clear
close all;
clc;

% Set to true to save a experience map evolution video
save_video = false;

% Set to true to save partial figures of map evolution
save_figures = false;

if save_video
    numFrames = 50; % Número de iterações/timesteps
    videoFile = 'Figures/Exp_Map/experience_map_evolution.avi';
    vidObj = VideoWriter(videoFile);
    vidObj.FrameRate = 4;
    vidObj.Quality = 100; 
    open(vidObj);
end

% Load experience map data
nodes = readtable('exported_data/nodes.csv');
links = readtable('exported_data/links.csv');

% Load Ground Truth
GT_table = readtable('exported_data/irat_gt.csv');

% extract lat and long
n2 = height(GT_table);
x = table2array(GT_table(1:n2,"x"));
y = table2array(GT_table(1:n2,"y"));

n = height(nodes);
nodes_x = [0];
nodes_y = [0];
id = 0;

min_x = min([x; nodes.x]);
max_x = max([x; nodes.x]);
delta_x = max_x-min_x;
x_lim = round([min_x-0.05*delta_x max_x+0.05*delta_x]);
min_y = min([y; nodes.y]);
max_y = max([y; nodes.y]);
delta_y = max_y-min_y;
y_lim = round([min_y-0.05*delta_y max_y+0.05*delta_y]);

fig_counter = 1;
figure'

i = 2;
while i <= n
        while (nodes.id(i) >= nodes.id(i-1))
            nodes_x = [nodes_x; nodes.x(i)];
            nodes_y = [nodes_y; nodes.y(i)];
            disp(nodes.id(i))
            i = i + 1;
            node_time_stamp = nodes.stamp_sec(i-1) % Time stemp of last map publish
            if (i > n)
                break
            end
        end
            [x, y] = ground_truth_cutting(node_time_stamp, GT_table);
            plot(x,y,'LineWidth',1.5,'Color','b','LineStyle','-')
            xlim(x_lim); ylim(y_lim);
            sz = 75; % Scatter marke size
            hold on
            scatter(nodes_x,nodes_y,'red','filled','Marker','o','SizeData',10)
            scatter(nodes_x(1),nodes_y(1),'red','filled','Marker','o','SizeData',sz)
            scatter(x(1),y(1),'blue','filled','Marker','o','SizeData',sz)
            scatter(x(end),y(end),'blue','diamond','filled','SizeData',sz)
            scatter(nodes_x(end),nodes_y(end),'red','diamond','filled','SizeData',sz)
            hold off
            title('Experience Map','FontSize',14,'Interpreter','latex')
            xlabel('$x$ (m)','FontSize',14,'Interpreter','latex');
            ylabel('$y$ (m)','FontSize',14,'Interpreter','latex');
            legend('ground truth','trajectory','Interpreter','latex','Location','best')            
            grid on
            hold off
            drawnow
      
            i = i + 1;

            if (i <= n) % Verify if doesn't exceed the array length
                  nodes_x = [nodes.x(i)];
                  nodes_y = [nodes.y(i)];
            end
            disp(i)
            if save_video
                writeVideo(vidObj, getframe(gcf));
            end
            if save_figures
                figure_name = "Figures/Exp_Map/map_evolution/PartialMap_"+num2str(fig_counter);
                print('-dpng', '-r600', figure_name+'.png');
                print('-depsc2', '-r600', figure_name+'.eps');
                fig_counter = fig_counter+1;
            end
end

%% Offset Correction
off_setx = nodes_x(1);
off_sety = nodes_y(1);

nodes_x = nodes_x - off_setx;
nodes_y = nodes_y - off_sety;

off_setx = x(1);
off_sety = y(1);

x = x - off_setx;
y = y - off_sety;

plot(x,y,'LineWidth',1.5,'Color','b','LineStyle','-')
% xlim(x_lim); ylim(y_lim);
sz = 75; % Scatter marke size
hold on
scatter(nodes_x,nodes_y,'red','filled','Marker','o','SizeData',20)
scatter(nodes_x(1),nodes_y(1),'red','filled','Marker','o','SizeData',sz*1.25)
scatter(x(1),y(1),'blue','filled','Marker','o','SizeData',sz*1.25)
scatter(x(end),y(end),'blue','diamond','filled','SizeData',sz*1.25)
scatter(nodes_x(end),nodes_y(end),'red','diamond','filled','SizeData',sz*1.25)
hold off
title('Experience Map - offset correction','FontSize',14,'Interpreter','latex')
xlabel('$x$ (m)','FontSize',14,'Interpreter','latex');
ylabel('$y$ (m)','FontSize',14,'Interpreter','latex');
legend('ground truth','trajectory','Interpreter','latex','Location','best')            
grid on
hold off
drawnow

if save_video
    writeVideo(vidObj, getframe(gcf));
end
if save_figures
    figure_name = "Figures/Exp_Map/map_evolution/PartialMap_"+num2str(fig_counter);
    print('-dpng', '-r600', figure_name+'.png');
    print('-depsc2', '-r600', figure_name+'.eps');
    fig_counter = fig_counter+1;
    print('-dpng', '-r600', 'Figures/Exp_Map/Final_Exp_map.png');
    print('-depsc2', '-r600', 'Figures/Exp_Map/Final_Exp_map.eps');
end


if save_video
    close(vidObj);
    % Convert to mp4: Need ffmeg in PATH
    system('ffmpeg -i Figures/Exp_Map/experience_map_evolution.avi -vf "scale=570:414" -c:v libx264 Figures/Exp_Map/experience_map_evolution.mp4');
end




%% Error Computation

last_node_count = nodes.node_count(end) 
idx_first = find(nodes.node_count == last_node_count,1)
last_map_table = nodes(idx_first:end, :);

% Find equivalent GT (GPS coordinates) by timestamp
new_len = height(last_map_table)
new_x = zeros(new_len,1);
new_y = zeros(new_len,1);

for i=1:new_len
    timestamp = last_map_table.stamp_sec(i);
    position = find(GT_table.stamp_sec == timestamp, 1);
    new_x(i) = GT_table.x(position);
    new_y(i) = GT_table.y(position);
end

em_x = table2array(last_map_table(1:new_len,"x"));
em_y = table2array(last_map_table(1:new_len,"y"));

% Offset Correction
off_setx = em_x(1);
off_sety = em_y(1);
em_x = em_x - off_setx;
em_y = em_y - off_sety;

off_setx = new_x(1);
off_sety = new_y(1);
new_x = new_x - off_setx;
new_y = new_y - off_sety;


erro_x = new_x - em_x;
erro_y = new_y - em_y;

erro_dist = zeros(new_len,1);
erro_sqrt = zeros(new_len,1);
for i=1:new_len
    erro_dist(i) = sqrt(erro_x(i)^2 + erro_y(i)^2);
    % erro_sqrt(i) = erro_x(i)^2 + erro_y(i)^2;
end
% erro_dist is always positive. It is the Euclidian distance between two
% points of both trajectories

% RMSe = sqrt(sum(erro_sqrt)/new_len)
erro_med = sum(erro_dist)/new_len;
k = 0:new_len-1;
[max_erro_dist, max_idx] = max(erro_dist)

figure'
plot(k,erro_dist,'LineWidth',1.5,'Color','b','LineStyle','-')
hold on
plot(k,erro_med*ones(new_len,1),'LineWidth',1.5,'Color','r','LineStyle','--')
xlim([0 new_len-1])
grid on
% Show coordinates next to point
lbl = sprintf('x = %i\ny = %0.2f', max_idx, max_erro_dist);
text(max_idx, max_erro_dist, lbl, 'VerticalAlignment', 'bottom', ...
     'HorizontalAlignment', 'left', 'FontSize', 10, 'BackgroundColor','y','Margin',2);
stem(max_idx,max_erro_dist,'LineWidth',1.5,'Color','k')
hold off
title(['Trajectory error - Average = ' num2str(erro_med) ' m'],'FontSize',12,'Interpreter','latex')
xlabel('$k$ (samples)','FontSize',14,'Interpreter','latex');
ylabel('Euclidian distance','FontSize',14,'Interpreter','latex');
legend('Euclidian distance','Average error','Maximum error','Interpreter','latex','Location','best') 

print('-dpng', '-r600', 'Figures/Exp_Map/Distance_error.png');
print('-depsc2', '-r600', 'Figures/Exp_Map/Distance_error.eps');

figure'
plot(x,y,'LineWidth',1.5,'Color','b','LineStyle','-')
hold on
scatter(em_x,em_y,'red','filled','Marker','o','SizeData',20)
plot([em_x(max_idx) new_x(max_idx)],[em_y(max_idx) new_y(max_idx)],'LineWidth',1.5,'Color','k','LineStyle','--')
sz = 75; % Scatter marke size

scatter(em_x(1),em_y(1),'red','filled','Marker','o','SizeData',sz*1.25)
scatter(x(1),y(1),'blue','filled','Marker','o','SizeData',sz*1.25)
scatter(x(end),y(end),'blue','diamond','filled','SizeData',sz*1.25)
scatter(em_x(end),em_y(end),'red','diamond','filled','SizeData',sz*1.25)
scatter(em_x(max_idx),em_y(max_idx),'black','diamond','filled','SizeData',sz*1.25)
scatter(new_x(max_idx),new_y(max_idx),'black','diamond','filled','SizeData',sz*1.25)
title(['Experience Map - Maximum error = ' num2str(max_erro_dist) ' m'], ...
    'FontSize',14,'Interpreter','latex')
xlabel('$x$ (m)','FontSize',14,'Interpreter','latex');
ylabel('$y$ (m)','FontSize',14,'Interpreter','latex');
legend('ground truth','trajectory','maximum error','Interpreter','latex','Location','best')            
grid on

print('-dpng', '-r600', 'Figures/Exp_Map/Maximum_error.png');
print('-depsc2', '-r600', 'Figures/Exp_Map/Maximum_error.eps');

% Export to .csv file
% headers = {'stamp_sec', 'Latitude', 'Longitude','x_estimated','y_estimated'};
% stamp_sec = 0:1:new_len-1;
% data = [stamp_sec' x_resampled y_resampled nodes_x nodes_y];

%%
function [x, y] = ground_truth_cutting(last_node_time_stamp, GT_table)

% % find time_stamp in GT_table
% [found, position] = ismember(last_node_time_stamp, GT_table.stamp_sec);
% if found
%     disp(position)
%     linha_encontrada = GT_table(position, :)
% end

i = 1;
while GT_table.stamp_sec(i) < last_node_time_stamp
    position = i;
    i = i + 1;
end

% cut gps table to last map time stamp
GT_table = GT_table(1:position-1, :);

% extract lat and long
n2 = height(GT_table);
x = table2array(GT_table(1:n2,"x"));
y = table2array(GT_table(1:n2,"y"));

end