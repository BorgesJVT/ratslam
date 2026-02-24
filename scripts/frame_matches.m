% frame_matches.m
% 
% .m file to load reported frame matches and visualize the two frames
% 
% Copyright (C) 2012
% Michael Milford (michael.milford@qut.edu.au)
% 
% 1. Queensland University of Technology, Australia
% 
% This program is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
% 
% You should have received a copy of the GNU General Public License
% along with this program.  If not, see <http://www.gnu.org/licenses/>.

clc;
clear all;
% close all;

% m1 = csvread('vt_id.dat', 1, 0);
% m1 = csvread('exported_data/vt_id.csv', 1, 0);
% ml = length(m1);
% start = m1(1, 1) / 1e9;
% t1 = m1(1:ml, 1) / 1e9 - start;

vt_data = readtable('exported_data/vt_id.csv');
m_length = length(vt_data.stamp_sec);
abs_time_ns = vt_data.stamp_sec*10^9 + vt_data.stamp_nsec;
start = abs_time_ns(1);
k = abs_time_ns - start; % discrete time starting from 0
% t1 = m1.stamp_sec - start;

frame_offset = 0;
match_buffer = 10;   % This is the minimum "gap" between the current frame and the matched frame that we will generate image output for.

% v1 = m1(1:ml, 5);
% v1 = v1 + 1;

% 
% vn = 0;
% vframes = zeros(length(v1), 1) - 1;
% 
% pair_match = [];
% 
% for c = 1:length(v1)
%     if v1(c) > vn
%        vn = v1(c);
%        vframes(v1(c)) = m1(c, 2) + 1;
%     elseif v1(c) < vn - match_buffer
%         pair_match = [pair_match; c vframes(v1(c))];
% 
%     end
% end

vt_ids = vt_data.current_id; % current active vt id
vt_ids = vt_ids + 1; % first index in matlab arry must be 1
vn = 0;
vframes = zeros(length(vt_ids), 1) - 1; % vframes inicializado com -1 para armazenar o frame 
% index mais recente para cada vt_id. 
frames_index = 1:m_length; % Starting from 1 because MATLAB is 1-based index 

pair_match = [];

for counter = 1:length(vt_ids)
    if vt_ids(counter) > vn
       vn = vt_ids(counter);
       vframes(vt_ids(counter)) = frames_index(counter); 
    elseif vt_ids(counter) < vn - match_buffer
        pair_match = [pair_match; counter vframes(vt_ids(counter))];
        
    end
end

pair_match = pair_match + frame_offset;
pair_match_with_timesec_vtid = zeros(length(pair_match),5)

for i = 1:length(pair_match)
    pair_match_with_timesec_vtid(i,:) = [pair_match(i,1) k(pair_match(i,1))/10^9 pair_match(i,2) k(pair_match(i,2))/10^9 vt_ids(pair_match(i,2))-1];
    % vt_ids(pair_match(i,2))-1 because vt_ids was increased by 1 at 63
    % line
end

% mov = mmreader('E:\datasets\car_driving\stlucia_large\stluciahq.avi');
% mov = mmreader('E:\datasets\iRat\aus_map\log_irat_red.avi');
% % mov = mmreader('newcollege_pano.avi');
% num_frames = get(mov, 'numberOfFrames');
% fh = get(mov, 'Height');
% fw = get(mov, 'Width');
% disp(['Num frames: ' int2str(num_frames) '. (width, height): ' int2str([fw fh])])
% 
% npmatches = size(pair_match);
% npmatches = npmatches(1);
% 
% length(pair_match)
% 
% for i = 1:length(pair_match)
%     im1 = [];
%     im2 = [];
% 
%     %     Horizontal arrangement
%     buf_width_vert = 20;
%     buf_width_horz = buf_width_vert;
% 
% 
%     vid_frame1 = read(mov, pair_match(i, 1));
%     vid_frame2 = read(mov, pair_match(i, 2));
%     tsz = size(vid_frame1);
%     vert_buffer = zeros(tsz(1), buf_width_vert, 3) + 255;
% 
%     im_comb = [vid_frame1 vert_buffer vid_frame2];
% 
%     imagesc(im_comb);
%     axis off;
% 
%     axis tight;
% 
%     set(gca,'LooseInset', [0 0 0 0])
%     set(gca,'Position', [0 0 1 1]);
% 
% 
%     im_comb_sz = size(im_comb);
%     ratio = im_comb_sz(2) / im_comb_sz(1);
%     psize = 2;
%     set(gcf, 'PaperPosition', [0  0.0  psize * ratio   psize]);
% 
%     drawnow;
%     pause(0.01);
% 
% 
%     framename = sprintf('frame_%.6d.jpg', i);
% 
%     print('-r200', '-djpeg100', framename);
% 
% end