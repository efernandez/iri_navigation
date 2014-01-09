% read a generic .pgm grid map file and return the force map
% read_pgm_grid_conv('mr_lab.pgm', 1,1,0,1)
% Input:
%	maps_file : string
%
% Output: saves the resultant .txt

function read_pgm_grid_conv( map_file, calc_flag, vis_flag, write_flag, check_flag  )

if calc_flag
%clear all;close all;
%map_file='mr_lab.pgm';

%load a ros map, using the yaml specifications
%the problem is imread loads the map in image coords and not in
%cartesians...

map = imread(map_file);
map_resolution= 0.20000;
%origin= [-80.4000, -10.0000, 0.000000];
%origin= [-14.750000, 13.90000, 0.000000];
origin= [-145.800000, -56.200000, 0.000000];
occupied_thresh= 0.65;
free_thresh= 0.196;
[N,M]=size(map);
%param = [k, lambda, A, B, d]
%param = [0, 1, 0.1, 5, 0];
param = [0, 1, 1, 5, 0];
w = 1/map_resolution; %time window of 5 m per side

%precalculate the force surrounding an obstacle:
x_cen = (w+1) * map_resolution;
y_cen = (w+1) * map_resolution;
for i = 1:2*w+1
	for j = 1:2*w+1
		x_ii = i * map_resolution;
		y_jj = j * map_resolution;
		f(i,j).force = sphere( param, [x_ii;y_jj], [x_cen;y_cen] );
        %f_deb(i,j) = norm(f(i,j).force);
	end
end
f(w+1,w+1).force = [0;0];
%contour(f_deb);

% calcules vector force corresponding to every free point in the space
for i = 1:N
	for j = 1:M
        force_map(i,j).force = [0;0];
    end
end
for i = 1:N%row
	for j = 1:M%column
		if map(i, j) <= 205 %occupied_thresh
			force_map_obs(i,j) = 0;
			% convolution
			iit_map_y_min = i - w;
			if iit_map_y_min < 1 , iit_map_y_min = 1; end;

			iit_map_y_max = i + w;
			if iit_map_y_max > N , iit_map_y_max = N; end;

			iit_map_x_min = j - w;
			if iit_map_x_min < 1 , iit_map_x_min = 1; end;

			iit_map_x_max = j + w;
			if iit_map_x_max > M , iit_map_x_max = M; end;
			
            for ii = iit_map_y_min:iit_map_y_max
                for jj = iit_map_x_min:iit_map_x_max
                    force_map(ii,jj).force = force_map(ii,jj).force +...
                       f( ii-iit_map_y_min+1 , jj-iit_map_x_min+1 ).force;
                end
            end
            

		else
			force_map_obs(i,j) = 1;
		end
	end
	i
end
end
%position test
%for i = 0:90
%	force_map(1+i,51).force = [1; 5];
%end

% Visualization
if vis_flag
    for i = 1:N
        for j = 1:M
            %quiver( j , i, force_map(i,j).force(2) , force_map(i,j).force(1) );
            cont_map(i,j) = norm(force_map(i,j).force);
        end
    end
    figure;
    contour(cont_map);
    figure;

    imshow(force_map_obs);
    hold on;
    for i = 1:5:N
        for j = 1:5:M
            quiver( j , i, force_map(i,j).force(2) , force_map(i,j).force(1) );
            %cont_map(i,j) = norm(force_map(i,j).force);
        end
    end
    hold off;
end

% writing on a file: C style, dual to the reading
if write_flag
fid = fopen('force_map.txt', 'w');
%fprintf(fid,'% xmin; xmax; ymin; ymax; resolution; force(1)_x; force(1)_y;...');

fprintf(fid,'%f',origin(1));%min_x
fprintf(fid,'\n%d',M);%num_elem_x
fprintf(fid,'\n%f',origin(2));%min_y
fprintf(fid,'\n%d',N);%num_elem_y
fprintf(fid,'\n%f',map_resolution);
for i = 1:N
    for j = 1:M
        fprintf(fid,'\n%f',force_map(end-i+1,j).force(2));%fx
        fprintf(fid,' %f',-force_map(end-i+1,j).force(1));%fy
        fprintf(fid,' %d', force_map_obs(end-i+1,j));%obstacle
    end
end
fclose(fid);
end

% check the correct reading
if check_flag
vv = load('force_map.txt');
v = vv(6:end);
v = reshape(v, 3, length(v)/3 );
v = v';
figure;
imshow(force_map_obs);
hold on;
for i = 1:5:N
    for j = 1:5:M
        y = vv(1) + (i-1)*map_resolution;
        x = vv(3) + (j-1)*map_resolution;
        iit = floor( (x-vv(1)) / vv(5))   + floor((y-vv(3)) / vv(5))*M +1;
        quiver( j , i, v(iit,1) , v(iit,2) );
    end
end
hold off;
end

