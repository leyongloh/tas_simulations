close all
clear
figure
hold on

nr = 70;
lidar_length = 6;
wheel_a =0;
zone_len = 3;
zone_wid = 2;

a_full = linspace(-pi/8,9*pi/8,nr);

%% Plot LIDAR
for i = 1:nr
   line([0, lidar_length*cos(a_full(i))],[0, lidar_length*sin(a_full(i))],'Color','red','LineStyle','-', 'Marker','o')
end
grid on

%% Plot measurements
tic
a = a_full(a_full> 0);
a = a(a<=pi);

max_zone_dist = zone_len*sin(pi/2-wheel_a);
max_zone_dist_per_beam = max_zone_dist./sin(a);

mask_zone_one = abs((zone_wid/2*sin(pi/2-wheel_a))./sin(pi/2-wheel_a-a));
mask_zone_one(mask_zone_one>=max_zone_dist_per_beam)=max_zone_dist_per_beam(mask_zone_one>=max_zone_dist_per_beam);

mask_zone_out = abs((3*zone_wid/2*sin(pi/2-wheel_a))./sin(pi/2-wheel_a-a));
mask_zone_out(mask_zone_out>=max_zone_dist_per_beam)=max_zone_dist_per_beam(mask_zone_out>=max_zone_dist_per_beam);
toc
for i = 1:length(mask_zone_out)
   line([0, mask_zone_one(i)*cos(a(i))],[0, mask_zone_one(i)*sin(a(i))],'Color','cyan','LineStyle','-', 'Marker','o')
   % Plot the outside zones two and three
   line([mask_zone_one(i)*cos(a(i)), mask_zone_out(i)*cos(a(i))],[mask_zone_one(i)*sin(a(i)), mask_zone_out(i)*sin(a(i))],'Color','m','LineStyle','-', 'Marker','o')
end

%% Plot Zones
mid_point_x = zone_len*sin(wheel_a);
mid_point_y = zone_len*cos(wheel_a);

% zone left
plot([-zone_wid/2-zone_wid, mid_point_x-zone_wid/2-zone_wid],[0,mid_point_y],'-go')
plot([zone_wid/2-zone_wid, mid_point_x+zone_wid/2-zone_wid],[0,mid_point_y],'-go')
plot([mid_point_x-zone_wid/2-zone_wid, mid_point_x+zone_wid/2-zone_wid],[mid_point_y,mid_point_y],'-go')

% zone right
plot([-zone_wid/2+zone_wid, mid_point_x-zone_wid/2+zone_wid],[0,mid_point_y],'-go')
plot([zone_wid/2+zone_wid, mid_point_x+zone_wid/2+zone_wid],[0,mid_point_y],'-go')
plot([mid_point_x-zone_wid/2+zone_wid, mid_point_x+zone_wid/2+zone_wid],[mid_point_y,mid_point_y],'-go')

% zone front
plot([-zone_wid/2, mid_point_x-zone_wid/2],[0,mid_point_y],'-bo')
plot([zone_wid/2, mid_point_x+zone_wid/2],[0,mid_point_y],'-bo')
plot([mid_point_x-zone_wid/2, mid_point_x+zone_wid/2],[mid_point_y,mid_point_y],'-bo')

line([-lidar_length,lidar_length],[0,0],'Color','black','Marker','o','LineWidth',0.8)