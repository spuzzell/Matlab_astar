function mapmaker()
[MAP_D] = MapGen();
save mapdata.mat
%% 1. Generating MAPS
function [MAP_DATA] = MapGen()
% Start Positions
C = [89,78;88,40;20,30];
M_size = [90,120,height(C)+2];
% Creation of a global map
MAP = (zeros(M_size));
% Adding known obsticales layer
MAP(8:52,92:108,1) =1; MAP(24:36,24:52,1) =1;
MAP(86,91:92,1) =1; MAP(74,95:96,1) =1;
MAP(85,92:93,1) =1; MAP(73,94:95,1) =1;
MAP(84,93:94,1) =1; MAP(72,93:94,1) =1;
MAP(83,94:95,1) =1; MAP(71,92:93,1) =1;
MAP(82,95:96,1) =1; MAP(70,91:92,1) =1;
MAP(75:81,96,1) =1;
% Adding saftey margin
MAP(:,:,2)=MAP(:,:,1);
MAP(7:53,91:109,2) =1; MAP(23:37,23:53,2) =1;
MAP(86,90:92,2) =1; MAP(70,90:92,2) =1;
% Creation of a binary occupancy map
MAP_BOM = binaryOccupancyMap(logical(MAP(:,:,1)));
% Creation of Goal Layer for A*
[MAP(C(2,2),C(2,1),3),MAP(C(3,2),C(3,1),4),MAP(C(1,2),C(1,1),5)]=deal(1);
MAP_DATA.MAP_BOM = MAP_BOM;
MAP_DATA.MAP = MAP;
MAP_DATA.C = C;
end
end
