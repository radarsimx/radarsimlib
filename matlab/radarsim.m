loadlibrary('radarsimc','radarsim.h');

x=calllib('radarsimc','Create_Transmitter',[1000,10001],[0,1], 2, [0,1,2], [0,1,2],3,[0], 1, 10);

calllib('radarsimc','Dump_Transmitter',x);

calllib('radarsimc','Free_Transmitter',x);

clear x;
unloadlibrary radarsimc;


% a = libpointer('doublePtr',[4 5 6]);
% b = libpointer('doublePtr',[1 2 3]);
% n = int32(3);
% s = calllib("addition","addition", a, b, n);
% s.setdatatype('doublePtr',double(n),1);
% out = s.Value;