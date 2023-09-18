loadlibrary('radarsimc','radarsim.h');

x=calllib('radarsimc','Create_Transmitter',[1000,10001],[0,1], 2, [0,1,2], [0,1,2],3,[0], 1, 10);

calllib('radarsimc','Dump_Transmitter',x);

calllib('radarsimc','Free_Transmitter',x);

clear x;
unloadlibrary radarsimc;