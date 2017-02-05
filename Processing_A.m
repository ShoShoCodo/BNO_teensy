%By: Shoa Russell 
%analog parser
% This program parses log files and plots the sensor data.

 tic;
% clc;
 clear;
 close all;

%Scalers to make the values mean something 
Vref= 2.5;
scaler=Vref/4095;
scaler2pounds=0;


 PRESS_BLOCKSIZE = 4*100;
 BLOCKSIZE = 4*PRESS_BLOCKSIZE;

% Open the log file
[Name, pathName] = uigetfile('*.bin', 'Select Log File');% selects the folder to pull data from 
%{ 
BaseName='Pressure_test';
 EndFileName='_raw.bin'; % picks which file you  will use 
 EndFileName='.bin';
 ShortEndName = EndFileName(1:end-4);
 mkdir(pathName,'figures')
 fileName=[BaseName,num2str(k),EndFileName]
 savelocation=[pathName,'figures',num2str(k)]
 while(fileName)
 fileID = fopen([pathName, fileName]);
%}

%while(fileName)
fileID = fopen([pathName, Name]);
rawData = fread(fileID, 'uint8');
numBlocks = floor(length(rawData)/BLOCKSIZE); % Only process whole blocks

if(numBlocks)
% Pre-allocate buffers
PRESSData = zeros(1, PRESS_BLOCKSIZE*(numBlocks-BLOCKSIZE));
% Parse the raw data
for n = 0:numBlocks-1
    % Parse PRESS Data: 4 bytes -> float
    for i=1:PRESS_BLOCKSIZE     
        val=i*4-3+(BLOCKSIZE*n);
        Datatemp3=rawData(i*4-3+(BLOCKSIZE*n));
        Datatemp2=bitshift(rawData(i*4-2+(BLOCKSIZE*n)), 8, 'uint32');
        Datatemp1=bitshift(rawData(i*4-1+(BLOCKSIZE*n)), 16, 'uint32');
        Datatemp0=bitshift(rawData(i*4+(BLOCKSIZE*n)), 24, 'uint32');
        PRESSData(i+PRESS_BLOCKSIZE*n) = ...
            typecast( ...
                uint32(bitor( ...
                    uint32(bitor( Datatemp3,Datatemp2,'uint32')), ...
                    uint32(bitor( Datatemp1,Datatemp0,'uint32')), ...
                'uint32' )),...
             'single');  

    end
end
%emgshort = emgData(1:15000);
PRESSData = reshape(PRESSData,8,[]); %this part is questionable, seems to work  
PRESSData = reshape(PRESSData,8,[])*scaler; % multiply by the scaler to put in volts
%{
insert conversion to lbs per volts, dont forget to subtract the inital bias
if not done in 
%}
p1=PRESSData(1,:);
p2=PRESSData(2,:);
p3=PRESSData(3,:);
p4=PRESSData(4,:);
p5=PRESSData(5,:);
p6=PRESSData(6,:);
p7=PRESSData(7,:);
p8=PRESSData(8,:);
fs = 250; %set sampling frequency 

t = 0:1/fs:(length(PRESSData)-1)/fs;

%mkdir(savelocation)% make the individual directory for each test batch

%*** Scaling the plots to all be the same size in Y and X axis ****
maxPressure_scale=max([p1;p2;p3;p4;p5;p6;p7;p8]);
maxPressure_scale=max(maxPressure_scale(:,:,:,:,:,:,:,:));
minPressure_scale=min([p1;p2;p3;p4;p5;p6;p7;p8]);
minPressure_scale=min(minPressure_scale(:,:,:,:,:,:,:,:));


E=figure('Name', 'Pressure foot 1 Data');%,'visible','off')
plot(t, p1);
hold on
plot(t, p2);
plot(t, p3);
plot(t, p4);
axis([0,max(t),minPressure_scale,maxPressure_scale+1])
title('Pressure foot 1 Data')
xlabel('Time (s)')
ylabel('PRESSURE (Volts)')
hold off

E=figure('Name', 'Pressure foot 2 Data');%,'visible','off')
plot(t, p5);
hold on
plot(t, p6);
plot(t, p7);
plot(t, p8)
axis([0,max(t),minPressure_scale,maxPressure_scale+1])
title('Pressure foot 2 Data')
xlabel('Time (s)')
ylabel('PRESSURE (Volts)')
%saveas(E,[pathName,'\figures',num2str(k),'\','PRESS',ShortEndName],'fig')

end % ends if for empty file and increments to the next 

toc  %end


