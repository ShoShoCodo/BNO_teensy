% %By: Shoa Russell 
% %analog parser
% % This program parses log files and plots the sensor data.
% 
 tic;
% clc;
 clear;
 close all;

 numsamples_P = 50;
%Scalers to make the values mean something 
Vref= 2.5;
scaler=Vref/4095;
scaler2pounds=0;


 PRESS_BLOCKSIZE = 8*numsamples_P;
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
PRESSData = zeros(1,5); %PRESS_BLOCKSIZE*(numBlocks-BLOCKSIZE));
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
p11=PRESSData(1,:);
p12=PRESSData(2,:);
p13=PRESSData(3,:);
p14=PRESSData(4,:);
p21=PRESSData(5,:);
p22=PRESSData(6,:);
p23=PRESSData(7,:);
p24=PRESSData(8,:);
fsP = 20; %set sampling frequency of the pressure sensor 

tP = 0:1:(length(PRESSData)-1);
%tP=0:p11;
%mkdir(savelocation)% make the individual directory for each test batch


 % used to find the bias, this number will be subtracted in the main 
 % software since it should be constant for each sensor but may be
 % different depending on the weather 
p11bias = min(p11) 
p12bias = min(p12)
p13bias = min(p13)
p14bias = min(p14)

p21bias = min(p21)
p22bias = min(p22)
p23bias = min(p23)
p24bias = min(p24)


%*** Scaling the plots to all be the same size in Y and X axis ****
maxPressure_scale=max([p11;p12;p13;p14;p21;p22;p23;p24]);
maxPressure_scale=max(maxPressure_scale(:,:,:,:,:,:,:,:));
minPressure_scale=min([p11;p12;p13;p14;p21;p22;p23;p24]);
minPressure_scale=min(minPressure_scale(:,:,:,:,:,:,:,:));


E=figure('Name', 'Pressure foot 1 Data');%,'visible','off')
plot(tP, p11);
hold on
plot(tP, p12);
plot(tP, p13);
plot(tP, p14);
axis([0,max(tP),minPressure_scale,maxPressure_scale+1])
title('Pressure foot 1 Data')
xlabel('Time (s)')
ylabel('PRESSURE (Volts)')
hold off

E=figure('Name', 'Pressure foot 2 Data');%,'visible','off')
plot(tP, p21);
hold on
plot(tP, p22);
plot(tP, p23);
plot(tP, p24)
axis([0,max(tP),minPressure_scale,maxPressure_scale+1])
title('Pressure foot 2 Data')
xlabel('Time (s)')
ylabel('PRESSURE (Volts)')
%saveas(E,[pathName,'\figures',num2str(k),'\','PRESS',ShortEndName],'fig')

end % ends if for empty file and increments to the next 

toc  %end


