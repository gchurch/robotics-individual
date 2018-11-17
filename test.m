clf;        %clears figures
clc;        %clears console
clear;      %clears workspace
axis equal; %keeps the x and y scale the same
map = [0,0;60,0;60,50;100,50;70,0;110,0;150,80;30,80;30,40;0,80];  %long map

% botSim = BotSim(map,[0.01,0.005,0]);  %sets up a botSim object a map, and debug mode on.
botSim = BotSim(map,[0,0,0]);  %sets up a botSim object a map, and debug mode on.
botSim.drawMap();
drawnow;
botSim.randomPose(10); %puts the robot in a random position at least 10cm away from a wall
target = botSim.getRndPtInMap(10);  %gets random target.

tic %starts timer

%your localisation function is called here.
returnedBot = localise(botSim,map,target); %Where the magic happens

resultsTime = toc %stops timer

%calculated how far away your robot is from the target.
resultsDis =  distance(target, returnedBot.getBotPos())