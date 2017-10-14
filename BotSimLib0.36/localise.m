function [botSim] = localise(botSim,map,target)
%This function returns botSim, and accepts, botSim, a map and a target.
%LOCALISE Template localisation function

%% setup code
%you can modify the map to take account of your robots configuration space
modifiedMap = map; %you need to do this modification yourself
botSim.setMap(modifiedMap);

%select the number of scan samples taking for the robot and particles
scanSamples = 20;
botSim.setScanConfig(botSim.generateScanConfig(scanSamples));

%generate some random particles inside the map
num =300; % number of particles
particles(num,1) = BotSim; %how to set up a vector of objects
for i = 1:num
    particles(i) = BotSim(modifiedMap);  %each particle should use the same map as the botSim object
    particles(i).randomPose(0); %spawn the particles in random locations
    particles(i).setScanConfig(botSim.generateScanConfig(scanSamples));
end

%% Localisation code
maxNumOfIterations = 1;
n = 0;
converged =0; %The filter has not converged yet
while(converged == 0 && n < maxNumOfIterations) %%particle filter loop
    n = n+1; %increment the current number of iterations
    botScan = botSim.ultraScan()'; %get a scan from the real robot.
    %% Write code for updating your particles scans
    
    %array containing the scans for each particle
    particleScans = [];
    for i = 1:num
        particleScans = [particleScans; particles(i).ultraScan()'];
    end
    
    %% Write code for scoring your particles    
    
    %The standard deviation depends on the precision of the sensor
    sigma = 1;
    %the probabilities of each particleScan
    probabilities = [];
    %iterating through all of the particleScans
    for i = 1:num
        highest = -1;
        particleScan = particleScans(i,:);
        samples = length(particleScan);
        %iterate through each cyclic shift of the scan
        for j = 0:samples-1
            shifted = circshift(particleScan,j);
            probs = normpdf(shifted, botScan, sigma);
            prob = prod(probs);
            %get the highest probability
            if prob > highest
                highest = prob;
            end
        end
        probabilities = [probabilities; highest];
    end
    [val, idx] = max(probabilities);
    disp("highest probability: " + val);
    disp("index: " + idx);
    disp("bot scan: ");
    disp(botScan);
    disp("best particle scan: ");
    disp(particleScans(idx,:));
    
    %% Write code for resampling your particles
    
    
    %% Write code to check for convergence   
	

    %% Write code to take a percentage of your particles and respawn in randomised locations (important for robustness)	
    
    
    %% Write code to decide how to move next
    % here they just turn in cicles as an example
    turn = 0.5;
    move = 2;
    botSim.turn(turn); %turn the real robot.  
    botSim.move(move); %move the real robot. These movements are recorded for marking 
    for i =1:num %for all the particles. 
        particles(i).turn(turn); %turn the particle in the same way as the real robot
        particles(i).move(move); %move the particle in the same way as the real robot
    end
    
    %% Drawing
    %only draw if you are in debug mode or it will be slow during marking
    if botSim.debug()
        hold off; %the drawMap() function will clear the drawing when hold is off
        botSim.drawMap(); %drawMap() turns hold back on again, so you can draw the bots
        botSim.drawBot(30,'g'); %draw robot with line length 30 and green
        for i =1:num
            particles(i).drawBot(3); %draw particle with line length 3 and default color
        end
        particles(idx).drawBot(30,'b');
        drawnow;
    end
end
end
