function [botSim] = localise(botSim,map,target)
%This function returns botSim, and accepts, botSim, a map and a target.
%LOCALISE Template localisation function

%% setup code

%variables
scanSamples = 6;
numOfParticles = 300;
randomRespawnProportion = 0.2;
maxNumOfIterations = 1;

%you can modify the map to take account of your robots configuration space
modifiedMap = map; %you need to do this modification yourself
botSim.setMap(modifiedMap);

%select the number of scan samples taking for the robot and particles
botSim.setScanConfig(botSim.generateScanConfig(scanSamples));

%generate some random particles inside the map
particles(numOfParticles,1) = BotSim; %how to set up a vector of objects
for i = 1:numOfParticles
    particles(i) = BotSim(modifiedMap);  %each particle should use the same map as the botSim object
    particles(i).randomPose(0); %spawn the particles in random locations
    particles(i).setScanConfig(botSim.generateScanConfig(scanSamples));
end

%How many particles will be respawned at a random location after each
%iteration
numberOfRandomRespawns = randomRespawnProportion * numOfParticles;

%% Localisation code
n = 0;
converged =0; %The filter has not converged yet
while(converged == 0 && n < maxNumOfIterations) %%particle filter loop
    n = n+1; %increment the current number of iterations
    disp("Iteration: " + n);
    botScan = botSim.ultraScan()'; %get a scan from the real robot.
    %% Write code for updating your particles scans
    
    %array containing the scans for each particle
    particleScans = [];
    for i = 1:numOfParticles
        particleScans = [particleScans; particles(i).ultraScan()'];
    end
    
    %% Write code for scoring your particles    
    
    %The standard deviation of the normal distribution depends on the precision of the sensor
    sigma = 1;
    %the probabilities of each particleScan
    probabilities = [];
    %iterating through all of the particleScans
    for i = 1:numOfParticles
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
    
    %normalizing probabilities
    weights = [];
    probabilitiesSum = sum(probabilities);
    for i = 1:numOfParticles
        weight = probabilities(i) / probabilitiesSum;
        weights = [weights; weight];
    end

    %% Write code for resampling your particles
    
    %calculating the cumulative distribution of the weights
    cumulative = [weights(1)];
    for i=2:numOfParticles
        cumulative = [cumulative; cumulative(i-1) + weights(i)];
    end
    
    %the number of particles to be respawned around current particles
    particleResamples = zeros([numOfParticles,1]);
    for i=1:(numOfParticles-numberOfRandomRespawns)
        random = rand();
        for j=1:numOfParticles
            if random < cumulative(j)
                particleResamples(j) = particleResamples(j) + 1;
                break;
            end
        end
    end
    
    %respawn particles around the particles with the best weights
    particlesUsed = 0;
    for i = 1:numOfParticles
        if particleResamples(i) > 0
            %get the pos of the parcticles to resample around
            particlePos = particles(i).getBotPos();
            particlePos = particlePos(1,:);
            disp("Resampling " + particleResamples(i) + " particles around: ");
            disp(particlePos);
            %set the particles' new pose with some error (need to set ang
            %aswell at some point
            for j=(particlesUsed+1):(particlesUsed+particleResamples(i))
                particlesUsed = particlesUsed + 1;
                particles(j).setBotPos([particlePos(1) + randn, particlePos(2) + randn]);
            end
        end
    end
    
    %% Write code to check for convergence   
	

    %% Write code to take a percentage of your particles and respawn in randomised locations (important for robustness)
    
    %Respawn the remaining particles in randomised locations
    for i=(particlesUsed+1):numOfParticles
        particles(i).randomPose(0);
        particlesUsed = particlesUsed + 1;
    end
    
    disp("Resampling " + numberOfRandomRespawns + " particles at randomised locations");
    
    disp("Total particles resampled: " + particlesUsed);
    
    %% Write code to decide how to move next
    % here they just turn in cicles as an example
    turn = 0.5;
    move = 2;
    botSim.turn(turn); %turn the real robot.  
    botSim.move(move); %move the real robot. These movements are recorded for marking 
    for i =1:numOfParticles %for all the particles. 
        particles(i).turn(turn); %turn the particle in the same way as the real robot
        particles(i).move(move); %move the particle in the same way as the real robot
    end
    
    %% Drawing
    %only draw if you are in debug mode or it will be slow during marking
    if botSim.debug()
        hold off; %the drawMap() function will clear the drawing when hold is off
        botSim.drawMap(); %drawMap() turns hold back on again, so you can draw the bots
        botSim.drawBot(30,'g'); %draw robot with line length 30 and green
        for i =1:numOfParticles
            particles(i).drawBot(3); %draw particle with line length 3 and default color
        end
        drawnow;
    end
end
end
