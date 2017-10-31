function [botSim] = localise(botSim,map,target)
%This function returns botSim, and accepts, botSim, a map and a target.
%LOCALISE Template localisation function

if botSim.debug()
    start = botSim.getBotPos();
    fprintf("start: (%f,%f)\n", start(1), start(2));
    fprintf("target: (%f,%f)\n", target(1), target(2));
end

%% setup code

%variables
scanSamples = 12;
numOfParticles = 300;
randomRespawnProportion = 0.5;
maxNumOfIterations = 300;
sensorSigma = 1;
sensorNoise = 1;
motionNoise = 0.001;
turnNoise = 0.0005;

%dimensions in the discretized grid
xnum = 20;
ynum = 20;

%you can modify the map to take account of your robots configuration space
modifiedMap = map; %you need to do this modification yourself
botSim.setMap(modifiedMap);

%select the number of scan samples taking for the robot and particles
botSim.setScanConfig(botSim.generateScanConfig(scanSamples));

%generate some random particles inside the map
particles(numOfParticles,1) = BotSim; %how to set up a vector of objects

%get discretized version of the map
discreteMap = DiscreteMap(botSim, xnum, ynum);
targetNode = discreteMap.findClosestNode(target);

%initialise particles
for i = 1:numOfParticles
    particles(i) = BotSim(modifiedMap);  %each particle should use the same map as the botSim object
    particles(i).randomPose(0); %spawn the particles in random locations
    particles(i).setScanConfig(botSim.generateScanConfig(scanSamples));
    particles(i).setMotionNoise(motionNoise);
    particles(i).setTurningNoise(turnNoise);
    particles(i).setSensorNoise(sensorNoise);
end

%How many particles will be respawned at a random location after each
%iteration
numberOfRandomRespawns = randomRespawnProportion * numOfParticles;

%Prediction of the pose of the robot
posPrediction = [];
angPrediction = 0;

%path planning info
currentPath = [];
nodePrediction = [0,0];

pathPlans = 0;

%% Localisation code
n = 0;
converged =0; %The filter has not converged yet
while(converged == 0 && n < maxNumOfIterations) %%particle filter loop
    n = n+1; %increment the current number of iterations
    
    %% Write code to decide how to move next
    if n <= 5
        % here they just turn in circles as an example
        turn = 0.5;
        move = 2;
        botSim.turn(turn); %turn the real robot.  
        botSim.move(move); %move the real robot. These movements are recorded for marking 
        for i =1:numOfParticles %for all the particles. 
            particles(i).turn(turn); %turn the particle in the same way as the real robot
            particles(i).move(move); %move the particle in the same way as the real robot
        end
    else
          
        %% Write code to check for convergence
        %The node that we are at according to our position prediction
        currentNode = discreteMap.findClosestNode(posPrediction);
        
        %print node values
        %{
        if botSim.debug()
            fprintf("currentNode: (%d,%d)\n", currentNode(1), currentNode(2));
            fprintf("predictedNode: (%d,%d)\n", nodePrediction(1), nodePrediction(2));
            fprintf("targetNode: (%d,%d)\n", targetNode(1), targetNode(2));
        end
        %}
        
        %If you are located in the final node then move to the final target position
        if currentNode(1) == targetNode(1) && currentNode(2) == targetNode(2)
            [newPos, newAng] = moveToPos(botSim, particles, posPrediction, angPrediction, target);
            posPrediction = newPos;
            angPrediction = newAng;
            converged = 1;
        end
        
        %Check if we should plan a new path
        if ~(currentNode(1) == nodePrediction(1)) || ~(currentNode(2) == nodePrediction(2))
          path = findPath(botSim, discreteMap, posPrediction, target);
          pathPlans = pathPlans + 1;
          nodePrediction = currentNode;
          if botSim.debug()
              fprintf("CALCULATED NEW PATH!!\n");
          end
        end
        
        pathDim = size(path);
        if pathDim(1) > 0
        
            %Getting information about the next position to move to
            nextNodeInfo = path(1,:);
            nextNode = [nextNodeInfo(1), nextNodeInfo(2)];
            nextPos = discreteMap.nodes(nextNode(1),nextNode(2)).pos;
        
            %moving to the next position
            nodePrediction = nextNode;
            [newPos, newAng] = moveToPos(botSim, particles, posPrediction, angPrediction, nextPos);
        
            %updating info
            path(1,:) = [];
            posPrediction = newPos;
            angPrediction = newAng;
        end
    end
    %% Write code for updating your particles scans
    botScan = botSim.ultraScan()'; %get a scan from the real robot.
    
    %array containing the scans for each particle
    particleScans = zeros(numOfParticles, scanSamples);
    for i = 1:numOfParticles
        particleScans(i,:) = particles(i).ultraScan()';
    end
    
    %% Write code for scoring your particles    
    
    %the probabilities of each particleScan
    probabilities = zeros(numOfParticles,1);
    %iterating through all of the particleScans
    for i = 1:numOfParticles
        highest = -1;
        bestShift = 0;
        particleScan = particleScans(i,:);
        samples = length(particleScan);
        %iterate through each cyclic shift of the scan
        for j = 0:samples-1
            shifted = circshift(particleScan,j);
            probs = normpdf(shifted, botScan, sensorSigma);
            prob = prod(probs);
            %get the highest probability
            if prob > highest
                highest = prob;
                bestShift = j;
            end
        end
        probabilities(i) = highest;
        %rotate the particles so they face the best direction
        newAng = particles(i).getBotAng() - bestShift * (2 * pi) / scanSamples;
        newAng = mod(newAng, 2 * pi);
        particles(i).setBotAng(newAng);
    end
    
    %normalizing probabilities
    weights = zeros(numOfParticles,1);
    probabilitiesSum = sum(probabilities);
    for i = 1:numOfParticles
        weight = probabilities(i) / probabilitiesSum;
        weights(i) = weight;
    end

    %value and index of the best weighted particle
    [val,idx] = max(weights);
    posPrediction = particles(idx).getBotPos();
    angPrediction = mod(particles(idx).getBotAng(),2*pi);
    
    
    %% Drawing
    %only draw if you are in debug mode or it will be slow during marking
    if botSim.debug()
        
        fprintf("Iteration %d\n", n);
        
        %getting the actual and predicted bot pose
        actualPos = botSim.getBotPos(0);
        actualAng = mod(botSim.getBotAng(0),2*pi);
        xError = abs(posPrediction(1) - actualPos(1));
        yError = abs(posPrediction(2) - actualPos(2));
        angError = abs(angPrediction - actualAng);
        
        %print actual and predicted position
        fprintf("Actual position:\t(%.3f, %.3f)\n", actualPos(1), actualPos(2));
        fprintf("Predicted position:\t(%.3f, %.3f)\n", posPrediction(1), posPrediction(2));
        fprintf("Position error: \t(%.3f, %.3f)\n", xError, yError);
        fprintf("\n");
        
        %print actual and predicted angle
        fprintf("Actual angle:\t\t%.3f\n", actualAng);
        fprintf("Predicted angle:\t%.3f\n", angPrediction);
        fprintf("Angle error:\t\t%.3f\n", angError);
        fprintf("\n\n");
        
        hold off; %the drawMap() function will clear the drawing when hold is off
        botSim.drawMap(); %drawMap() turns hold back on again, so you can draw the bots
        botSim.drawBot(30,'g'); %draw robot with line length 30 and green
        for i =1:numOfParticles
            particles(i).drawBot(3); %draw particle with line length 3 and default color
        end
        particles(idx).drawBot(30,'r');
        plot(target(1),target(2),"*");
        drawnow;
    end
    
    %% check for convergence
    if converged == 1
        break;
    end

    %% Write code for resampling your particles
    
    %calculating the cumulative distribution of the weights
    cumulative = zeros(numOfParticles,1);
    cumulative(1) = weights(1);
    for i=2:numOfParticles
        cumulative(i) = cumulative(i-1) + weights(i);
    end
    
    %the number of particles to be respawned around current particles
    particleResamples = zeros(numOfParticles,1);
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
            particleAng = particles(i).getBotAng();
            %set the particles' new pose with some error (need to set ang aswell at some point
            for j=(particlesUsed+1):(particlesUsed+particleResamples(i))
                particlesUsed = particlesUsed + 1;
                particles(j).setBotPos([particlePos(1) + randn, particlePos(2) + randn]);
            end
        end
    end
    
    %% Write code to take a percentage of your particles and respawn in randomised locations (important for robustness)
    
    %Respawn the remaining particles in randomised locations
    for i=(particlesUsed+1):numOfParticles
        particles(i).randomPose(0);
        particlesUsed = particlesUsed + 1;
    end
   
end
fprintf("Number of paths planned: %d\n", pathPlans);
end



%% helpler functions

function [newPos,newAngle] = moveToPos(botSim, particles, startPos, startAng, targetPos)
    newAngle = calculateAngle(startPos,targetPos);
    turnAngle = newAngle - startAng;
    botSim.turn(turnAngle);
    dist = pdist2(startPos, targetPos);
    botSim.move(dist);
    newPos = targetPos;
    pdim = size(particles);
    for i=1:pdim(1)
        particles(i).turn(turnAngle);
        particles(i).move(dist);
    end
end

function finalPos = followPath(botSim, discreteMap, startPos, startAng, path, targetPos)
    start = startPos;
    angle = startAng;
    newAngle = 0;
    dims = size(path);
    for i=1:dims(1)
        targetInfo = path(i,:);
        targetNode = [targetInfo(1),targetInfo(2)];
        %get the next target from the path
        target = discreteMap.nodes(targetNode(1), targetNode(2)).pos;
        % turning the bot towards the target
        newAngle = calculateAngle(start, target);
        turnAngle = newAngle - angle;
        botSim.turn(turnAngle);
        angle = newAngle;
        % moving the bot to the target
        dist = pdist2(start, target);
        botSim.move(dist);
        %the target is the new start node in the next iteration
        start = target;
        if botSim.debug()
            botSim.drawBot(30,'g'); %draw robot with line length 30 and green
        end
    end
    finalPos = start;
end

function angle = calculateAngle(start, target)
    diff = target - start;
    alpha = atan(diff(2)/diff(1));
    if diff(1) > 0
        angle = mod(alpha,2*pi);
    elseif diff(1) < 0
        angle = pi + alpha;
    else
        if diff(2) > 0
            angle = pi/2;
        else
            angle = 3*pi/2;
        end
    end
end