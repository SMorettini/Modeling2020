
%addpath(genpath(path))

nTimes = 1; % Number of times in which a function is going to be solved
dimension = 5; % Dimension of the problem
%populationSize = 60; % Adjust this to your algorithm
%populationSize = 100;% for GA
%populationSize = 60;% for PSO and DE
populationSize = 60;%for modelling
%SIMONE
algorithmType=2; %1 is GA, 2 is DE, 3 is PSO
alphaCrossover=0.5;
probabilityOfMutation=0.5;

scalingFactor=0.7;
recombinationProbability=0.8;

Vmax=pi;
Wcognitivo=1;
Wsocial=2;

%SIMONE

tot=0;

for t = 1:nTimes

        %maxEval = 10000*dimension; % maximum number of evaluation
        maxEval = 800;%*dimension; % maximum number of evaluation
        
        lower=[5, 0.1, 0.1, 0.1, 0.1];
        upper=[30, 2, 2, 2, 5];

        currentEval = 0;

        % Start generating the initial population
        population = zeros(populationSize, dimension);
        for j =1:populationSize
                population(j,:) = lower + (upper-lower).*rand(1,dimension);
        end

        populationFitness = calculateFitness_FUN_offspring(population,false); %Fitness values of all individuals (smaller value is better)
        bestSolutionFitness = min(populationFitness);
        currentEval = currentEval + populationSize;
        
        
        %%%%!!!!! PSO INTIALIZATION !!!!!%%%%%
        pBestPosition=population;
        Velocity=-Vmax +(Vmax -(-Vmax)).* rand(populationSize,dimension);
        populationBestFitness=populationFitness;
        neighbour=zeros(populationSize, 3);
        for k=1:populationSize
            %  neighbour(k,1)=k-1;
            %  neighbour(k,2)=k+1;
            %  neighbour(k,3)=k+2;
            neighbour(k,1)=floor(k/5)+1;
            neighbour(k,2)=floor(k/5)+2;
            neighbour(k,3)=floor(k/5)+3;
            neighbour(k,4)=floor(k/5)+4;
            neighbour(k,5)=floor(k/5)+5;
        end
        neighbour(1,1)=populationSize;
        neighbour(populationSize-1,3)=1;
        neighbour(populationSize,2)=1;
        neighbour(populationSize,3)=2;
        %%%%!!!!! END PSO INTIALIZATION !!!!!%%%%%
        
        % Algorithm loop

        while(currentEval < maxEval)

            if(algorithmType==1)
                %GA
                %ASSIGN PROBABILITY
                [populationFitness, indices] = sort(populationFitness);%for matrix sortrows
                population=population(indices,:);
                %x=input('waiting');
                probabilityOfSelection=1./populationFitness;
                total=sum(probabilityOfSelection);
                probabilityOfSelection=probabilityOfSelection/total;
                accumulator=0;
                for p=1:populationSize
                    accumulator=accumulator+probabilityOfSelection(p);
                    probabilityOfSelection(p)=accumulator;
                end
                
                %CROSSOVER. LINEAR ORDER
                tmpPopulation=population;
                offsprings=zeros(populationSize, dimension);
                for p = 1:populationSize/2
                    selected=0;
                    randValue=rand;
                    j=2;
                    while(selected==0)
                        sizeOdprobs=size(probabilityOfSelection);
                        if(randValue<probabilityOfSelection(j) || j==sizeOdprobs(2))
                            %DO CROSSOVER
                            Cmin=min(tmpPopulation(1,:), tmpPopulation(j,:));
                            Cmax=max(tmpPopulation(1,:), tmpPopulation(j,:));
                            I=Cmax-Cmin;
                            Cmin=Cmin-I*alphaCrossover;
                            Cmax=Cmax+I*alphaCrossover;
                            Cmin=times(Cmin>lower,Cmin)+times(Cmin<lower,lower);
                            Cmax=times(Cmax<upper,Cmax)+times(Cmax>upper,upper);
                            offsprings((p-1)*2+1,:)=Cmin + (Cmax-Cmin).*rand(1,dimension);
                            offsprings((p-1)*2+2,:)=Cmin + (Cmax-Cmin).*rand(1,dimension);
                            selected=1;
                            tmpPopulation(j,:)=[];
                            probabilityOfSelection(j)=[];
                            tmpPopulation(1,:)=[];
                            probabilityOfSelection(1)=[];
                            
                        end
                        j=j+1;
                    end
                end
                %MUTATION
                mutations=normrnd(0,0.5,populationSize, dimension);
                probPerInd=rand(1,populationSize)<probabilityOfMutation;
                offspringsMutated=offsprings+diag(probPerInd)*mutations;
                %offspringsMutated=offsprings;
                
                %evaluate fitness of offsprings
                offspringsMutatedFitness = calculateFitness_FUN_offspring(offspringsMutated,false); %Fitness values of all individuals (smaller value is better)
                bestFitnessNow = min(offspringsMutatedFitness);
                currentEval = currentEval + populationSize;
                %evaluate fitness of offsprings END
                
                %REPLACEMENT
                [offspringsMutatedFitness, indices] = sort(offspringsMutatedFitness);
                offspringsMutated=offspringsMutated(indices,:);
                [populationFitness, indices] = sort(populationFitness);%for matrix sortrows
                population=population(indices,:);
                population(40:90,:)=offspringsMutated(1:51,:);%ATTENZIONE PRENDE SOLO I MIGLRIORI NON VA PROPRIO BENE
                populationFitness(:,40:90)=offspringsMutatedFitness(:,1:51);
                population(90:100,:)=offspringsMutated(81:91,:);%ATTENZIONE PRENDE SOLO I MIGLRIORI NON VA PROPRIO BENE
                populationFitness(:,90:100)=offspringsMutatedFitness(:,81:91);
                if(bestFitnessNow<bestSolutionFitness)
                    bestSolutionFitness=bestFitnessNow;
                    %fprintf('%d \n',bestSolutionFitness);
                end
                
            elseif(algorithmType==2)
                %DE
                %MUTATION
                mutatedPopulation=population;
                for m=1:populationSize
                    different=0;
                    while different==0
                    r = randi([1 populationSize],1,3);
                        %different=1;
                    if(~((r(1)==r(2))||(r(1)==r(3))||(r(2)==r(3))))
                        different=1;
                    end
                    if(~(r(2)==r(3)))
                        different=1;
                    end
                    end
                %  r
                    %   input('aaa');
                    mutatedPopulation(m,:)= population(r(1),:)+scalingFactor*(population(r(2),:)-population(r(3),:));
                end
                %mutatedPopulation=times(mutatedPopulation>lower,mutatedPopulation)+times(mutatedPopulation<lower,lower);
                %mutatedPopulation=times(mutatedPopulation<upper,mutatedPopulation)+times(mutatedPopulation>upper,upper);
                %mutatedPopulation(mutatedPopulation<lower)=lower; ONLY FOR ONE SINGLE LOWER
                %mutatedPopulation(mutatedPopulation>upper)=upper; ONLY FOR ONE SINGLE UPPER
                mutatedPopulation=mutatedPopulation.*(mutatedPopulation>lower)+lower.*(mutatedPopulation<=lower);
                mutatedPopulation=mutatedPopulation.*(mutatedPopulation<upper)+upper.*(mutatedPopulation>=upper);
                
                %RECOMBINATION
                fromOff=rand(populationSize,dimension)<recombinationProbability;
                %fromOff
                %input('a');
                %  for row=1:populationSize
                %       fromOff(row,randi([1,dimension]))=1;
                %  end
                offsprings=population.*~fromOff+mutatedPopulation.*fromOff;
                
                %UPDATE POPULATION
                offspringsFitness = calculateFitness_FUN_offspring(offsprings,false); %Fitness values of all individuals (smaller value is better)
                bestSolutionFitnessNow = min(offspringsFitness);
                currentEval = currentEval + populationSize;            
                better=offspringsFitness<populationFitness;
                population=diag(~better)*population+diag(better)*offsprings;        
                populationFitness=populationFitness.*~better+offspringsFitness.*better;
                
                if(bestSolutionFitnessNow<bestSolutionFitness)
                    bestSolutionFitness=bestSolutionFitnessNow;
                %fprintf('%d \n',bestSolutionFitness);
                end
            
            else
                
                populationFitness = calculateFitness_FUN_offspring(population,false); %Fitness values of all individuals (smaller value is better)
                bestSolutionFitnessNow = min(populationFitness);
                currentEval = currentEval + populationSize;
                
                populationBestFitness(populationFitness<populationBestFitness)=populationFitness(populationFitness<populationBestFitness);
                pBestPosition(populationFitness<populationBestFitness,:)=population(populationFitness<populationBestFitness,:);
                
                bestNeighbour=zeros(populationSize, dimension);
                for q=1:populationSize
                    %  if(populationBestFitness(neighbour(q,1))<populationBestFitness(neighbour(q,2)))
                    %      bestNeighbour(q,:)=population(neighbour(q,1),:);
                    %   else
                    %       bestNeighbour(q,:)=population(neighbour(q,2),:);
                    %   end
                    minn=min([populationBestFitness(neighbour(q,1)),populationBestFitness(neighbour(q,2)),populationBestFitness(neighbour(q,3)),populationBestFitness(neighbour(q,4)),populationBestFitness(neighbour(q,5))]);
                    bestNeighbour(q,:)=0;
                    bestNeighbour(q,:)=(populationBestFitness(neighbour(q,1))==minn)*population(neighbour(q,1),:);
                    bestNeighbour(q,:)=(populationBestFitness(neighbour(q,2))==minn)*population(neighbour(q,2),:);
                    bestNeighbour(q,:)=(populationBestFitness(neighbour(q,3))==minn)*population(neighbour(q,3),:);
                    bestNeighbour(q,:)=(populationBestFitness(neighbour(q,4))==minn)*population(neighbour(q,4),:);
                    bestNeighbour(q,:)=(populationBestFitness(neighbour(q,5))==minn)*population(neighbour(q,5),:);
                end
                %rand1=diag(rand(populationSize,1));
                %rand2=diag(rand(populationSize,1));
                rand1=rand(populationSize,dimension);
                rand2=rand(populationSize,dimension);
                Velocity=Velocity+Wcognitivo*rand1.*(pBestPosition-population)+Wsocial*rand2.*(bestNeighbour-population);
                Velocity(Velocity>Vmax)=Vmax;
                Velocity(Velocity<-Vmax)=-Vmax;
                % input('aa');
                
                
                
                
                
                
                
                
                %PSO
                population=population+Velocity;
                %population(population>upper)=upper; ONLY FOR ONE SINGLE LOWER
                %population(population<lower)=lower; ONLY FOR ONE SINGLE UPPER
                population=population.*(population>lower)+lower.*(population<=lower);
                population=population.*(population<upper)+upper.*(population>=upper);
                
                
            
                if(bestSolutionFitnessNow<bestSolutionFitness)
                    bestSolutionFitness=bestSolutionFitnessNow;
                % fprintf('%d \n',bestSolutionFitness);
                end
            end           

            [fits, inds] = sort(populationFitness);%for matrix sortrows
            pop_ordered=population(inds,:);

            fprintf('%dth eval, The best individual fitness is %d  %d \n', currentEval, bestSolutionFitness,fits(1));
            fprintf(' %2d ', pop_ordered(1,:));
            fprintf('\n');
calculateFitness_FUN_offspring( pop_ordered(1,:),true);

        end

        % best individual
        % bestSolutionFitness = min(populationFitness);
        fprintf('%dth run, The best individual fitness is %d  \n', t, bestSolutionFitness);
        
        tot=tot+bestSolutionFitness;
end


 