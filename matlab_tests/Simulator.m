function Simulator
    clear
    clc
    
    % HAND MODEL __________________________________________________________
    handModel = HandModel;
    hand_defaults = handModel.getDefaults();
    observedModel.depth = handModel.renderHand([0;0;0;1],hand_defaults);
    
    [height,width,~] = size(observedModel.depth);
    observedModel.segm = zeros(height,width,1);
    for i = 1:height
        for j = 1:width
            if (observedModel.depth(i,j) ~= 0)
                observedModel.segm(i,j) = 1;
            end
        end
    end
    % _____________________________________________________________

    % Parameters __________________________________________________________
    global generation_amount;
    global particle_amount;
    global dimension_amount;
    global handJoint;
    
    global c1;
    global c2;
    global w;
    
    global parameter_space;
    global bestSoFar;
    global bestT;
    global particles;
    
    generation_amount = 100;
    particle_amount = 25;
    dimension_amount = 23;
    bestSoFar = 1000;
    bestT = zeros(1,dimension_amount);
    c1 = 2.8;
    c2 = 1.3;
    w = 2/abs(2-(c1+c2)-sqrt(((c1+c2).^2)-(4*(c1+c2))));
    % _____________________________________________________________________

    % Setup dimension names
    handJoint.wrist_1_a = 1;
    handJoint.wrist_1_b = 2;
    handJoint.wrist_1_c = 3;

    handJoint.thumb_1_a = 4;
    handJoint.thumb_1_b = 5;
    handJoint.thumb_2_a = 6;
    handJoint.thumb_3_a = 7;

    handJoint.index_1_a = 8;
    handJoint.index_1_b = 9;
    handJoint.index_2_a = 10;
    handJoint.index_3_a = 11;

    handJoint.middle_1_a = 12;
    handJoint.middle_1_b = 13;
    handJoint.middle_2_a = 14;
    handJoint.middle_3_a = 15;

    handJoint.third_1_a = 16;
    handJoint.third_1_b = 17;
    handJoint.third_2_a = 18;
    handJoint.third_3_a = 19;

    handJoint.little_1_a = 20;
    handJoint.little_1_b = 21;
    handJoint.little_2_a = 22;
    handJoint.little_3_a = 23;
    
    % Setup multi-dimensional parameter space limits
    parameter_space(handJoint.wrist_1_a).minimum = -hand_defaults.max_size;
    parameter_space(handJoint.wrist_1_a).maximum = hand_defaults.max_size;
    parameter_space(handJoint.wrist_1_b).minimum = -hand_defaults.max_size;
    parameter_space(handJoint.wrist_1_b).maximum = hand_defaults.max_size;
    parameter_space(handJoint.wrist_1_c).minimum = -hand_defaults.max_size;
    parameter_space(handJoint.wrist_1_c).maximum = hand_defaults.max_size;
    
    parameter_space(handJoint.thumb_1_a).minimum = 0;
    parameter_space(handJoint.thumb_1_a).maximum = hand_defaults.thumb(1).d;
    parameter_space(handJoint.thumb_1_b).minimum = -hand_defaults.thumb(1).d/2;
    parameter_space(handJoint.thumb_1_b).maximum = hand_defaults.thumb(1).d/2;
    parameter_space(handJoint.thumb_2_a).minimum = 0;
    parameter_space(handJoint.thumb_2_a).maximum = hand_defaults.thumb(2).d;
    parameter_space(handJoint.thumb_3_a).minimum = 0;
    parameter_space(handJoint.thumb_3_a).maximum = hand_defaults.thumb(3).d;

    parameter_space(handJoint.index_1_a).minimum = 0;
    parameter_space(handJoint.index_1_a).maximum = hand_defaults.index(1).d;
    parameter_space(handJoint.index_1_b).minimum = -hand_defaults.index(1).d/2;
    parameter_space(handJoint.index_1_b).maximum = hand_defaults.index(1).d/2;
    parameter_space(handJoint.index_2_a).minimum = 0;
    parameter_space(handJoint.index_2_a).maximum = hand_defaults.index(2).d;
    parameter_space(handJoint.index_3_a).minimum = 0;
    parameter_space(handJoint.index_3_a).maximum = hand_defaults.index(3).d;

    parameter_space(handJoint.middle_1_a).minimum = 0;
    parameter_space(handJoint.middle_1_a).maximum = hand_defaults.middle(1).d;
    parameter_space(handJoint.middle_1_b).minimum = -hand_defaults.middle(1).d/2;
    parameter_space(handJoint.middle_1_b).maximum = hand_defaults.middle(1).d/2;
    parameter_space(handJoint.middle_2_a).minimum = 0;
    parameter_space(handJoint.middle_2_a).maximum = hand_defaults.middle(2).d;
    parameter_space(handJoint.middle_3_a).minimum = 0;
    parameter_space(handJoint.middle_3_a).maximum = hand_defaults.middle(3).d;

    parameter_space(handJoint.third_1_a).minimum = 0;
    parameter_space(handJoint.third_1_a).maximum = hand_defaults.third(1).d;
    parameter_space(handJoint.third_1_b).minimum = -hand_defaults.third(1).d/2;
    parameter_space(handJoint.third_1_b).maximum = hand_defaults.third(1).d/2;
    parameter_space(handJoint.third_2_a).minimum = 0;
    parameter_space(handJoint.third_2_a).maximum = hand_defaults.third(2).d;
    parameter_space(handJoint.third_3_a).minimum = 0;
    parameter_space(handJoint.third_3_a).maximum = hand_defaults.third(3).d;

    parameter_space(handJoint.little_1_a).minimum = 0;
    parameter_space(handJoint.little_1_a).maximum = hand_defaults.little(1).d;
    parameter_space(handJoint.little_1_b).minimum = -hand_defaults.little(1).d/2;
    parameter_space(handJoint.little_1_b).maximum = hand_defaults.little(1).d/2;
    parameter_space(handJoint.little_2_a).minimum = 0;
    parameter_space(handJoint.little_2_a).maximum = hand_defaults.little(2).d;
    parameter_space(handJoint.little_3_a).minimum = 0;
    parameter_space(handJoint.little_3_a).maximum = hand_defaults.little(3).d;
    
%    if (rem(particle_amount,dimension_amount)==0)
        % Initial best population is random
        for d = 1:dimension_amount
            bestT(d) = (rand*(parameter_space(d).maximum-parameter_space(d).minimum)) + parameter_space(d).minimum;
        end
        
        % Time should be continuous, but for now 30 should be enough
        maxT = 30;
tic
%        for t = 1:maxT
            % Generate Initial Population _________________________________
            % First member of initial population in frame t is the best
            % alternative obtained in frame t-1
            for d = 1:dimension_amount
                particles(1).nextPosition(d) = bestT(d);
                particles(1).bestPosition(d) = particles(1).nextPosition(d);
                particles(1).velocity(d) = 0;
                % while the rest of the particles are merely perturbations 
                % of the first member;
                for p = 2:particle_amount
                    % Since a number between [0..1] is being multiplied to
                    % an already scaled value, its result will never
                    % overflow the parameter space for the dimension;
                    particles(p).nextPosition(d) = particles(1).nextPosition(d)*rand;
                    particles(p).bestPosition(d) = particles(1).nextPosition(d);
                    particles(p).velocity(d) = 0;
                end
            end
            
            for p = 1:particle_amount
                particles(p).bestSoFar = 1000;
            end
            % _____________________________________________________________
            
            % Iterate through Generations _________________________________
            for k = 1:generation_amount
                for p = 1:particle_amount
                    for d = 1:dimension_amount
                        particles(p).currentPosition(d) = particles(p).nextPosition(d);
                    end
                    
                    fitness = evaluate(p,observedModel,handModel);
                    
                    if fitness < particles(p).bestSoFar
                        particles(p).bestSoFar = fitness;
                        for d = 1:dimension_amount
                            particles(p).bestPosition(d) = particles(p).currentPosition(d);
                        end
                        
                        if (particles(p).bestSoFar < bestSoFar)
                            bestSoFar = particles(p).bestSoFar;
                            bestT(:) = particles(p).bestPosition(:);
                        end
                    end
                end
                
%                index = 1;
                for p = 1:particle_amount
                    for d = 1:dimension_amount
                        m1 = particles(p).bestPosition(d)-particles(p).currentPosition(d);
                        m2 = bestT(d)-particles(p).currentPosition(d);
                        particles(p).velocity(d) = w * (particles(p).velocity(d) + (c1*rand*(m1) ) + (c2*rand*(m2)));
                        particles(p).nextPosition(d) = particles(p).currentPosition(d) + particles(p).velocity(d);
                        if particles(p).nextPosition(d) < parameter_space(d).minimum
                            particles(p).nextPosition(d) = parameter_space(d).minimum;
                        elseif particles(p).nextPosition(d) > parameter_space(d).maximum
                            particles(p).nextPosition(d) = parameter_space(d).maximum;
                        end
                    end
 %                   x(index) = particles(p).bestPosition(1);
 %                   index = index + 1;
 %                                   plot(x, 'x');
 %                   pause(0.0001)
                end
            end
            % _____________________________________________________________
        %end
    %end
    
toc
    fitness
    bestT(:)
end

function result = evaluate(hypothesis,handObservationObject,handModelObject)
    global particles;
    global handJoint;
    
    defaults = handModelObject.getDefaults();
    
    % 1. Calculate angles for transformation
    % _ Wrist _
    rel_cartesian_wrist.x = particles(hypothesis).currentPosition(handJoint.wrist_1_a);
    rel_cartesian_wrist.y = particles(hypothesis).currentPosition(handJoint.wrist_1_b);
    rel_cartesian_wrist.z = particles(hypothesis).currentPosition(handJoint.wrist_1_c);
    data.pitch = radtodeg(atan2(rel_cartesian_wrist.z,defaults.max_size));       % Z
    data.roll = radtodeg(atan2(rel_cartesian_wrist.x,defaults.max_size));        % X
    data.yaw = radtodeg(atan2(rel_cartesian_wrist.y,defaults.max_size));         % Y
    
    % Thumb _ - | 2DOF | 1DOF | 1DOF |
    rel_cartesian_thumb_1.x = particles(hypothesis).currentPosition(handJoint.thumb_1_a);
    rel_cartesian_thumb_1.y = particles(hypothesis).currentPosition(handJoint.thumb_1_b);
    rel_cartesian_thumb_1.z = sqrt(defaults.thumb(1).d^2 - rel_cartesian_thumb_1.x^2 - rel_cartesian_thumb_1.y^2);
    data.thumb(1).d = rel_cartesian_thumb_1.z;
    data.thumb(1).flexion = radtodeg(atan2(rel_cartesian_thumb_1.x,defaults.thumb(1).d));
    data.thumb(1).abduction = radtodeg(atan2(rel_cartesian_thumb_1.y,defaults.thumb(1).d));
    rel_cartesian_thumb_2.x = particles(hypothesis).currentPosition(handJoint.thumb_2_a);
    rel_cartesian_thumb_2.z = sqrt(defaults.thumb(2).d^2 - rel_cartesian_thumb_2.x^2);
    data.thumb(2).d = rel_cartesian_thumb_2.z;
    data.thumb(2).flexion = radtodeg(atan2(rel_cartesian_thumb_2.x,defaults.thumb(2).d));
    rel_cartesian_thumb_3.x = particles(hypothesis).currentPosition(handJoint.thumb_3_a);
    rel_cartesian_thumb_3.z = sqrt(defaults.thumb(3).d^2 - rel_cartesian_thumb_3.x^2);
    data.thumb(3).d = rel_cartesian_thumb_3.z;
    data.thumb(3).flexion = radtodeg(atan2(rel_cartesian_thumb_3.x,defaults.thumb(3).d));
    % Index _ - | 2DOF | 1DOF | 1DOF |
    rel_cartesian_index_1.x = particles(hypothesis).currentPosition(handJoint.index_1_a);
    rel_cartesian_index_1.y = particles(hypothesis).currentPosition(handJoint.index_1_b);
    rel_cartesian_index_1.z = sqrt(defaults.index(1).d^2 - rel_cartesian_index_1.x^2 - rel_cartesian_index_1.y^2);
    data.index(1).d = rel_cartesian_index_1.z;
    data.index(1).flexion = radtodeg(atan2(rel_cartesian_index_1.x,defaults.index(1).d));
    data.index(1).abduction = radtodeg(atan2(rel_cartesian_index_1.y,defaults.index(1).d));
    rel_cartesian_index_2.x = particles(hypothesis).currentPosition(handJoint.index_2_a);
    rel_cartesian_index_2.z = sqrt(defaults.index(2).d^2 - rel_cartesian_index_2.x^2);
    data.index(2).d = rel_cartesian_index_2.z;
    data.index(2).flexion = radtodeg(atan2(rel_cartesian_index_2.x,defaults.index(2).d));
    rel_cartesian_index_3.x = particles(hypothesis).currentPosition(handJoint.index_3_a);
    rel_cartesian_index_3.z = sqrt(defaults.index(3).d^2 - rel_cartesian_index_3.x^2);
    data.index(3).d = rel_cartesian_index_3.z;
    data.index(3).flexion = radtodeg(atan2(rel_cartesian_index_3.x,defaults.index(3).d));
    % Middle _ - | 2DOF | 1DOF | 1DOF |
    rel_cartesian_middle_1.x = particles(hypothesis).currentPosition(handJoint.middle_1_a);
    rel_cartesian_middle_1.y = particles(hypothesis).currentPosition(handJoint.middle_1_b);
    rel_cartesian_middle_1.z = sqrt(defaults.middle(1).d^2 - rel_cartesian_middle_1.x^2 - rel_cartesian_middle_1.y^2);
    data.middle(1).d = rel_cartesian_middle_1.z;
    data.middle(1).flexion = radtodeg(atan2(rel_cartesian_middle_1.x,defaults.middle(1).d));
    data.middle(1).abduction = radtodeg(atan2(rel_cartesian_middle_1.y,defaults.middle(1).d));
    rel_cartesian_middle_2.x = particles(hypothesis).currentPosition(handJoint.middle_2_a);
    rel_cartesian_middle_2.z = sqrt(defaults.middle(2).d^2 - rel_cartesian_middle_2.x^2);
    data.middle(2).d = rel_cartesian_middle_2.z;
    data.middle(2).flexion = radtodeg(atan2(rel_cartesian_middle_2.x,defaults.middle(2).d));
    rel_cartesian_middle_3.x = particles(hypothesis).currentPosition(handJoint.middle_3_a);
    rel_cartesian_middle_3.z = sqrt(defaults.middle(3).d^2 - rel_cartesian_middle_3.x^2);
    data.middle(3).d = rel_cartesian_middle_3.z;
    data.middle(3).flexion = radtodeg(atan2(rel_cartesian_middle_3.x,defaults.middle(3).d));
    % Third _ - | 2DOF | 1DOF | 1DOF |
    rel_cartesian_third_1.x = particles(hypothesis).currentPosition(handJoint.third_1_a);
    rel_cartesian_third_1.y = particles(hypothesis).currentPosition(handJoint.third_1_b);
    rel_cartesian_third_1.z = sqrt(defaults.third(1).d^2 - rel_cartesian_third_1.x^2 - rel_cartesian_third_1.y^2);
    data.third(1).d = rel_cartesian_third_1.z;
    data.third(1).flexion = radtodeg(atan2(rel_cartesian_third_1.x,defaults.third(1).d));
    data.third(1).abduction = radtodeg(atan2(rel_cartesian_third_1.y,defaults.third(1).d));
    rel_cartesian_third_2.x = particles(hypothesis).currentPosition(handJoint.third_2_a);
    rel_cartesian_third_2.z = sqrt(defaults.third(2).d^2 - rel_cartesian_third_2.x^2);
    data.third(2).d = rel_cartesian_third_2.z;
    data.third(2).flexion = radtodeg(atan2(rel_cartesian_third_2.x,defaults.third(2).d));
    rel_cartesian_third_3.x = particles(hypothesis).currentPosition(handJoint.third_3_a);
    rel_cartesian_third_3.z = sqrt(defaults.third(3).d^2 - rel_cartesian_third_3.x^2);
    data.third(3).d = rel_cartesian_third_3.z;
    data.third(3).flexion = radtodeg(atan2(rel_cartesian_third_3.x,defaults.third(3).d));
    % Little _ - | 2DOF | 1DOF | 1DOF |
    rel_cartesian_little_1.x = particles(hypothesis).currentPosition(handJoint.little_1_a);
    rel_cartesian_little_1.y = particles(hypothesis).currentPosition(handJoint.little_1_b);
    rel_cartesian_little_1.z = sqrt(defaults.little(1).d^2 - rel_cartesian_little_1.x^2 - rel_cartesian_little_1.y^2);
    data.little(1).d = rel_cartesian_little_1.z;
    data.little(1).flexion = radtodeg(atan2(rel_cartesian_little_1.x,defaults.little(1).d));
    data.little(1).abduction = radtodeg(atan2(rel_cartesian_little_1.y,defaults.little(1).d));
    rel_cartesian_little_2.x = particles(hypothesis).currentPosition(handJoint.little_2_a);
    rel_cartesian_little_2.z = sqrt(defaults.little(2).d^2 - rel_cartesian_little_2.x^2);
    data.little(2).d = rel_cartesian_little_2.z;
    data.little(2).flexion = radtodeg(atan2(rel_cartesian_little_2.x,defaults.little(2).d));
    rel_cartesian_little_3.x = particles(hypothesis).currentPosition(handJoint.little_3_a);
    rel_cartesian_little_3.z = sqrt(defaults.little(3).d^2 - rel_cartesian_little_3.x^2);
    data.little(3).d = rel_cartesian_little_3.z;
    data.little(3).flexion = radtodeg(atan2(rel_cartesian_little_3.x,defaults.little(3).d));
    
    % 2. Apply to model, render and obtain rd
    handHypothesisObject.depth = handModelObject.renderHand([0;0;0;1],data);
    %handHypothesisObject.depth = handModelObject.renderHand([0;0;0;1],handModelObject.getDefaults());
%    figure(2);
%    imshow(handHypothesisObject.depth);
    % 4. Compare to os and obtain rm
    [d_height,d_width,~] = size(handHypothesisObject.depth);
    handHypothesisObject.segm = zeros(d_height,d_width,1);
    m_min = 0;
    typecast(m_min,'double');
    for i = 1:d_height
        for j = 1:d_width
            a = abs(handObservationObject.depth(i,j) - handHypothesisObject.depth(i,j));
            %m_min = m_min + min(a,40)
            m_min = m_min + min(a,40);
            if ((abs(handObservationObject.depth(i,j) - handHypothesisObject.depth(i,j)) <= 10) || (handObservationObject.depth(i,j) == 0)) % 1 cm or non-existent
                handHypothesisObject.segm(i,j) = 1;
            end
        end
    end
    
    % 6. Calculate E(o,h)
    %%% - Start by D(o,h)
    segm_int = handObservationObject.segm & handHypothesisObject.segm;
    segm_uni = handObservationObject.segm | handHypothesisObject.segm;

    segm_int_count = nnz(segm_int);
    segm_uni_count = nnz(segm_uni);
%     figure(4);
%     imshow(segm_int);
%     figure(5);
%     imshow(segm_uni);
    
    a = (m_min/(segm_uni_count+0.01));
    x = (2*segm_int_count)/(segm_int_count+segm_uni_count)
    y = 1 - x;
    b = (20*y);
    d = a + b;
    %%% - Obtain constrictions (finger interpenetration) kc(h)
    kc = (0 - min(degtorad(data.index(1).abduction) - degtorad(data.middle(1).abduction),0)) + ...
         (0 - min(degtorad(data.middle(1).abduction) - degtorad(data.third(1).abduction),0)) + ...
         (0 - min(degtorad(data.third(1).abduction) - degtorad(data.little(1).abduction),0));
    d;
    %%% - Calculate E(o,h)
    e = d+10*kc;
    % 7. Return value (minimization solution for current hypothesis p)
    e
    result = e;
    % ___________
end
