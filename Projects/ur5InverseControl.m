%% Developer: Seyi R. Afolayan 
% Wrote the ur5InverseControl function for Team 7 Place and Draw final project.

function done = ur5InverseControl(ur5, startFrame, overDistance1, downDistance, overDistance2, lineTime)
    % Initialize step sizes in millimeters
    stepSizes = [overDistance1, downDistance, overDistance2] * 1000;
    
    % Calculate timing for each movement step based on line time and step size
    timings = [lineTime / stepSizes(1), lineTime / stepSizes(2), lineTime / stepSizes(3)];
    
    % Iterate over the first overdistance
    for i = 1:stepSizes(1)
        % Calculate new transform for moving over
        newTransformOver1 = startFrame * [1, 0, 0, 0; 0, 1, 0, -overDistance1 * i / stepSizes(1); 0, 0, 1, 0; 0, 0, 0, 1];
        
        % Find the UR5 joint angles for the new transform
        thetaSolutionsOver1 = ur5InvKin(newTransformOver1);
        
        % Determine the best joint configuration
        jointIteration = determineAngle(thetaSolutionsOver1, ur5);
        
        % Move UR5 to the first position
        ur5.move_joints(thetaSolutionsOver1(1:6, jointIteration), timings(1));
        disp("Moving to first position");
        pause(timings(1));
        
        % Update the transform frame
        Frame_B = tf_frame("base_link", "frame1_" + i, newTransformOver1);
    end
    
    % Repeat for the down distance
    for i = 1:stepSizes(2)
        newTransformDown = newTransformOver1 * [1, 0, 0, downDistance * i / stepSizes(2); 0, 1, 0, 0; 0, 0, 1, 0; 0, 0, 0, 1];
        thetaSolutionsDown = ur5InvKin(newTransformDown);
        jointIteration = determineAngle(thetaSolutionsDown, ur5);
        
        % Move UR5 to the second position
        ur5.move_joints(thetaSolutionsDown(1:6, jointIteration), timings(2));
        disp("Moving to second position");
        pause(timings(2));
        
        % Update the transform frame
        Frame_C = tf_frame("base_link", "frame2_" + i, newTransformDown);
    end
    
    % Repeat for the second over distance
    for i = 1:stepSizes(3)
        newTransformOver2 = newTransformDown * [1, 0, 0, 0; 0, 1, 0, -overDistance2 * i / stepSizes(3); 0, 0, 1, 0; 0, 0, 0, 1];
        thetaSolutionsOver2 = ur5InvKin(newTransformOver2);
        jointIteration = determineAngle(thetaSolutionsOver2, ur5);
        
        % Move UR5 to the third position
        ur5.move_joints(thetaSolutionsOver2(1:6, jointIteration), timings(3));
        disp("Moving to third position");
        pause(timings(3));
        
        % Update the transform frame
        Frame_D = tf_frame("base_link", "frame3_" + i, newTransformOver2);
    end
    
    % Set done flag to indicate completion
    done = 1;
end

