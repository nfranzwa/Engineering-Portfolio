function createParol6GUI()
    % Create the figure
    fig = uifigure('Position', [100, 100, 700, 500], 'Name', 'Parol 6 Control Panel');
    
    % Create sliders and input fields for each joint angle
    uilabel(fig, 'Position', [20, 450, 100, 22], 'Text', 'Joint Angles:');
    
    % Labels, sliders, and input fields for each joint angle in two columns
    jointAngleSliders = gobjects(1, 6);
    jointAngleFields = gobjects(1, 6);
    for i = 1:3
        uilabel(fig, 'Position', [20, 450 - 80*i, 50, 22], 'Text', sprintf('Joint %d:', i));
        jointAngleSliders(i) = uislider(fig, 'Position', [80, 455 - 80*i, 200, 3], 'Limits', [-180 180], 'Value', 0);
        jointAngleFields(i) = uieditfield(fig, 'numeric', 'Position', [290, 440 - 80*i, 60, 22], 'Value', 0);
        jointAngleSliders(i).ValueChangedFcn = @(sld, event) sliderChanged(i);
        jointAngleFields(i).ValueChangedFcn = @(fld, event) fieldChanged(i);
    end
    for i = 4:6
        uilabel(fig, 'Position', [360, 450 - 80*(i-3), 50, 22], 'Text', sprintf('Joint %d:', i));
        jointAngleSliders(i) = uislider(fig, 'Position', [420, 455 - 80*(i-3), 200, 3], 'Limits', [-180 180], 'Value', 0);
        jointAngleFields(i) = uieditfield(fig, 'numeric', 'Position', [630, 440 - 80*(i-3), 60, 22], 'Value', 0);
        jointAngleSliders(i).ValueChangedFcn = @(sld, event) sliderChanged(i);
        jointAngleFields(i).ValueChangedFcn = @(fld, event) fieldChanged(i);
    end
    
    % Create buttons for simulation control and center them
    buttonWidth = 100;
    buttonHeight = 30;
    buttonSpacing = 20;
    figWidth = fig.Position(3);
    
    stopButton = uibutton(fig, 'Position', [(figWidth/2 - buttonWidth - buttonSpacing/2), 100, buttonWidth, buttonHeight], 'Text', 'Stop', 'ButtonPushedFcn', @(stopButton, event) stopSimulation());
    resetButton = uibutton(fig, 'Position', [(figWidth/2 + buttonSpacing/2), 100, buttonWidth, buttonHeight], 'Text', 'Reset', 'ButtonPushedFcn', @(resetButton, event) resetSimulation());
    
    % Create display for XYZ coordinates
    uilabel(fig, 'Position', [20, 60, 100, 22], 'Text', 'XYZ Coordinates:');
    xyzDisplay = uilabel(fig, 'Position', [120, 60, 250, 22], 'Text', 'X: 0.000, Y: 0.000, Z: 0.000');
    
    % Load the Simulink model
    model = 'PAROL6';
    load_system(model);
    
    % Find Outport blocks
    xPort = [model '/1'];
    yPort = [model '/Y'];
    zPort = [model '/3'];
    
    % Start the simulation in normal mode
    set_param(model, 'SimulationMode', 'normal');
    set_param(model, 'SimulationCommand', 'start');
    
    % Flag to control the simulation loop
    isRunning = true;
    
    % Create a timer for updating XYZ coordinates
    updateTimer = timer('ExecutionMode', 'fixedRate', 'Period', 0.1, 'TimerFcn', @updateXYZCoordinates);
    start(updateTimer);
    
    % Nested function to update the simulation with new joint angles
    function updateSimulation()
        % Get joint angles from the sliders
        jointAngles = zeros(1, 6);
        for j = 1:6
            jointAngles(j) = jointAngleSliders(j).Value;
        end
        
        % Set the joint angles
        first_slider_gain_block = sprintf('%s/Slider Gain', model);
        set_param(first_slider_gain_block, 'Gain', num2str(jointAngles(1)));
        for j = 1:5
            slider_gain_block = sprintf('%s/Slider Gain%d', model, j);
            set_param(slider_gain_block, 'Gain', num2str(jointAngles(j + 1)));
        end
        
        % Update the model to apply the new parameters
        set_param(model, 'SimulationCommand', 'update');
    end

    % Nested function to handle slider changes
    function sliderChanged(index)
        jointAngleFields(index).Value = jointAngleSliders(index).Value;
        updateSimulation();
    end

    % Nested function to handle field changes
    function fieldChanged(index)
        jointAngleSliders(index).Value = jointAngleFields(index).Value;
        updateSimulation();
    end

    % Nested function to stop the simulation
    function stopSimulation()
        set_param(model, 'SimulationCommand', 'stop');
        isRunning = false;
        stop(updateTimer);
        delete(updateTimer);
        
        % Reset the joint angles to 0
        for j = 1:6
            jointAngleSliders(j).Value = 0;
            jointAngleFields(j).Value = 0;
        end
        
        % Update the simulation with reset joint angles
        updateSimulation();
        
        close(fig);  % Close the GUI
    end

    % Nested function to reset the simulation
    function resetSimulation()
        set_param(model, 'SimulationCommand', 'stop');
        
        % Reset the joint angles to 0
        for j = 1:6
            jointAngleSliders(j).Value = 0;
            jointAngleFields(j).Value = 0;
        end
        
        % Update the simulation with reset joint angles
        updateSimulation();
        
        % Restart the simulation
        set_param(model, 'SimulationCommand', 'start');
        
        xyzDisplay.Text = 'X: 0.000, Y: 0.000, Z: 0.000';
    end

    % Nested function to get XYZ coordinates
    function xyz = getXYZCoordinates()
        try
            xRTO = get_param(xPort, 'RuntimeObject');
            yRTO = get_param(yPort, 'RuntimeObject');
            zRTO = get_param(zPort, 'RuntimeObject');
            
            x = xRTO.InputPort(1).Data;
            y = yRTO.InputPort(1).Data;
            z = zRTO.InputPort(1).Data;
            
            xyz = [x, y, z];
        catch e
            disp(['Error getting XYZ coordinates: ' e.message]);
            xyz = [0, 0, 0];
        end
    end

    % Nested function to update the XYZ coordinates in the GUI
    function updateXYZCoordinates(~, ~)
        if isRunning
            xyz = getXYZCoordinates();
            xyzDisplay.Text = sprintf('X: %.3f, Y: %.3f, Z: %.3f', xyz(1), xyz(2), xyz(3));
        end
    end
end