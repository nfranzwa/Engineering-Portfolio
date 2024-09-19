classdef RoboticArmGUI < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure               matlab.ui.Figure
        HomingButton           matlab.ui.control.Button
        ShutdownButton         matlab.ui.control.Button
        ResetButton            matlab.ui.control.Button
        JointSlider1           matlab.ui.control.Slider
        JointSlider2           matlab.ui.control.Slider
        JointSlider3           matlab.ui.control.Slider
        JointSlider4           matlab.ui.control.Slider
        JointSlider5           matlab.ui.control.Slider
        JointSlider6           matlab.ui.control.Slider
        JointLabel1            matlab.ui.control.Label
        JointLabel2            matlab.ui.control.Label
        JointLabel3            matlab.ui.control.Label
        JointLabel4            matlab.ui.control.Label
        JointLabel5            matlab.ui.control.Label
        JointLabel6            matlab.ui.control.Label
        XCoordEditField        matlab.ui.control.NumericEditField
        YCoordEditField        matlab.ui.control.NumericEditField
        ZCoordEditField        matlab.ui.control.NumericEditField
        XLabel                 matlab.ui.control.Label
        YLabel                 matlab.ui.control.Label
        ZLabel                 matlab.ui.control.Label
        MoveToPositionButton   matlab.ui.control.Button
        SerialPort             
    end

    properties (Access = private)
        DH_params
        JointLimits
    end

    methods (Access = private)

        function readSerialData(app, ~, ~)
            data = readline(app.SerialPort);
            disp(data);
            if contains(data, "Homing sequence completed.")
                app.HomingButton.Enable = 'on';
                app.HomingButton.Text = 'Start Homing';
            elseif contains(data, "Shutdown sequence completed.")
                app.ShutdownButton.Enable = 'on';
                app.ShutdownButton.Text = 'Start Shutdown';
            end
        end

        function sendJointAngles(app)
            sliders = [app.JointSlider1, app.JointSlider2, app.JointSlider3, ...
                       app.JointSlider4, app.JointSlider5, app.JointSlider6];
            angles = arrayfun(@(slider) slider.Value, sliders);
            command = sprintf("M%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", angles);
            write(app.SerialPort, command, "string");
        end

        function HomingButtonPushed(app, ~)
            app.HomingButton.Enable = 'off';
            app.HomingButton.Text = 'Homing...';
            write(app.SerialPort, "H", "string");
        end

        function ShutdownButtonPushed(app, ~)
            app.ShutdownButton.Enable = 'off';
            app.ShutdownButton.Text = 'Shutting down...';
            write(app.SerialPort, "S", "string");
        end

        function ResetButtonPushed(app, ~)
            write(app.SerialPort, "R", "string");
            sliders = [app.JointSlider1, app.JointSlider2, app.JointSlider3, ...
                       app.JointSlider4, app.JointSlider5, app.JointSlider6];
            for i = 1:6
                slider = sliders(i);
                slider.Value = 0;
                if slider.Value < app.JointLimits(i,1)
                    slider.Value = app.JointLimits(i,1);
                elseif slider.Value > app.JointLimits(i,2)
                    slider.Value = app.JointLimits(i,2);
                end
            end
        end

        function SliderValueChanged(app, ~)
            app.sendJointAngles();
        end

        function MoveToPositionButtonPushed(app, ~)
            x_mm = app.XCoordEditField.Value;
            y_mm = app.YCoordEditField.Value;
            z_mm = app.ZCoordEditField.Value;
            
            % Convert mm to m
            x_m = x_mm / 1000;
            y_m = y_mm / 1000;
            z_m = z_mm / 1000;
            
            % Perform inverse kinematics
            jointAngles = app.inverseKinematics([x_m; y_m; z_m]);
            
            % Update sliders
            app.JointSlider1.Value = jointAngles(1);
            app.JointSlider2.Value = jointAngles(2);
            app.JointSlider3.Value = jointAngles(3);
            app.JointSlider4.Value = jointAngles(4);
            app.JointSlider5.Value = jointAngles(5);
            app.JointSlider6.Value = jointAngles(6);
            
            % Send joint angles to Arduino
            app.sendJointAngles();
        end

        function jointAngles = inverseKinematics(app, targetPos)
            % Numerical inverse kinematics using optimization
            options = optimoptions('fmincon','Display','off');
            initialGuess = zeros(6,1);  % Initial guess for joint angles
            lb = deg2rad(app.JointLimits(:,1));  % Lower bounds
            ub = deg2rad(app.JointLimits(:,2));  % Upper bounds
            
            jointAngles = fmincon(@(q) norm(app.forwardKinematics(q) - targetPos), ...
                                  initialGuess, [], [], [], [], lb, ub, [], options);
            
            % Convert radians to degrees
            jointAngles = rad2deg(jointAngles);
            
            % Ensure joint angles are within limits
            jointAngles = max(min(jointAngles, app.JointLimits(:,2)), app.JointLimits(:,1));
        end

        function endEffectorPos = forwardKinematics(app, q)
            % Forward kinematics using DH parameters
            T = eye(4);
            for i = 1:6
                a = app.DH_params(i,1);
                alpha = app.DH_params(i,2);
                d = app.DH_params(i,3);
                theta = app.DH_params(i,4) + q(i);
                
                T = T * [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta);
                         sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
                         0 sin(alpha) cos(alpha) d;
                         0 0 0 1];
            end
            endEffectorPos = T(1:3,4);
        end

        function createComponents(app)
            % Create UIFigure
            app.UIFigure = uifigure;
            app.UIFigure.Position = [100 100 640 600];
            app.UIFigure.Name = 'Robotic Arm Control';

            % Create HomingButton
            app.HomingButton = uibutton(app.UIFigure);
            app.HomingButton.ButtonPushedFcn = createCallbackFcn(app, @HomingButtonPushed, true);
            app.HomingButton.Position = [50 550 100 22];
            app.HomingButton.Text = 'Start Homing';

            % Create ShutdownButton
            app.ShutdownButton = uibutton(app.UIFigure);
            app.ShutdownButton.ButtonPushedFcn = createCallbackFcn(app, @ShutdownButtonPushed, true);
            app.ShutdownButton.Position = [270 550 100 22];
            app.ShutdownButton.Text = 'Start Shutdown';

            % Create ResetButton
            app.ResetButton = uibutton(app.UIFigure);
            app.ResetButton.ButtonPushedFcn = createCallbackFcn(app, @ResetButtonPushed, true);
            app.ResetButton.Position = [490 550 100 22];
            app.ResetButton.Text = 'Reset Joints';

            % Create JointSliders and Labels
            sliderProperties = {'JointSlider1', 'JointSlider2', 'JointSlider3', ...
                                'JointSlider4', 'JointSlider5', 'JointSlider6'};
            labelProperties = {'JointLabel1', 'JointLabel2', 'JointLabel3', ...
                               'JointLabel4', 'JointLabel5', 'JointLabel6'};
            
            for i = 1:6
                app.(labelProperties{i}) = uilabel(app.UIFigure);
                app.(labelProperties{i}).Position = [50 510-60*(i-1) 30 22];
                app.(labelProperties{i}).Text = sprintf('J%d', i);

                app.(sliderProperties{i}) = uislider(app.UIFigure);
                app.(sliderProperties{i}).Limits = app.JointLimits(i,:);
                app.(sliderProperties{i}).ValueChangedFcn = createCallbackFcn(app, @SliderValueChanged, true);
                app.(sliderProperties{i}).Position = [100 520-60*(i-1) 500 3];
            end

            % Create X, Y, Z input fields and labels
            app.XLabel = uilabel(app.UIFigure);
            app.XLabel.Position = [50 140 40 22];
            app.XLabel.Text = 'X (mm):';

            app.XCoordEditField = uieditfield(app.UIFigure, 'numeric');
            app.XCoordEditField.Position = [90 140 80 22];

            app.YLabel = uilabel(app.UIFigure);
            app.YLabel.Position = [190 140 40 22];
            app.YLabel.Text = 'Y (mm):';

            app.YCoordEditField = uieditfield(app.UIFigure, 'numeric');
            app.YCoordEditField.Position = [230 140 80 22];

            app.ZLabel = uilabel(app.UIFigure);
            app.ZLabel.Position = [330 140 40 22];
            app.ZLabel.Text = 'Z (mm):';

            app.ZCoordEditField = uieditfield(app.UIFigure, 'numeric');
            app.ZCoordEditField.Position = [370 140 80 22];

            % Create MoveToPositionButton
            app.MoveToPositionButton = uibutton(app.UIFigure);
            app.MoveToPositionButton.ButtonPushedFcn = createCallbackFcn(app, @MoveToPositionButtonPushed, true);
            app.MoveToPositionButton.Position = [470 140 120 22];
            app.MoveToPositionButton.Text = 'Move to Position';
        end
    end

    methods (Access = public)

        function app = RoboticArmGUI
            % Create and configure components
            app.JointLimits = [-90 90; -40 60; -60 90; -180 180; -90 90; -180 180];
            createComponents(app)

            % Initialize DH parameters
            app.DH_params = [ 0.1105 -pi/2 0.02342 0;
                              0 pi 0.180 -pi/2;
                              0 pi/2 -0.04350 pi;
                              -0.17635 -pi/2 0 0;
                              0 pi/2 0 0;
                              -0.0628 pi -45.250 pi];

            % Initialize serial connection
            app.SerialPort = serialport("/dev/cu.usbmodem11101", 9600); % Replace with your Arduino port
            configureCallback(app.SerialPort, "terminator", @app.readSerialData);

            % Show the figure after all components are created
            app.UIFigure.Visible = 'on';
        end

        function delete(app)
            % Delete the app
            delete(app.SerialPort);
            delete(app.UIFigure);
        end
    end
end