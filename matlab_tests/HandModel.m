classdef HandModel < handle
    properties (GetAccess = private)
        obj
        joints
    end
    methods
        % Se for preciso colocar mais uma joint para o thumb
        function obj = HandModel
            % DH parameter 'a' is discarded _
            % Setting initial displacement (a) parameter is irrelevant
            % since the joints are not separated, and therefore, to all
            % cases a = 0 is applied and is never changed
        end
        
        function default = getDefaults(obj)
            % _ FINGERS _
            % Root _
            default.yaw = 0;
            default.roll = 0;
            default.pitch = 0;
            % Thumb _
            default.thumb(1).d = 60;
            default.thumb(1).abduction = 0;
            default.thumb(1).flexion = 0;
            default.thumb(2).d = 40;
            default.thumb(2).flexion = 0;
            default.thumb(3).d = 30;
            default.thumb(3).flexion = 0;
            % Index _
            default.index(1).d = 45;
            default.index(1).abduction = 0;
            default.index(1).flexion = 0;
            default.index(2).d = 40;
            default.index(2).flexion = 0;
            default.index(3).d = 25;
            default.index(3).flexion = 0;
            % Middle _
            default.middle(1).d = 50;
            default.middle(1).abduction = 0;
            default.middle(1).flexion = 0;
            default.middle(2).d = 40;
            default.middle(2).flexion = 0;
            default.middle(3).d = 30;
            default.middle(3).flexion = 0;
            % Third _
            default.third(1).d = 45;
            default.third(1).abduction = 0;
            default.third(1).flexion = 0;
            default.third(2).d = 40;
            default.third(2).flexion = 0;
            default.third(3).d = 30;
            default.third(3).flexion = 0;
            % Little _
            default.little(1).d = 40;
            default.little(1).abduction = 0;
            default.little(1).flexion = 0;
            default.little(2).d = 35;
            default.little(2).flexion = 0;
            default.little(3).d = 25;
            default.little(3).flexion = 0;
            
            default.max_size = default.middle(1).d + default.middle(2).d + default.middle(3).d;
        end
        
        function depth = renderHand(obj,origin,data)
            % DH Parameters and Homogeneous Matrix Calculation ____________
            % (alpha, a, d, theta)
            % _ Root _
            obj.joints.root(1).t = transformDH(degtorad(-90),0,0,degtorad(0));
            obj.joints.root(2).t = transformDH(degtorad(data.roll),0,0,degtorad(-90+data.yaw));
            obj.joints.root(3).t = transformDH(degtorad(data.pitch),0,0,degtorad(-90));
            obj.joints.root(4).t = transformDH(degtorad(0),0,0,degtorad(90));
            
            % Thumb finger
            obj.joints.thumb(1).t = transformDH(degtorad(60),0,0,degtorad(0));
            obj.joints.thumb(2).t = transformDH(degtorad(data.thumb(1).abduction),0,0,degtorad(0));
            obj.joints.thumb(3).t = transformDH(degtorad(data.thumb(1).flexion),0,0,degtorad(-90));
            obj.joints.thumb(4).t = transformDH(degtorad(data.thumb(2).flexion),0,data.thumb(1).d,degtorad(0));
            obj.joints.thumb(5).t = transformDH(degtorad(data.thumb(3).flexion),0,data.thumb(2).d,degtorad(0));
            obj.joints.thumb(6).t = transformDH(degtorad(0),0,data.thumb(3).d,degtorad(0));
            
            % Index finger
            obj.joints.index(1).t = transformDH(degtorad(15),0,0,degtorad(0));
            obj.joints.index(2).t = transformDH(degtorad(data.index(1).abduction),0,90,degtorad(0));
            obj.joints.index(3).t = transformDH(degtorad(data.index(1).flexion),0,0,degtorad(-90));
            obj.joints.index(4).t = transformDH(degtorad(data.index(2).flexion),0,data.index(1).d,degtorad(0));
            obj.joints.index(5).t = transformDH(degtorad(data.index(3).flexion),0,data.index(2).d,degtorad(0));
            obj.joints.index(6).t = transformDH(degtorad(0),0,data.index(3).d,degtorad(0));
            
            % Middle finger
            obj.joints.middle(1).t = transformDH(degtorad(0),0,0,degtorad(0));
            obj.joints.middle(2).t = transformDH(degtorad(data.middle(1).abduction),0,100,degtorad(0));
            obj.joints.middle(3).t = transformDH(degtorad(data.middle(1).flexion),0,0,degtorad(-90));
            obj.joints.middle(4).t = transformDH(degtorad(data.middle(2).flexion),0,data.middle(1).d,degtorad(0));
            obj.joints.middle(5).t = transformDH(degtorad(data.middle(3).flexion),0,data.middle(2).d,degtorad(0));
            obj.joints.middle(6).t = transformDH(degtorad(0),0,data.middle(3).d,degtorad(0));
            
            % Third finger
            obj.joints.third(1).t = transformDH(degtorad(-15),0,0,degtorad(0));
            obj.joints.third(2).t = transformDH(degtorad(data.third(1).abduction),0,90,degtorad(0));
            obj.joints.third(3).t = transformDH(degtorad(data.third(1).flexion),0,0,degtorad(-90));
            obj.joints.third(4).t = transformDH(degtorad(data.third(2).flexion),0,data.third(1).d,degtorad(0));
            obj.joints.third(5).t = transformDH(degtorad(data.third(3).flexion),0,data.third(2).d,degtorad(0));
            obj.joints.third(6).t = transformDH(degtorad(0),0,data.third(3).d,degtorad(0));
            
            % Little finger
            obj.joints.little(1).t = transformDH(degtorad(-30),0,0,degtorad(0));
            obj.joints.little(2).t = transformDH(degtorad(data.little(1).abduction),0,80,degtorad(0));
            obj.joints.little(3).t = transformDH(degtorad(data.little(1).flexion),0,0,degtorad(-90));
            obj.joints.little(4).t = transformDH(degtorad(data.little(2).flexion),0,data.little(1).d,degtorad(0));
            obj.joints.little(5).t = transformDH(degtorad(data.little(3).flexion),0,data.little(2).d,degtorad(0));
            obj.joints.little(6).t = transformDH(degtorad(0),0,data.little(3).d,degtorad(0));
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Forward Kinematic Chain and Position Coordinates ____________
            % Root
            obj.joints.root(1).tm = obj.joints.root(1).t * origin;
            obj.joints.root(2).tm = obj.joints.root(1).t * obj.joints.root(2).t * origin;
            obj.joints.root(3).tm = obj.joints.root(1).t * obj.joints.root(2).t * obj.joints.root(3).t * origin;
            obj.joints.root(4).tm = obj.joints.root(1).t * obj.joints.root(2).t * obj.joints.root(3).t * obj.joints.root(4).t * origin;
            
            joint_position.root(1).x = obj.joints.root(4).tm(1)/obj.joints.root(4).tm(4);
            joint_position.root(1).y = obj.joints.root(4).tm(2)/obj.joints.root(4).tm(4);
            joint_position.root(1).z = obj.joints.root(4).tm(3)/obj.joints.root(4).tm(4);
            
            % Thumb finger
            obj.joints.thumb(1).tm = obj.joints.root(1).t * obj.joints.root(2).t * obj.joints.root(3).t * obj.joints.root(4).t * obj.joints.thumb(1).t * origin;
            obj.joints.thumb(2).tm = obj.joints.root(1).t * obj.joints.root(2).t * obj.joints.root(3).t * obj.joints.root(4).t * obj.joints.thumb(1).t * obj.joints.thumb(2).t * origin;
            obj.joints.thumb(3).tm = obj.joints.root(1).t * obj.joints.root(2).t * obj.joints.root(3).t * obj.joints.root(4).t * obj.joints.thumb(1).t * obj.joints.thumb(2).t * obj.joints.thumb(3).t * origin;
            obj.joints.thumb(4).tm = obj.joints.root(1).t * obj.joints.root(2).t * obj.joints.root(3).t * obj.joints.root(4).t * obj.joints.thumb(1).t * obj.joints.thumb(2).t * obj.joints.thumb(3).t * obj.joints.thumb(4).t * origin;
            obj.joints.thumb(5).tm = obj.joints.root(1).t * obj.joints.root(2).t * obj.joints.root(3).t * obj.joints.root(4).t * obj.joints.thumb(1).t * obj.joints.thumb(2).t * obj.joints.thumb(3).t * obj.joints.thumb(4).t * obj.joints.thumb(5).t * origin;
            obj.joints.thumb(6).tm = obj.joints.root(1).t * obj.joints.root(2).t * obj.joints.root(3).t * obj.joints.root(4).t * obj.joints.thumb(1).t * obj.joints.thumb(2).t * obj.joints.thumb(3).t * obj.joints.thumb(4).t * obj.joints.thumb(5).t * obj.joints.thumb(6).t * origin;
            
            joint_position.thumb(1).x = obj.joints.thumb(4).tm(1)/obj.joints.thumb(4).tm(4);
            joint_position.thumb(1).y = obj.joints.thumb(4).tm(2)/obj.joints.thumb(4).tm(4);
            joint_position.thumb(1).z = obj.joints.thumb(4).tm(3)/obj.joints.thumb(4).tm(4);
            
            joint_position.thumb(2).x = obj.joints.thumb(5).tm(1)/obj.joints.thumb(5).tm(4);
            joint_position.thumb(2).y = obj.joints.thumb(5).tm(2)/obj.joints.thumb(5).tm(4);
            joint_position.thumb(2).z = obj.joints.thumb(5).tm(3)/obj.joints.thumb(5).tm(4);
            
            joint_position.thumb(3).x = obj.joints.thumb(6).tm(1)/obj.joints.thumb(6).tm(4);
            joint_position.thumb(3).y = obj.joints.thumb(6).tm(2)/obj.joints.thumb(6).tm(4);
            joint_position.thumb(3).z = obj.joints.thumb(6).tm(3)/obj.joints.thumb(6).tm(4);
            
            % Index finger
            obj.joints.index(1).tm = obj.joints.root(1).t * obj.joints.root(2).t * obj.joints.root(3).t * obj.joints.root(4).t * obj.joints.index(1).t * origin;
            obj.joints.index(2).tm = obj.joints.root(1).t * obj.joints.root(2).t * obj.joints.root(3).t * obj.joints.root(4).t * obj.joints.index(1).t * obj.joints.index(2).t * origin;
            obj.joints.index(3).tm = obj.joints.root(1).t * obj.joints.root(2).t * obj.joints.root(3).t * obj.joints.root(4).t * obj.joints.index(1).t * obj.joints.index(2).t * obj.joints.index(3).t * origin;
            obj.joints.index(4).tm = obj.joints.root(1).t * obj.joints.root(2).t * obj.joints.root(3).t * obj.joints.root(4).t * obj.joints.index(1).t * obj.joints.index(2).t * obj.joints.index(3).t * obj.joints.index(4).t * origin;
            obj.joints.index(5).tm = obj.joints.root(1).t * obj.joints.root(2).t * obj.joints.root(3).t * obj.joints.root(4).t * obj.joints.index(1).t * obj.joints.index(2).t * obj.joints.index(3).t * obj.joints.index(4).t * obj.joints.index(5).t * origin;
            obj.joints.index(6).tm = obj.joints.root(1).t * obj.joints.root(2).t * obj.joints.root(3).t * obj.joints.root(4).t * obj.joints.index(1).t * obj.joints.index(2).t * obj.joints.index(3).t * obj.joints.index(4).t * obj.joints.index(5).t * obj.joints.index(6).t * origin;
            
            joint_position.index(1).x = obj.joints.index(3).tm(1)/obj.joints.index(3).tm(4);
            joint_position.index(1).y = obj.joints.index(3).tm(2)/obj.joints.index(3).tm(4);
            joint_position.index(1).z = obj.joints.index(3).tm(3)/obj.joints.index(3).tm(4);
            
            joint_position.index(2).x = obj.joints.index(4).tm(1)/obj.joints.index(4).tm(4);
            joint_position.index(2).y = obj.joints.index(4).tm(2)/obj.joints.index(4).tm(4);
            joint_position.index(2).z = obj.joints.index(4).tm(3)/obj.joints.index(4).tm(4);
            
            joint_position.index(3).x = obj.joints.index(5).tm(1)/obj.joints.index(5).tm(4);
            joint_position.index(3).y = obj.joints.index(5).tm(2)/obj.joints.index(5).tm(4);
            joint_position.index(3).z = obj.joints.index(5).tm(3)/obj.joints.index(5).tm(4);
            
            joint_position.index(4).x = obj.joints.index(6).tm(1)/obj.joints.index(6).tm(4);
            joint_position.index(4).y = obj.joints.index(6).tm(2)/obj.joints.index(6).tm(4);
            joint_position.index(4).z = obj.joints.index(6).tm(3)/obj.joints.index(6).tm(4);
            
            % Middle finger
            obj.joints.middle(1).tm = obj.joints.root(1).t * obj.joints.root(2).t * obj.joints.root(3).t * obj.joints.root(4).t * obj.joints.middle(1).t * origin;
            obj.joints.middle(2).tm = obj.joints.root(1).t * obj.joints.root(2).t * obj.joints.root(3).t * obj.joints.root(4).t * obj.joints.middle(1).t * obj.joints.middle(2).t * origin;
            obj.joints.middle(3).tm = obj.joints.root(1).t * obj.joints.root(2).t * obj.joints.root(3).t * obj.joints.root(4).t * obj.joints.middle(1).t * obj.joints.middle(2).t * obj.joints.middle(3).t * origin;
            obj.joints.middle(4).tm = obj.joints.root(1).t * obj.joints.root(2).t * obj.joints.root(3).t * obj.joints.root(4).t * obj.joints.middle(1).t * obj.joints.middle(2).t * obj.joints.middle(3).t * obj.joints.middle(4).t * origin;
            obj.joints.middle(5).tm = obj.joints.root(1).t * obj.joints.root(2).t * obj.joints.root(3).t * obj.joints.root(4).t * obj.joints.middle(1).t * obj.joints.middle(2).t * obj.joints.middle(3).t * obj.joints.middle(4).t * obj.joints.middle(5).t * origin;
            obj.joints.middle(6).tm = obj.joints.root(1).t * obj.joints.root(2).t * obj.joints.root(3).t * obj.joints.root(4).t * obj.joints.middle(1).t * obj.joints.middle(2).t * obj.joints.middle(3).t * obj.joints.middle(4).t * obj.joints.middle(5).t * obj.joints.middle(6).t * origin;
            
            joint_position.middle(1).x = obj.joints.middle(3).tm(1)/obj.joints.middle(3).tm(4);
            joint_position.middle(1).y = obj.joints.middle(3).tm(2)/obj.joints.middle(3).tm(4);
            joint_position.middle(1).z = obj.joints.middle(3).tm(3)/obj.joints.middle(3).tm(4);
            
            joint_position.middle(2).x = obj.joints.middle(4).tm(1)/obj.joints.middle(4).tm(4);
            joint_position.middle(2).y = obj.joints.middle(4).tm(2)/obj.joints.middle(4).tm(4);
            joint_position.middle(2).z = obj.joints.middle(4).tm(3)/obj.joints.middle(4).tm(4);
            
            joint_position.middle(3).x = obj.joints.middle(5).tm(1)/obj.joints.middle(5).tm(4);
            joint_position.middle(3).y = obj.joints.middle(5).tm(2)/obj.joints.middle(5).tm(4);
            joint_position.middle(3).z = obj.joints.middle(5).tm(3)/obj.joints.middle(5).tm(4);
            
            joint_position.middle(4).x = obj.joints.middle(6).tm(1)/obj.joints.middle(6).tm(4);
            joint_position.middle(4).y = obj.joints.middle(6).tm(2)/obj.joints.middle(6).tm(4);
            joint_position.middle(4).z = obj.joints.middle(6).tm(3)/obj.joints.middle(6).tm(4);
            
            % Third finger
            obj.joints.third(1).tm = obj.joints.root(1).t * obj.joints.root(2).t * obj.joints.root(3).t * obj.joints.root(4).t * obj.joints.third(1).t * origin;
            obj.joints.third(2).tm = obj.joints.root(1).t * obj.joints.root(2).t * obj.joints.root(3).t * obj.joints.root(4).t * obj.joints.third(1).t * obj.joints.third(2).t * origin;
            obj.joints.third(3).tm = obj.joints.root(1).t * obj.joints.root(2).t * obj.joints.root(3).t * obj.joints.root(4).t * obj.joints.third(1).t * obj.joints.third(2).t * obj.joints.third(3).t * origin;
            obj.joints.third(4).tm = obj.joints.root(1).t * obj.joints.root(2).t * obj.joints.root(3).t * obj.joints.root(4).t * obj.joints.third(1).t * obj.joints.third(2).t * obj.joints.third(3).t * obj.joints.third(4).t * origin;
            obj.joints.third(5).tm = obj.joints.root(1).t * obj.joints.root(2).t * obj.joints.root(3).t * obj.joints.root(4).t * obj.joints.third(1).t * obj.joints.third(2).t * obj.joints.third(3).t * obj.joints.third(4).t * obj.joints.third(5).t * origin;
            obj.joints.third(6).tm = obj.joints.root(1).t * obj.joints.root(2).t * obj.joints.root(3).t * obj.joints.root(4).t * obj.joints.third(1).t * obj.joints.third(2).t * obj.joints.third(3).t * obj.joints.third(4).t * obj.joints.third(5).t * obj.joints.third(6).t * origin;
            
            joint_position.third(1).x = obj.joints.third(3).tm(1)/obj.joints.third(3).tm(4);
            joint_position.third(1).y = obj.joints.third(3).tm(2)/obj.joints.third(3).tm(4);
            joint_position.third(1).z = obj.joints.third(3).tm(3)/obj.joints.third(3).tm(4);
            
            joint_position.third(2).x = obj.joints.third(4).tm(1)/obj.joints.third(4).tm(4);
            joint_position.third(2).y = obj.joints.third(4).tm(2)/obj.joints.third(4).tm(4);
            joint_position.third(2).z = obj.joints.third(4).tm(3)/obj.joints.third(4).tm(4);
            
            joint_position.third(3).x = obj.joints.third(5).tm(1)/obj.joints.third(5).tm(4);
            joint_position.third(3).y = obj.joints.third(5).tm(2)/obj.joints.third(5).tm(4);
            joint_position.third(3).z = obj.joints.third(5).tm(3)/obj.joints.third(5).tm(4);
            
            joint_position.third(4).x = obj.joints.third(6).tm(1)/obj.joints.third(6).tm(4);
            joint_position.third(4).y = obj.joints.third(6).tm(2)/obj.joints.third(6).tm(4);
            joint_position.third(4).z = obj.joints.third(6).tm(3)/obj.joints.third(6).tm(4);
            
            % Little finger
            obj.joints.little(1).tm = obj.joints.root(1).t * obj.joints.root(2).t * obj.joints.root(3).t * obj.joints.root(4).t * obj.joints.little(1).t * origin;
            obj.joints.little(2).tm = obj.joints.root(1).t * obj.joints.root(2).t * obj.joints.root(3).t * obj.joints.root(4).t * obj.joints.little(1).t * obj.joints.little(2).t * origin;
            obj.joints.little(3).tm = obj.joints.root(1).t * obj.joints.root(2).t * obj.joints.root(3).t * obj.joints.root(4).t * obj.joints.little(1).t * obj.joints.little(2).t * obj.joints.little(3).t * origin;
            obj.joints.little(4).tm = obj.joints.root(1).t * obj.joints.root(2).t * obj.joints.root(3).t * obj.joints.root(4).t * obj.joints.little(1).t * obj.joints.little(2).t * obj.joints.little(3).t * obj.joints.little(4).t * origin;
            obj.joints.little(5).tm = obj.joints.root(1).t * obj.joints.root(2).t * obj.joints.root(3).t * obj.joints.root(4).t * obj.joints.little(1).t * obj.joints.little(2).t * obj.joints.little(3).t * obj.joints.little(4).t * obj.joints.little(5).t * origin;
            obj.joints.little(6).tm = obj.joints.root(1).t * obj.joints.root(2).t * obj.joints.root(3).t * obj.joints.root(4).t * obj.joints.little(1).t * obj.joints.little(2).t * obj.joints.little(3).t * obj.joints.little(4).t * obj.joints.little(5).t * obj.joints.little(6).t * origin;
            
            joint_position.little(1).x = obj.joints.little(3).tm(1)/obj.joints.little(3).tm(4);
            joint_position.little(1).y = obj.joints.little(3).tm(2)/obj.joints.little(3).tm(4);
            joint_position.little(1).z = obj.joints.little(3).tm(3)/obj.joints.little(3).tm(4);
            
            joint_position.little(2).x = obj.joints.little(4).tm(1)/obj.joints.little(4).tm(4);
            joint_position.little(2).y = obj.joints.little(4).tm(2)/obj.joints.little(4).tm(4);
            joint_position.little(2).z = obj.joints.little(4).tm(3)/obj.joints.little(4).tm(4);
            
            joint_position.little(3).x = obj.joints.little(5).tm(1)/obj.joints.little(5).tm(4);
            joint_position.little(3).y = obj.joints.little(5).tm(2)/obj.joints.little(5).tm(4);
            joint_position.little(3).z = obj.joints.little(5).tm(3)/obj.joints.little(5).tm(4);
            
            joint_position.little(4).x = obj.joints.little(6).tm(1)/obj.joints.little(6).tm(4);
            joint_position.little(4).y = obj.joints.little(6).tm(2)/obj.joints.little(6).tm(4);
            joint_position.little(4).z = obj.joints.little(6).tm(3)/obj.joints.little(6).tm(4);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % _____________________________________________________________
            % Draw hand hypothesis in both 3D figure and get both
            % segmentation and depth map
            figure(1);
            clf;
            hold on;
            xlim([-250 250]);
            ylim([-250 250]);
            zlim([-250 250]);
            
            colormap(gray(4096));
            
            % Thumb
            thumb_finger.x = [joint_position.root(1).x joint_position.thumb(1).x joint_position.thumb(2).x joint_position.thumb(3).x];
            thumb_finger.y = [joint_position.root(1).y joint_position.thumb(1).y joint_position.thumb(2).y joint_position.thumb(3).y];
            thumb_finger.z = [joint_position.root(1).z joint_position.thumb(1).z joint_position.thumb(2).z joint_position.thumb(3).z];
            thumb_finger.c = [(joint_position.root(1).z+250)/500 ...
                               (joint_position.thumb(1).z+250)/500 ...
                               (joint_position.thumb(2).z+250)/500 ...
                               (joint_position.thumb(3).z+250)/500];
            surface('XData',[thumb_finger.x(:) thumb_finger.x(:)],...
                    'YData',[thumb_finger.y(:) thumb_finger.y(:)],...
                    'ZData',[thumb_finger.z(:) thumb_finger.z(:)],...
                    'CData',[thumb_finger.c(:) thumb_finger.c(:)],...
                    'FaceColor','none','EdgeColor','flat','Marker','none','linewidth',15);
            % Index
            index_finger.x = [joint_position.root(1).x joint_position.index(1).x joint_position.index(2).x joint_position.index(3).x joint_position.index(4).x];
            index_finger.y = [joint_position.root(1).y joint_position.index(1).y joint_position.index(2).y joint_position.index(3).y joint_position.index(4).y];
            index_finger.z = [joint_position.root(1).z joint_position.index(1).z joint_position.index(2).z joint_position.index(3).z joint_position.index(4).z];
            index_finger.c = [(joint_position.root(1).z+250)/500 ...
                              (joint_position.index(1).z+250)/500 ...
                              (joint_position.index(2).z+250)/500 ...
                              (joint_position.index(3).z+250)/500 ...
                              (joint_position.index(4).z+250)/500];
            surface('XData',[index_finger.x(:) index_finger.x(:)],...
                    'YData',[index_finger.y(:) index_finger.y(:)],...
                    'ZData',[index_finger.z(:) index_finger.z(:)],...
                    'CData',[index_finger.c(:) index_finger.c(:)],...
                    'FaceColor','none','EdgeColor','flat','Marker','none','linewidth',15);
            % Middle
            middle_finger.x = [joint_position.root(1).x joint_position.middle(1).x joint_position.middle(2).x joint_position.middle(3).x joint_position.middle(4).x];
            middle_finger.y = [joint_position.root(1).y joint_position.middle(1).y joint_position.middle(2).y joint_position.middle(3).y joint_position.middle(4).y];
            middle_finger.z = [joint_position.root(1).z joint_position.middle(1).z joint_position.middle(2).z joint_position.middle(3).z joint_position.middle(4).z];
            middle_finger.c = [(joint_position.root(1).z+250)/500 ...
                              (joint_position.middle(1).z+250)/500 ...
                              (joint_position.middle(2).z+250)/500 ...
                              (joint_position.middle(3).z+250)/500 ...
                              (joint_position.middle(4).z+250)/500];
            surface('XData',[middle_finger.x(:) middle_finger.x(:)],...
                    'YData',[middle_finger.y(:) middle_finger.y(:)],...
                    'ZData',[middle_finger.z(:) middle_finger.z(:)],...
                    'CData',[middle_finger.c(:) middle_finger.c(:)],...
                    'FaceColor','none','EdgeColor','flat','Marker','none','linewidth',15);
            %Third
            third_finger.x = [joint_position.root(1).x joint_position.third(1).x joint_position.third(2).x joint_position.third(3).x joint_position.third(4).x];
            third_finger.y = [joint_position.root(1).y joint_position.third(1).y joint_position.third(2).y joint_position.third(3).y joint_position.third(4).y];
            third_finger.z = [joint_position.root(1).z joint_position.third(1).z joint_position.third(2).z joint_position.third(3).z joint_position.third(4).z];
            third_finger.c = [(joint_position.root(1).z+250)/500 ...
                              (joint_position.third(1).z+250)/500 ...
                              (joint_position.third(2).z+250)/500 ...
                              (joint_position.third(3).z+250)/500 ...
                              (joint_position.third(4).z+250)/500];
            surface('XData',[third_finger.x(:) third_finger.x(:)],...
                    'YData',[third_finger.y(:) third_finger.y(:)],...
                    'ZData',[third_finger.z(:) third_finger.z(:)],...
                    'CData',[third_finger.c(:) third_finger.c(:)],...
                    'FaceColor','none','EdgeColor','flat','Marker','none','linewidth',15);
            % Little
            little_finger.x = [joint_position.root(1).x joint_position.little(1).x joint_position.little(2).x joint_position.little(3).x joint_position.little(4).x];
            little_finger.y = [joint_position.root(1).y joint_position.little(1).y joint_position.little(2).y joint_position.little(3).y joint_position.little(4).y];
            little_finger.z = [joint_position.root(1).z joint_position.little(1).z joint_position.little(2).z joint_position.little(3).z joint_position.little(4).z];
            little_finger.c = [(joint_position.root(1).z+250)/500 ...
                              (joint_position.little(1).z+250)/500 ...
                              (joint_position.little(2).z+250)/500 ...
                              (joint_position.little(3).z+250)/500 ...
                              (joint_position.little(4).z+250)/500];
            surface('XData',[little_finger.x(:) little_finger.x(:)],...
                    'YData',[little_finger.y(:) little_finger.y(:)],...
                    'ZData',[little_finger.z(:) little_finger.z(:)],...
                    'CData',[little_finger.c(:) little_finger.c(:)],...
                    'FaceColor','none','EdgeColor','flat','Marker','none','linewidth',15);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            whitebg(1,[0 0 0]);
            set(gca,'xtick',[],'ytick',[]);
            set(gca,'xcolor','k','ycolor','k');
            set(gca,'clim',[0 1]);
            view(0,90);
            
            depthFrame = getframe;
            depth = depthFrame.cdata(:,:,1);
            % _____________________________________________________________
        end
    end
end
