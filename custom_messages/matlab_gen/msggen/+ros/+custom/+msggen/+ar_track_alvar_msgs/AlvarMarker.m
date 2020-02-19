classdef AlvarMarker < ros.Message
    %AlvarMarker MATLAB implementation of ar_track_alvar_msgs/AlvarMarker
    %   This class was automatically generated by
    %   ros.msg.internal.gen.MessageClassGenerator.
    
    %   Copyright 2014-2020 The MathWorks, Inc.
    
    %#ok<*INUSD>
    
    properties (Constant)
        MessageType = 'ar_track_alvar_msgs/AlvarMarker' % The ROS message type
    end
    
    properties (Constant, Hidden)
        MD5Checksum = 'ef2b6ad42bcb18e16b22fefb5c0fb85f' % The MD5 Checksum of the message definition
    end
    
    properties (Access = protected)
        JavaMessage % The Java message object
    end
    
    properties (Constant, Access = protected)
        GeometryMsgsPoseStampedClass = ros.msg.internal.MessageFactory.getClassForType('geometry_msgs/PoseStamped') % Dispatch to MATLAB class for message type geometry_msgs/PoseStamped
        StdMsgsHeaderClass = ros.msg.internal.MessageFactory.getClassForType('std_msgs/Header') % Dispatch to MATLAB class for message type std_msgs/Header
    end
    
    properties (Dependent)
        Header
        Id
        Confidence
        Pose
    end
    
    properties (Access = protected)
        Cache = struct('Header', [], 'Pose', []) % The cache for fast data access
    end
    
    properties (Constant, Hidden)
        PropertyList = {'Confidence', 'Header', 'Id', 'Pose'} % List of non-constant message properties
        ROSPropertyList = {'confidence', 'header', 'id', 'pose'} % List of non-constant ROS message properties
    end
    
    methods
        function obj = AlvarMarker(msg)
            %AlvarMarker Construct the message object AlvarMarker
            import com.mathworks.toolbox.ros.message.MessageInfo;
            
            % Support default constructor
            if nargin == 0
                obj.JavaMessage = obj.createNewJavaMessage;
                return;
            end
            
            % Construct appropriate empty array
            if isempty(msg)
                obj = obj.empty(0,1);
                return;
            end
            
            % Make scalar construction fast
            if isscalar(msg)
                % Check for correct input class
                if ~MessageInfo.compareTypes(msg(1), obj.MessageType)
                    error(message('ros:mlros:message:NoTypeMatch', obj.MessageType, ...
                        char(MessageInfo.getType(msg(1))) ));
                end
                obj.JavaMessage = msg(1);
                return;
            end
            
            % Check that this is a vector of scalar messages. Since this
            % is an object array, use arrayfun to verify.
            if ~all(arrayfun(@isscalar, msg))
                error(message('ros:mlros:message:MessageArraySizeError'));
            end
            
            % Check that all messages in the array have the correct type
            if ~all(arrayfun(@(x) MessageInfo.compareTypes(x, obj.MessageType), msg))
                error(message('ros:mlros:message:NoTypeMatchArray', obj.MessageType));
            end
            
            % Construct array of objects if necessary
            objType = class(obj);
            for i = 1:length(msg)
                obj(i,1) = feval(objType, msg(i)); %#ok<AGROW>
            end
        end
        
        function header = get.Header(obj)
            %get.Header Get the value for property Header
            if isempty(obj.Cache.Header)
                obj.Cache.Header = feval(obj.StdMsgsHeaderClass, obj.JavaMessage.getHeader);
            end
            header = obj.Cache.Header;
        end
        
        function set.Header(obj, header)
            %set.Header Set the value for property Header
            validateattributes(header, {obj.StdMsgsHeaderClass}, {'nonempty', 'scalar'}, 'AlvarMarker', 'Header');
            
            obj.JavaMessage.setHeader(header.getJavaObject);
            
            % Update cache if necessary
            if ~isempty(obj.Cache.Header)
                obj.Cache.Header.setJavaObject(header.getJavaObject);
            end
        end
        
        function id = get.Id(obj)
            %get.Id Get the value for property Id
            id = typecast(int32(obj.JavaMessage.getId), 'uint32');
        end
        
        function set.Id(obj, id)
            %set.Id Set the value for property Id
            validateattributes(id, {'numeric'}, {'nonempty', 'scalar'}, 'AlvarMarker', 'Id');
            
            obj.JavaMessage.setId(id);
        end
        
        function confidence = get.Confidence(obj)
            %get.Confidence Get the value for property Confidence
            confidence = typecast(int32(obj.JavaMessage.getConfidence), 'uint32');
        end
        
        function set.Confidence(obj, confidence)
            %set.Confidence Set the value for property Confidence
            validateattributes(confidence, {'numeric'}, {'nonempty', 'scalar'}, 'AlvarMarker', 'Confidence');
            
            obj.JavaMessage.setConfidence(confidence);
        end
        
        function pose = get.Pose(obj)
            %get.Pose Get the value for property Pose
            if isempty(obj.Cache.Pose)
                obj.Cache.Pose = feval(obj.GeometryMsgsPoseStampedClass, obj.JavaMessage.getPose);
            end
            pose = obj.Cache.Pose;
        end
        
        function set.Pose(obj, pose)
            %set.Pose Set the value for property Pose
            validateattributes(pose, {obj.GeometryMsgsPoseStampedClass}, {'nonempty', 'scalar'}, 'AlvarMarker', 'Pose');
            
            obj.JavaMessage.setPose(pose.getJavaObject);
            
            % Update cache if necessary
            if ~isempty(obj.Cache.Pose)
                obj.Cache.Pose.setJavaObject(pose.getJavaObject);
            end
        end
    end
    
    methods (Access = protected)
        function resetCache(obj)
            %resetCache Resets any cached properties
            obj.Cache.Header = [];
            obj.Cache.Pose = [];
        end
        
        function cpObj = copyElement(obj)
            %copyElement Implements deep copy behavior for message
            
            % Call default copy method for shallow copy
            cpObj = copyElement@ros.Message(obj);
            
            % Clear any existing cached properties
            cpObj.resetCache;
            
            % Create a new Java message object
            cpObj.JavaMessage = obj.createNewJavaMessage;
            
            % Iterate over all primitive properties
            cpObj.Id = obj.Id;
            cpObj.Confidence = obj.Confidence;
            
            % Recursively copy compound properties
            cpObj.Header = copy(obj.Header);
            cpObj.Pose = copy(obj.Pose);
        end
        
        function reload(obj, strObj)
            %reload Called by loadobj to assign properties
            obj.Id = strObj.Id;
            obj.Confidence = strObj.Confidence;
            obj.Header = feval([obj.StdMsgsHeaderClass '.loadobj'], strObj.Header);
            obj.Pose = feval([obj.GeometryMsgsPoseStampedClass '.loadobj'], strObj.Pose);
        end
    end
    
    methods (Access = ?ros.Message)
        function strObj = saveobj(obj)
            %saveobj Implements saving of message to MAT file
            
            % Return an empty element if object array is empty
            if isempty(obj)
                strObj = struct.empty;
                return
            end
            
            strObj.Id = obj.Id;
            strObj.Confidence = obj.Confidence;
            strObj.Header = saveobj(obj.Header);
            strObj.Pose = saveobj(obj.Pose);
        end
    end
    
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
            %loadobj Implements loading of message from MAT file
            
            % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.custom.msggen.ar_track_alvar_msgs.AlvarMarker.empty(0,1);
                return
            end
            
            % Create an empty message object
            obj = ros.custom.msggen.ar_track_alvar_msgs.AlvarMarker;
            obj.reload(strObj);
        end
    end
end
