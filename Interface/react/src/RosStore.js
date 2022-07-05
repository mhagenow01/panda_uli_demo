import create from "zustand";
import ROSLIB from 'roslib';
import {Viewer, Grid, UrdfClient,PointCloud2} from 'ros3d';
import * as ROS3D from 'ros3d';

import useAppStore from './AppStore';

const store = (set) => ({
    //url: 'ws://localhost:9090',
    url: 'ws://192.168.3.5:9090',
    // SetURL resets ROS
    setUrl: (url) => set((_)=>({url:url,connection:'disconnected'})),
    ros: null,
    connection: 'disconnected',
    talkerTopic: null,
    commandTopic: null,
    imageTopic: null,
    pathTopic: null,
    onConnection: () => set({connection:'connected'}),
    onError: () => set({connection:'disconnected'}),
    onClose: () => set({connection:'disconnected'}),
    show3D: () => set((state)=>{
        const ros = state.ros
        let viewer = new Viewer({
            divID : 'urdf',
            width : useAppStore.getState().imageWidth,
            height : useAppStore.getState().imageHeight,
            antialias : true,
            background : "#f8f8f8"

        });

        viewer.addObject(new Grid());
        var tfClient = new ROSLIB.TFClient({
            ros : ros,
            angularThres : 0.01,
            transThres : 0.01,
            rate : 10.0,
            fixedFrame : '/panda_link0'
          });
      
        //   Setup the marker client.
        var markerClient = new ROS3D.MarkerArrayClient({
            ros : ros,
            tfClient : tfClient,
            topic : '/reachabilitymap',
            path: process.env.PUBLIC_URL + 'assets/',
            rootObject : viewer.scene
        });
        var markerClient2 = new ROS3D.MarkerArrayClient({
            ros : ros,
            tfClient : tfClient,
            topic : '/meshes',
            path: process.env.PUBLIC_URL + 'assets/',
            rootObject : viewer.scene
        });

        var urdfClient = new UrdfClient({
            ros : ros,
            tfClient : tfClient,
            path : process.env.PUBLIC_URL + 'assets/',
            rootObject : viewer.scene,
        });

        var _rgb_lut = new Float32Array(256);
        for (var i = 0; i < 256; i++) {
          _rgb_lut[i] = i / 255.0
        }
        var _floatColor = new Float32Array(1);
        let colormap = undefined;
        colormap = function(x) {
            _floatColor[0] = x;
            const intColorArray = new Int32Array(_floatColor.buffer);
            const intColor = intColorArray[0];

            return {
                r: _rgb_lut[(intColor >> 16) & 0xff],
                g: _rgb_lut[(intColor >> 8) & 0xff],
                b: _rgb_lut[(intColor) & 0xff],              
                a: 1.0
            };
        }
        var pointCloud2 = new PointCloud2({
            ros : ros,
            tfClient : tfClient,
            topic : '/filtered_cloud',
            max_pts : 3245728,
            material: { size: 0.02},
            colorsrc: 'rgb',
            colormap: colormap,
            rootObject : viewer.scene
          });
    }),
    connect: () => set((state)=>{
        if(state.connection==="connected")
            return
        const ros = new ROSLIB.Ros({url:state.url});
        ros.on('connection', state.onConnection);
        ros.on('error', state.onError);
        ros.on('close', state.onClose);

        const talkerTopic = new ROSLIB.Topic({
            ros: ros,
            name: 'ros_server/talker',
            messageType: 'std_msgs/String'
        });

        const eventTopic = new ROSLIB.Topic({
            ros: ros,
            name: 'interaction_events',
            messageType: 'std_msgs/String'
        });
        
        const objTopic = new ROSLIB.Topic({
            ros: ros,
            name: 'getObjPose',
            messageType: 'std_msgs/String'
        })

        const modelTopic = new ROSLIB.Topic({
            ros: ros,
            name: 'executeModel',
            messageType: 'std_msgs/String'
        });

        const commandTopic = new ROSLIB.Topic({
            ros: ros,
            name: 'ui/commands',
            messageType: 'std_msgs/String'
        });

        const joyTopic = new ROSLIB.Topic({
            ros: ros,
            name: 'ui/joy',
            messageType: 'sensor_msgs/Joy'
        });

        const paramTopic = new ROSLIB.Topic({
            ros: ros,
            name: 'ui/parameters',
            messageType: 'std_msgs/String'
        });

        const rvizTopic = new ROSLIB.Topic({
            ros: ros,
            name: 'rviz_triggers',
            messageType: 'std_msgs/String'
        });

        const setParamTopic = new ROSLIB.Topic({
            ros: ros,
            name: 'ui/set_parameters',
            messageType: 'std_msgs/String'
        });

		const pathTopic = new ROSLIB.Topic({
			ros: ros,
			//name: 'camera/image_raw/compressed',
			name: '/ui/path',
			messageType: 'std_msgs/String',
		});

		const reachTopic = new ROSLIB.Topic({
			ros: ros,
			//name: 'camera/image_raw/compressed',
			name: '/ui/reach',
			messageType: 'std_msgs/String',
		});
		const feedbackTopic = new ROSLIB.Topic({
			ros: ros,
			name: '/ui/feedback',
			messageType: 'std_msgs/String',
		});

        pathTopic.subscribe(function (message) {
			useAppStore.getState().setPath(message.data.split(';'));
		});
        reachTopic.subscribe(function (message) {
            var pointType = message.data.split(':')[0]
            if(pointType === "good")
			    useAppStore.getState().setGood(message.data.split(':')[1].split(';'));
            if(pointType === "bad")
                useAppStore.getState().setBad(message.data.split(':')[1].split(';'));
		});

		const imageTopic = new ROSLIB.Topic({
			ros: ros,
			//name: 'camera/image_raw/compressed',
			name: 'k4a/rgb/image_raw/compressed',
			messageType: 'sensor_msgs/CompressedImage',
		});
         
        imageTopic.subscribe(function (message) {
			useAppStore.getState().setImage('data:image/jpg;base64,' + message.data);
			// console.log(imagedata);
			// document.getElementById('livestream').src = imagedata;
		});

		const robotTopic = new ROSLIB.Topic({
			ros: ros,
			name: '/ui/robot_state',
			messageType: 'std_msgs/String',
		});

		const paperTopic = new ROSLIB.Topic({
			ros: ros,
			name: '/panda/sandpaper_health',
			messageType: 'std_msgs/Float64',
		});

        feedbackTopic.subscribe((msg)=>{useAppStore.getState().setFeedback(msg.data)});
        robotTopic.subscribe((msg)=>useAppStore.getState().setRobotStatus(msg.data));
        paperTopic.subscribe((msg)=>useAppStore.getState().setPaperStatus(msg.data));
        rvizTopic.subscribe((msg)=>useAppStore.getState().receivedRviz(msg.data));
        talkerTopic.subscribe((msg)=>useAppStore.getState().addMessage(msg.data));
        eventTopic.subscribe((msg)=>useAppStore.getState().setCanvasOpacity(1));
        setParamTopic.subscribe((msg)=>useAppStore.getState().setParameters(msg.data));
        ros.connect();
        useAppStore.getState().resizeWindow();

        return {
            url:state.url,
            connection:'connecting',
            ros:ros,
            talkerTopic:talkerTopic,
            commandTopic:commandTopic,
            imageTopic:imageTopic,
            paramTopic:paramTopic,
            joyTopic:joyTopic,
            robotTopic:robotTopic,
            rvizTopic:rvizTopic,
            objTopic:objTopic,
            modelTopic:modelTopic,
        };
    })
});

const useRosStore = create(store);

//useRosStore.getState().setUrl('ws://localhost:9090');
useRosStore.getState().setUrl('ws://192.168.3.5:9090');
export default useRosStore;