import create from "zustand";
import ROSLIB from 'roslib';

import useAppStore from './AppStore';

const store = (set) => ({
    url: 'ws://localhost:9090',
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
    connect: () => set((state)=>{
        const ros = new ROSLIB.Ros({url:state.url});
        ros.on('connection', state.onConnection);
        ros.on('error', state.onError);
        ros.on('close', state.onClose);

        const talkerTopic = new ROSLIB.Topic({
            ros: ros,
            name: 'ros_server/talker',
            messageType: 'std_msgs/String'
        });

        const commandTopic = new ROSLIB.Topic({
            ros: ros,
            name: 'ui/commands',
            messageType: 'std_msgs/String'
        });

        const paramTopic = new ROSLIB.Topic({
            ros: ros,
            name: 'ui/parameters',
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

        pathTopic.subscribe(function (message) {
			useAppStore.getState().setPath(message.data.split(';'));
		});

		const imageTopic = new ROSLIB.Topic({
			ros: ros,
			//name: 'camera/image_raw/compressed',
			name: '/rgb/image_raw/compressed',
			messageType: 'sensor_msgs/CompressedImage',
		});

        imageTopic.subscribe(function (message) {
			useAppStore.getState().setImage('data:image/jpg;base64,' + message.data);
			// console.log(imagedata);
			// document.getElementById('livestream').src = imagedata;
		});

        talkerTopic.subscribe((msg)=>useAppStore.getState().addMessage(msg.data));
        setParamTopic.subscribe((msg)=>useAppStore.getState().setParameters(msg.data));

        ros.connect();

        return {
            url:state.url,
            connection:'connecting',
            ros:ros,
            talkerTopic:talkerTopic,
            commandTopic:commandTopic,
            imageTopic:imageTopic,
            paramTopic:paramTopic,
        };
    })
});

const useRosStore = create(store);

useRosStore.getState().setUrl('ws://localhost:9090');

export default useRosStore;