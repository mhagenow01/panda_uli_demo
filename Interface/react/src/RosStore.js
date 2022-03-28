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
    listenerTopic: null,
    imageTopic: null,
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

        const listenerTopic = new ROSLIB.Topic({
            ros: ros,
            name: 'ros_server/listener',
            messageType: 'std_msgs/String'
        });

		const imageTopic = new ROSLIB.Topic({
			ros: ros,
			name: 'camera/image_raw/compressed',
			//name: '/rviz1/camera1/image/compressed',
			messageType: 'sensor_msgs/CompressedImage',
		});

        imageTopic.subscribe(function (message) {
			var imagedata = 'data:image/jpg;base64,' + message.data;
			// console.log(imagedata);
			document.getElementById('livestream').src = imagedata;
		});

        talkerTopic.subscribe((msg)=>useAppStore.getState().addMessage(msg.data));

        ros.connect();

        return {
            url:state.url,
            connection:'connecting',
            ros:ros,
            talkerTopic:talkerTopic,
            listenerTopic:listenerTopic,
            imageTopic:imageTopic,
        };
    })
});

const useRosStore = create(store);

useRosStore.getState().setUrl('ws://localhost:9090');

export default useRosStore;