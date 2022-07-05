import create from "zustand";
import produce from 'immer';
import ROSLIB from 'roslib';

import useRosStore from './RosStore';

const immer = (config) => (set, get, api) =>
  config((fn) => set(produce(fn)), get, api);

const store = (set,get) => ({
    // The frame specifies the expert (color) frame
    messages: ['default message 2','default message 1'],
    robotStatus: "grey",
    paperStatus: 0.0,
    gamepads: [1,1],
    path: [{x:0,y:0}],
    good: [{x:0,y:0}],
    bad: [{x:0,y:0}],
    imageWidth: 1073,//1200
    imageHeight: 805,//900
    imagedata: "",
    knownWorkflow: 0,
    canvasOpacity: 1.,
    scanning: false,
    computed_traj: false,
    computing: false,
    parameters: [ {type: "select", id:"passes", label:"# of passes", value:"2", options:["2","3","4","5"]},
                  {type: "select", id:"direction", label:"Orientation", value:"horizontal", options:["horizontal","vertical"]},
                  {type: "select", id:"material", label:"Material", value:"Composite", options:["Composite","Metal","Paint"]},
                  {type: "slider", id:"force", label:"Force", value:1,min:0,max:10,unit: "N"},
                  {type: "slider", id:"speed", label:"Feed Rate", value:2,min:1,max:20, unit: "mm/s"},
                  {type: "slider", id:"pitch", label:"Pitch angle", value:0,min:-10,max:10, unit: "deg"},
                  {type: "select", id:"tool", label:"Tool", value:"pandaOrbital", options:["pandaOrbital","panda_gripper"]}
                ],
    configDetails: "",
    corners: [...Array(4)].map((_, i) => ({
      id: i.toString(),
      x: 100 + (i%4)* 100,
      y: 100,
      isDragging: false,
    })),
    setScanning: (val) => set(state=>{
      state.scanning= val
    }),
    setComputedTraj: (val) => set(state=>{
      state.computed_traj= val
    }),
    setComputing: (val) => set(state=>{
      state.computing= val
    }),
    setKnownWorkflow: (val) => set(state =>{
      state.knownWorkflow = val
      state.canvasOpacity=1-val
    }),
    setPaperStatus: (val) => set(state =>{
      state.paperStatus = val
    }),
    resizeWindow: () => set(state=>{
      console.log('resized to: ', window.innerWidth, 'x', window.innerHeight)
      var w = window.innerWidth*.98
      var h = window.innerHeight*.953
      //var k=16./9.
      var k=4./3.
      if( w > k*h)
        w = k*h
      else
        h = w/k
      state.imageWidth = w
      state.imageHeight = h
    }),
    setRobotStatus: (msg) => set(state=>{
      state.robotStatus = msg 
    }),
    receivedRviz: (msg) => set(state=>{
      if(msg==="scanningdone")
        state.scanning=false
      if(msg==="computetrajdone"){
        state.computing=false
        state.computed_traj=true
      }
      if(msg==="execdone"){
        state.computing=false
        state.computed_traj=false
      }
      
    }),
    setGamepads: (msg) => set(state=>{
      // https://answers.ros.org/question/284741/seq-or-time-stamp-for-publishing-a-message/
      try {
        var currentTime = new Date();
        var secs = Math.floor(currentTime.getTime()/1000);
        var nsecs = Math.round(1000000000*(currentTime.getTime()/1000-secs));
        var buttons = []
        msg[0].buttons.map((button) => {
          buttons.push(button.value)  
        })
        var joy_msg = new ROSLIB.Message({
          header : {
            seq : 0,
            stamp : {
              secs: Math.floor(currentTime.getTime()/1000),
              nsecs: Math.round(1000000000*(currentTime.getTime()/1000-secs))
            },
            frame_id: "/dev/input/js0"
          },
          axes: msg[0].axes,
          buttons: buttons
        });

        useRosStore.getState().joyTopic.publish(joy_msg)
      }
      catch (err){
        console.log(err)
      }
      state.gamepads = msg
    }),
    addMessage: (message) => set(state=>{
        state.messages = [message,...state.messages]
    }),
    spline: () => set(state =>{
      useRosStore.getState().commandTopic.publish({data:"spline:"+state.corners.map((item) => (String(item.x/state.imageWidth)+','+String(item.y/state.imageHeight))).join(';')})
      //useRosStore.getState().commandTopic.publish({data:state.corners.map(item => {String(item.x)+','+String(item.y)}).join(';')})
    }),
    get_path: () => set(state =>{
      useRosStore.getState().commandTopic.publish({data:"get_path"})
    }),
    sendMessage: (msg) => set(state =>{
      useRosStore.getState().commandTopic.publish({data:msg})
      //useRosStore.getState().commandTopic.publish({data:state.corners.map(item => {String(item.x)+','+String(item.y)}).join(';')})
    }),
    sendTrigger: (msg) => set(state =>{
      useRosStore.getState().rvizTopic.publish({data:msg})
    sendObject: (msg) => set(state =>{
      useRosStore.getState().objTopic.publish({data:msg})
    }),
    sendModel: (msg) => set(state =>{
      useRosStore.getState().modelTopic.publish({data:msg})
    }),
    setImage: (msg) => set(state=>{
      state.imagedata = msg
    }),
    setCorner: (i,x,y) => set(state=>{
      state.corners[i].x=x
      state.corners[i].y=y
    }),
    setImageWidth: (w) => set(state=>{
      state.imageWidth = w
    }),
    setImageHeight: (h) => set(state=>{
      state.imageHeight = h
    }),
    setPath: (coords) => set(state=>{
      state.path = []
      coords.map((coord) => {
        coord = coord.split(',')
        state.path.push({
          x: parseFloat(coord[0])*state.imageWidth,
          y: parseFloat(coord[1])*state.imageHeight
        });   
      })
    }),
    setGood: (coords) => set(state=>{
      state.good = []
      coords.map((coord) => {
        coord = coord.split(',')
        state.good.push({
          x: parseFloat(coord[0])*state.imageWidth,
          y: parseFloat(coord[1])*state.imageHeight
        });   
      })
      console.log(state.good)
    }),
    setBad: (coords) => set(state=>{
      state.bad = []
      coords.map((coord) => {
        coord = coord.split(',')
        state.bad.push({
          x: parseFloat(coord[0])*state.imageWidth,
          y: parseFloat(coord[1])*state.imageHeight
        });   
      })
    }),
    setParameter: (idx,value) => set(state=>{
       state.parameters[idx].value = value
       useRosStore.getState().paramTopic.publish({data:JSON.stringify(state.parameters)})
     }),
     publishStates: () => set(state=>{
      useRosStore.getState().paramTopic.publish({data:JSON.stringify(state.parameters)})
    }),
     setParameters: (param) => set(state=>{
       state.parameters=JSON.parse(param)}),
     setCanvasOpacity: (val) => set(state=>{
         state.canvasOpacity=val})
});

const useAppStore = create(immer(store));

export default useAppStore;