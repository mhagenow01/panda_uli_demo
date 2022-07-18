import create from "zustand";
import produce from 'immer';
import ROSLIB from 'roslib';

import useRosStore from './RosStore';

const immer = (config) => (set, get, api) =>
  config((fn) => set(produce(fn)), get, api);

const store = (set,get) => ({
    // The frame specifies the expert (color) frame
    messages: ['default message 2','default message 1'],
    pathComputed: false,
    reachComputed: false,
    robotStatus: "grey",
    paperStatus: 0.0,
    paperChange: "Change paper",
    timer: 0,
    targetOpacity:0.,
    rvizMode:0,
    gamepads: [1,1],
    path: [{x:0,y:0}],
    good: [{x:0,y:0}],
    bad: [{x:0,y:0}],
    imageWidth: 1073,//1200
    imageHeight: 805,//900
    imagedata: "",
    knownWorkflow: 0,
    feedback: "",
    canvasOpacity: 1.,
    scanning: false,
    computed_traj: false,
    computing: false,
    configDetails: "",
    executeState: "Execute",
    corners: [...Array(4)].map((_, i) => ({
      id: i.toString(),
      x: 100 + (i%4)* 100,
      y: 100,
      isDragging: false,
    })),
    setRvizMode: (val) => set(state=>{
      state.rvizMode = val
      if(val ==1)
        setTimeout(function() {useRosStore.getState().show3D()}.bind(this), 10)
    }),
    decreaseTimer: () => set(state=>{
      if(state.timer > 0)
        state.timer -= 1
      else
        state.feedback = ""
    }),
    clearReach: () => set(state=>{
      state.good = []
      state.bad = []
      console.log("clearing")
    }),
    setScanning: (val) => set(state=>{
      state.scanning= val
    }),
    setExecuteState: (val) => set(state=>{
      state.executeState = val
    }),
    setPaperChange: (val) => set(state=>{
      state.paperChange = val
    }),
    setFeedback: (msg) => set(state=>{
      let cmds = msg.split(';')
      state.timer = cmds[0]
      state.feedback= cmds[1]
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
      state.targetOpacity=1-val
    }),
    setPaperStatus: (val) => set(state =>{
      state.paperStatus = val
    }),
    resizeWindow: () => set(state=>{
      console.log('resized to: ', window.innerWidth, 'x', window.innerHeight)
      var w = window.innerWidth*.63
      var h = window.innerHeight*1.03
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
      if(msg !== "green" && state.executeState === "Pause")
        state.executeState = "Resume" 
    }),
    receivedRviz: (msg) => set(state=>{
      if(msg==="scanningdone"){
        state.scanning=false
        state.targetOpacity=1.
      }
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
      // console.log(msg)
      // https://answers.ros.org/question/284741/seq-or-time-stamp-for-publishing-a-message/
      try {
        
        var currentTime = new Date();
        var secs = Math.floor(currentTime.getTime()/1000);
        var nsecs = Math.round(1000000000*(currentTime.getTime()/1000-secs));
        var buttons = []
        msg.buttons.map((button) => {
          buttons.push(button.value)  
        })
        ////state.paperStatus = (msg.axes[0]+1)/2.
        var joy_msg = new ROSLIB.Message({
          header : {
            seq : 0,
            stamp : {
              secs: Math.floor(currentTime.getTime()/1000),
              nsecs: Math.round(1000000000*(currentTime.getTime()/1000-secs))
            },
            frame_id: "/dev/input/js0"
          },
          axes: msg.axes,
          buttons: buttons
        });

        useRosStore.getState().joyTopic.publish(joy_msg)
      }
      catch (err){
        console.log(err)
      }
      //state.gamepads = msg
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
      //useRosStore.getState().commandTopic.publish({data:state.corners.map(item => {String(item.x)+','+String(item.y)}).join(';')})
    }),
    sendMessage: (msg) => set(state =>{
      useRosStore.getState().commandTopic.publish({data:msg})
      //useRosStore.getState().commandTopic.publish({data:state.corners.map(item => {String(item.x)+','+String(item.y)}).join(';')})
    }),
    sendTrigger: (msg) => set(state =>{
      useRosStore.getState().rvizTopic.publish({data:msg})
      //useRosStore.getState().commandTopic.publish({data:state.corners.map(item => {String(item.x)+','+String(item.y)}).join(';')})
    }),
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
    setPathComputed: (val) => set(state=>{
      state.pathComputed = val
    }),
    setReachComputed: (val) => set(state=>{
      state.reachComputed = val
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
      state.pathComputed = true
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
      state.reachComputed = true
    }),
    setCanvasOpacity: (val) => set(state=>{
        state.canvasOpacity=val}),
    setTargetOpacity: (val) => set(state=>{
        state.targetOpacity=val}),
    handleEvent: (val) => set(state=>{
      if(val === "motion_finished"){
        state.canvasOpacity(1) 
        state.executeState = "Execute"
      }
    })
});

const useAppStore = create(immer(store));


var haveEvents = 'GamepadEvent' in window;
var haveWebkitEvents = 'WebKitGamepadEvent' in window;
var controller = [];
var rAF = window.mozRequestAnimationFrame ||
  window.webkitRequestAnimationFrame ||
  window.requestAnimationFrame;

const updateGamepadStore = useAppStore.getState().setGamepads;
function connecthandler(e) {
  addgamepad(e.gamepad);
}
function addgamepad(gamepad) {
  // console.log("New gamepad")
  controller = gamepad; 
  
  rAF(updateStatus);
}

function disconnecthandler(e) {
  removegamepad(e.gamepad);
}

function removegamepad(gamepad) {
  // delete controllers[gamepad.index];
}

function updateStatus() {
  // console.log("update")
  scangamepads();
  updateGamepadStore(controller)
  rAF(updateStatus);
}

function scangamepads() {
  var gamepads = navigator.getGamepads ? navigator.getGamepads() : (navigator.webkitGetGamepads ? navigator.webkitGetGamepads() : []);
  
  controller=gamepads[0]
  
}

if (haveEvents) {
  window.addEventListener("gamepadconnected", connecthandler);
  window.addEventListener("gamepaddisconnected", disconnecthandler);
} else if (haveWebkitEvents) {
  window.addEventListener("webkitgamepadconnected", connecthandler);
  window.addEventListener("webkitgamepaddisconnected", disconnecthandler);
} else {
  setInterval(scangamepads, 500);
}

export default useAppStore;