import create from "zustand";
import produce from 'immer';

import useRosStore from './RosStore';

const immer = (config) => (set, get, api) =>
  config((fn) => set(produce(fn)), get, api);

const store = (set,get) => ({
    // The frame specifies the expert (color) frame
    messages: ['default message 2','default message 1'],
    path: [{x:0,y:0}],
    imageWidth: 1200,
    imageHeight: 900,
    imagedata: "",
    canvasOpacity: 1.,
    parameters: [ {type: "select", label:"Number of passes", value:"2", options:["2","3","4","5"]},
                  {type: "select", label:"Orientation", value:"horizontal", options:["horizontal","vertical"]},
                  {type: "select", label:"Material", value:"Composite", options:["Composite","Metal","Paint"]},
                  {type: "slider", label:"Force (N)", value:1,min:0,max:10},
                  {type: "select", label:"Tool", value:"pandaOrbital", options:["pandaOrbital","panda_gripper"]}
                ],
    configDetails: "",
    corners: [...Array(4)].map((_, i) => ({
      id: i.toString(),
      x: Math.random() * 500,
      y: Math.random() * 500,
      isDragging: false,
    })),
    addMessage: (message) => set(state=>{
        state.messages = [message,...state.messages]
    }),
    sendCoordinates: () => set(state =>{
      useRosStore.getState().commandTopic.publish({data:"spline:"+state.corners.map((item) => (String(item.x/state.imageWidth)+','+String(item.y/state.imageHeight))).join(';')})
      //useRosStore.getState().commandTopic.publish({data:state.corners.map(item => {String(item.x)+','+String(item.y)}).join(';')})
    }),
    sendMessage: (msg) => set(state =>{
      useRosStore.getState().commandTopic.publish({data:msg})
      //useRosStore.getState().commandTopic.publish({data:state.corners.map(item => {String(item.x)+','+String(item.y)}).join(';')})
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