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
    corners: [...Array(4)].map((_, i) => ({
      id: i.toString(),
      x: Math.random() * 500,
      y: Math.random() * 500,
      isDragging: false,
    })),
    addMessage: (message) => set(state=>{
        state.messages = [message,...state.messages]
    }),
    sendMessage: () => set(state =>{
      useRosStore.getState().commandTopic.publish({data:state.corners.map((item) => (String(item.x/state.imageWidth)+','+String(item.y/state.imageHeight))).join(';')})
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
      console.log(coords)
      state.path = []
      coords.map((coord) => {
        coord = coord.split(',')
        state.path.push({
          x: parseFloat(coord[0])*state.imageWidth,
          y: parseFloat(coord[1])*state.imageHeight
        });   
      })
      console.log(state.path)
    })
});

const useAppStore = create(immer(store));

export default useAppStore;