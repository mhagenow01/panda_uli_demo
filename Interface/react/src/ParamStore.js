import create from "zustand";
import produce from 'immer';
import ROSLIB from 'roslib';

import useRosStore from './RosStore';


const immer = (config) => (set, get, api) =>
  config((fn) => set(produce(fn)), get, api);

const store = (set,get) => ({
    parameters: [ {type: "select", id:"passes", label:"# of passes", value:"2", options:["2","3","4","5","6","7"]},
                  {type: "select", id:"direction", label:"Orientation", value:"Horizontal", options:["Horizontal","Vertical"]},
                  {type: "select", id:"pattern", label:"Pattern", value:"None", options:["None","Circles"]},
                  {type: "slider", id:"force", label:"Force", value:7,min:0,max:20,unit: "N"},
                  {type: "slider", id:"speed", label:"Feed Rate", value:10,min:1,max:60, unit: "mm/s"},
                  {type: "slider", id:"pitch", label:"Pitch angle", value:0,min:-10,max:10, unit: "deg"},
                  {type: "select", id:"tool", label:"Tool", value:"pandaOrbital", options:["pandaOrbital","panda_gripper"]}
                ],
    setParameter: (idx,value) => set(state=>{
       state.parameters[idx].value = value
       useRosStore.getState().paramTopic.publish({data:JSON.stringify(state.parameters)})
     }),
     publishStates: () => set(state=>{
      useRosStore.getState().paramTopic.publish({data:JSON.stringify(state.parameters)})
     })
});

const useParamStore = create(immer(store));

export default useParamStore;