import create from "zustand";
import produce from 'immer';

import useRosStore from './RosStore';

const immer = (config) => (set, get, api) =>
  config((fn) => set(produce(fn)), get, api);

const store = (set,get) => ({
    // The frame specifies the expert (color) frame
    messages: ['default message 2','default message 1'],
    addMessage: (message) => set(state=>{
        state.messages = [message,...state.messages]
    }),
    sendMessage: (msg) => useRosStore.getState().listenerTopic.publish({data:msg})
});

const useAppStore = create(immer(store));

export default useAppStore;