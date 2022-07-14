import create from "zustand";
import produce from 'immer';

const immer = (config) => (set, get, api) =>
  config((fn) => set(produce(fn)), get, api);

const store = (set,get) => ({
    // The frame specifies the expert (color) frame
    imagedata: "",
    setImage: (msg) => set(state=>{
      console.log("Setting image")
      state.imagedata = msg
    }),
});

const useImageStore = create(immer(store));

export default useImageStore;