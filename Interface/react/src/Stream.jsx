import React from 'react';
import useAppStore from './AppStore';
import useImageStore from './ImageStore';
import { Grommet, List, Stack,Button, TextInput, Heading, Text, Card, CardHeader, CardBody, Grid, Box,CheckBox } from 'grommet';


export const Stream = (props) => {
    var imagedata = useImageStore(state=>state.imagedata);
    const [maxWidth,maxHeight,resizeWindow] = useAppStore(state=>[state.imageWidth, state.imageHeight,state.resizeWindow])
    React.useEffect(() => {
        function handleResize() {
          resizeWindow()
        }
        window.addEventListener('resize', handleResize)
      })
    return (
        <img
            id='livestream'
            alt='Authoring-Robot Stream'
            src={imagedata}
            style={{
                width: maxWidth+'px',
                height: maxHeight+'px',
                //height: window.innerHeight,
                objectFix: 'cover',
                zIndex: 1,
            }}
        />
    );
}