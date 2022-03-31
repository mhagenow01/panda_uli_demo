import React from 'react';
import useAppStore from './AppStore';

export const Stream = (props) => {
    var imagedata = useAppStore(state=>state.imagedata);
    const [maxWidth,maxHeight] = useAppStore(state=>[state.imageWidth, state.imageHeight])
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