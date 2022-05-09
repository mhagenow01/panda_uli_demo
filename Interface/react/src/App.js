import React, {useState} from 'react';
// import {List,Button,Checkbox,Card,Space,Input} from 'antd';
// import {EnterOutlined} from '@ant-design/icons';
import {Stream} from './Stream';
import {RPanel} from './RPanel';
import {Canvas} from './Canvas';
import {TopBar} from './TopBar';
import {BotBar} from './BotBar';
import {View} from './3Dview';
import { Grommet, List, Stack,Button, TextInput, Card, Grid, Box,CheckBox,Tabs, Tab } from 'grommet';
import styled from "styled-components";
import useAppStore from './AppStore';
import useRosStore from './RosStore';


function App() {
  const [show3D] = useRosStore(state=>([state.show3D])) 
  return (
    <Box height='100vh' width='100vw' background='light-1'>
      <Grid
        rows={['10vh', '80vh','10vh']}
        columns={['80vw', '20vw']}
        gap="none"
        height='100vh' 
        width='100vw'
        areas={[
          {name: 'top', start:[0,0], end: [1,0]},
          {name: 'video', start:[0,1], end: [0,1]},
          { name: 'rpanel', start: [1, 1], end: [1, 1] },
          {name: 'bot', start:[0,2], end: [1,2]},
        ]}
      >
        <Box gridArea="top" background="brand">
          <TopBar/>
        </Box>
        <Stack>
        <Tabs gridArea="video" onActive={(val) => {
          if(val ==1)
          setTimeout(function() { //Start the timer
            show3D()
            }.bind(this), 10)
          
          }}>
          <Tab title="Camera View">
            <Stack>
              <Stream />
              <Canvas />
            </Stack>
          </Tab>
          <Tab title="3D View">
            <View/>
          </Tab>
        </Tabs>
        </Stack>
        <RPanel gridArea='rpanel'/>
        <Box gridArea="bot" background="brand">
          <BotBar/>
        </Box>
      </Grid>
      </Box>
    );
}

export default App;
