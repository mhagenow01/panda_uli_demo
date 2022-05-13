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
  const primaryColor="#9b0000"
  const theme = {
    name: 'CoFrame',
    rounding: 4,
    defaultMode: 'dark',
    global: {
        colors: {
            brand: primaryColor,
            background: '#000000',
            control: primaryColor
        },
        font: {
            family: "Helvetica"
        },
        focus: {
            border: {
                color: primaryColor
            }
        },
        input: {
            padding: 4,
            extend: { backgroundColor: '#FFFFFF55' }
        },
        // edgeSize: {large: 50, small: 10, medium: 15}
    },
    button: {
        border: {
            radius: "4px"
        }
    },
    radioButton: {
        size: "16px",
        border: { color: '#00000088' }
    },
    checkBox: {
        size: "20px",
        border: { color: '#00000088' },
        color: primaryColor,
        hover: { border: { color: '#00000088' }, }
    },
    textInput: {
        disabled: { opacity: 1 }
    },
    notification: {
        toast: {
            container: {
                elevation: 'none'
            }
        },
        container: {
            border: { color: 'lightgrey' },
            background: {
                color: 'background-front',
            }
        }
    },
    tab: {
        active: {
            background: primaryColor,
            color: 'dark-1'
        },
        background: 'dark-3',
        border: undefined,
        color: 'white',
        hover: {
             background: '#444444',
             color: 'white'
        },
        margin: undefined,
        pad: {
            bottom: undefined,
            horizontal: 'small',
        },
        extend: {
            borderRadius: 4,
            padding: 6
        }
    },
    tabs: {
        gap: 'medium',
        header: {
            extend: {padding: 10}
        },
        panel: { padding: 10 },
        extend: {padding: 10}
    }
}
  return (
    <Grommet
      full theme={theme}
      >
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
        <Box gridArea="top" background="#9b0000">
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
        <Box gridArea="bot" background="#9b0000">
          <BotBar/>
        </Box>
      </Grid>
      </Box>
    </Grommet>
    );
}

export default App;
