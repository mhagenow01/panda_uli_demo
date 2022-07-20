import React, {useState,useEffect} from 'react';
// import {List,Button,Checkbox,Card,Space,Input} from 'antd';
// import {EnterOutlined} from '@ant-design/icons';
import {Stream} from './Stream';
import {RPanel} from './RPanel';
import {Canvas} from './Canvas';
import {TopBar} from './TopBar';
import {BotBar} from './BotBar';
import {View} from './3Dview';
import Gamepad from 'react-gamepad'
import { Grommet, List, Stack,Button, TextInput, Card, Grid, Box, Text, CheckBox,Tabs, Tab, Layer } from 'grommet';
import styled from "styled-components";
import useAppStore from './AppStore';
import useRosStore from './RosStore';

function App() {
  
  const [show3D] = useRosStore(state=>([state.show3D])) 
  const [feedback,rvizMode,setRvizMode] = useAppStore(state=>([state.feedback,state.rvizMode,state.setRvizMode])) 
  const primaryColor="#9b0000"
  const theme = {
    name: 'UW-sanding',
    //full: true,
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
          width: "4px"
        },
        default: {
          background:{
            color: "#c5050c"
          }
        },
        color: "#adadad"

    },
    text: {
      textAlign:"center",
      alignSelf:"center",
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
            borderRadius: 9,
            padding: 6
        }
    },
    tabs: {
        gap: 'medium',
        header: {
            extend: {padding: 4}
        },
        panel: { padding: 0},
        extend: {padding: 3}
    },

  }

  return (  
    <Grommet
    full={true}
      theme={theme}
      >
      <Box height='100vh' width='100vw' direction="row" background="#9b0000">      
        {/* <GamepadHandle/> */}
        <Tabs width="70vw" justify="start" activeIndex={rvizMode} onActive={(val) => {
          setRvizMode(val)
          }}>
          <Tab title={<Box><Text size="2vh">Camera View</Text></Box>}>
            <Stack>
              <Stream />
              <Canvas />
            </Stack>
          </Tab>
          <Tab title={<Box><Text size="2vh">3D View</Text></Box>}>
            <View/>
          </Tab>
        </Tabs>
        <RPanel/>
        <Layer responsive={false} plain={true} modal={false} position="top">
          <Box
            radius="small"
            pad="medium"
          >
            <Box
              border={{ color: 'brand', size: 'xsmall' }}
              background={{
                color: "light-3",
                opacity: .9
              }}
              round="medium"
              width="medium"
              pad="medium"
              alignContent='center'
              opacity={.4}
              hidden={feedback ===""}
            >
              <Text size="4vh" alignSelf='center'>{feedback}</Text>
          </Box>
          </Box>
        </Layer>
      </Box>
    </Grommet>
    );
}

export default App;
