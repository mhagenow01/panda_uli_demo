import React, {useState} from 'react';
// import {List,Button,Checkbox,Card,Space,Input} from 'antd';
// import {EnterOutlined} from '@ant-design/icons';
import {Stream} from './Stream';
import {RPanel} from './RPanel';
import {Canvas} from './Canvas';
import { Grommet, List, Stack,Button, TextInput, Card, Grid, Box,CheckBox } from 'grommet';
import styled from "styled-components";
import useAppStore from './AppStore';
import useRosStore from './RosStore';


function App() {
  return (
    <Box height='100vh' width='100vw' background='light-1'>
      <Grid
        rows={['10vh', '90vh']}
        columns={['80vw', '20vw']}
        gap="none"
        height='100vh' 
        width='100vw'
        areas={[
          {name: 'top', start:[0,0], end: [1,0]},
          {name: 'video', start:[0,1], end: [0,1]},
          { name: 'rpanel', start: [1, 1], end: [1, 1] },
        ]}
      >
        <Box gridArea="top" background="brand" />
        <Stack gridArea="video">
          <Stream />
          <Canvas />
        </Stack>
        <RPanel gridArea='rpanel'/>
      </Grid>
      </Box>
    );
}

export default App;
