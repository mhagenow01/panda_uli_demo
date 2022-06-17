import React from 'react';
import useAppStore from './AppStore';
import useRosStore from './RosStore';
import {ParameterModule} from './ParameterModule';
import { Grommet, List, Stack,Button, TextInput, Heading, Text, Card, CardHeader, CardBody, Grid, Box,CheckBox } from 'grommet';

export const RPanel = (props) => {  
  const [connect, connection, url,imageTopic,tmpSub] = useRosStore(state=>([state.connect, state.connection, state.url,state.imageTopic,state.tmpSub])) 
  const parameters = useAppStore(state=>state.parameters)
  
  const [messages, addMessage, sendMessage, get_path,publishStates,setCanvasOpacity,robotStatus] = useAppStore(state=>([state.messages,state.addMessage,state.sendMessage,state.get_path,state.publishStates,state.setCanvasOpacity,state.robotStatus]))

    return (
      
      <Box alignContent="stretch" align="stretch" justify="between" border={{ color: 'black', size: 'medium' }}>
        <Box basis="6vh" fill="horizontal" alignContent='center' justify="start" direction="row" gap="medium">
            <Box alignSelf={"center"} background={{color:connection==='connected'?"green":"red"}} round={"large"} width="4vh" height={"4vh"}/>
            <Button onClick={()=>{
                connect();
                publishStates();
              }}>
              <Box><Text size="2vh">{connection==='connected'?"Connected":"Connect"} to {url}</Text></Box>
            </Button>
        </Box>
        <Box basis="6vh" fill="horizontal" alignContent='center' justify="start" direction="row" gap="medium">
            <Box alignSelf={"center"} background={{color:robotStatus}} round={"large"} width="4vh" height={"4vh"}/>
            <Box justify={"center"} ><Text justify={"center"} size="2vh">Robot state</Text></Box>
        </Box>

        <Box >
          <Card background="light-1">
            <CardHeader pad="small" background="light-2">
                <Text alignSelf="center" color="#9b0000" size="3vh">Properties</Text>
            </CardHeader>
            <CardBody pad="small">
            {parameters.map((button,idx) => (
              <ParameterModule param={button} key={idx} idx={idx}/>
            ))
            }
            </CardBody>
          </Card>
        </Box>
        <Box justify="center" direction="row" gap="medium">
          <Button size="xsmall" alignSelf="center" label=<Box><Text size="3vh">Send</Text></Box> 
            onClick={() => {
              get_path();
              //tmpSub.displayCloud();
            }}
            />

          <Button size="xsmall" label=<Box><Text size="3vh">Execute</Text></Box> 
          onClick={() => {
            sendMessage("execute");
            setCanvasOpacity(0);
          }} />
        </Box> 
      </Box>
    )
}