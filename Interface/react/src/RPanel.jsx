import React, {useEffect} from 'react';
import useAppStore from './AppStore';
import useRosStore from './RosStore';
import {ParameterModule} from './ParameterModule';
import { Grommet, List, Stack,Button, Collapsible, Tabs, Tab, TextInput, Heading, Text, Card, CardHeader, CardBody, Grid, Box,CheckBox } from 'grommet';

//https://dev.to/nibble/what-is-uselayouteffect-hook-and-when-do-you-use-it-3lan
function d2hex(number)
{
  let s=parseInt(number*255).toString(16)
  if (s.length == 1)
    s="0"+s
  return s
}

export const RPanel = (props) => {  
  const [connect, connection, url,imageTopic,tmpSub] = useRosStore(state=>([state.connect, state.connection, state.url,state.imageTopic,state.tmpSub])) 
  const [parameters,scanning,computed_traj,computing,setScanning,setComputed_traj,setComputing,setKnownWorkflow] = useAppStore(state=>[state.parameters,state.scanning,state.computed_traj,state.computing,state.setScanning,state.setComputed_traj,state.setComputing,state.setKnownWorkflow])
  const [messages, addMessage, sendTrigger, sendMessage, get_path,publishStates,setCanvasOpacity,robotStatus,paperStatus] = useAppStore(state=>([state.messages,state.addMessage,state.sendTrigger,state.sendMessage,state.get_path,state.publishStates,state.setCanvasOpacity,state.robotStatus,state.paperStatus]))
  const [sendObject,sendModel,decreaseTimer] = useAppStore(state=>([state.sendObject,state.sendModel,state.decreaseTimer]))
  const [open, setOpen] = React.useState(true);

  useEffect(() => {
    setInterval(() => {
      decreaseTimer()
    }, 500)
  }, []);
  useEffect(() => {
    setInterval(() => {
      connect()
    }, 1000)
  }, []);
    return (
      <Box background="light-2" 
        round="medium"
        direction="row" justify="end" height="100vh">
        <Button color="light-2" alignSelf="stretch" basis="small" onClick={() => setOpen(!open)} label={open?">":"<"} />
        <Collapsible direction="horizontal" open={open}>
          <Box direction='column' gap="xxsmall" pad="xxsmall">
            <Box basis="6vh" fill="horizontal" alignContent='center'  direction="row" gap="medium">
              <Box alignSelf={"center"} background={{color:connection==='connected'?"green":"red"}} round={"large"} width="4vh" height={"4vh"}/>
                <Button onClick={()=>{
                    connect();
                    publishStates();
                  }}>
                  <Box><Text size="3vh">{connection==='connected'?"Connected":"Connect"} to {url}</Text></Box>
                </Button>
            </Box>
            <Box basis="6vh" fill="horizontal" alignContent='center' justify="start" direction="row" gap="medium">
              <Box alignSelf={"center"} background={{color:robotStatus}} round={"large"} width="4vh" height={"4vh"}/>
              <Box justify={"center"} ><Text justify={"center"} size="3vh">Robot state</Text></Box>
              <Box alignSelf={"center"} background={{color:["#",d2hex(1-paperStatus),d2hex(paperStatus),"00"].join('')}} round={"large"} width="4vh" height={"4vh"}/>
              <Box justify={"center"} ><Text justify={"center"} size="3vh">Paper state</Text></Box>
              <Button size="xsmall" alignSelf="center" label={<Box><Text size="3vh">Change Paper</Text></Box>} 
                    onClick={() => {
                    }} />
            </Box>
            
            <Tabs alignSelf="stretch" margin="none" onActive={(val) => {
              setKnownWorkflow(val)
              }}> 
              <Tab title={<Box><Text size="4vh">Unknown Object</Text></Box>}>
                <Box
                round="medium"
                pad="xsmall"
                justify="evenly"
                alignSelf="stretch"
                direction='column' 
                height="75vh"
                gap="small"
                width="40vw"
                >
                  <Box>
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
                    <Button size="xsmall" alignSelf="center" label={<Box><Text size="3vh">Send</Text></Box>}
                      onClick={() => {
                        get_path();
                        //tmpSub.displayCloud();
                      }}
                      />

                    <Button size="xsmall" alignSelf="center" label={<Box><Text size="3vh">Execute</Text></Box>} 
                    onClick={() => {
                      sendMessage("execute");
                      setCanvasOpacity(0);
                    }} />
                  </Box> 
                </Box>
              </Tab>
              <Tab title={<Box><Text size="4vh">Known Object</Text></Box>}>
                <Box
                  round="medium"
                  align="center"
                  justify="evenly"
                  alignSelf="stretch"
                  direction='column' 
                  height="75vh"
                  gap="small"
                  width="40vw"
                  pad="small"
                  >
                    <Button size="small" disabled={scanning} label={<Box><Text size="4vh">Run Scan</Text></Box>} 
                      onClick={() => {
                        sendTrigger("scan"); //Waiting for scanningdone
                        setScanning(true)
                      }} />
                    <Button size="small" label={<Box><Text size="4vh">Delete Object</Text></Box>} 
                      onClick={() => {
                        sendTrigger("deleteactive"); 
                      }} />
                    <Button size="small" label={<Box><Text size="4vh">Change Model</Text></Box>} 
                      onClick={() => {
                        sendTrigger("toggleactive"); 
                      }} />
                    <Button size="small" disabled={computing} label={<Box><Text size="4vh">{computed_traj?"Execute Trajectory":"Compute Trajectory"}</Text></Box>} 
                      onClick={() => {
                        console.log(computed_traj)
                        if(computed_traj)
                          sendModel("execute"); //Waiting for scanningdone
                        else
                          sendObject("on")
                        setComputing(true)
                      }} />
                  </Box>
              </Tab>
            </Tabs>
          </Box>
        </Collapsible>
      </Box>
    )
}