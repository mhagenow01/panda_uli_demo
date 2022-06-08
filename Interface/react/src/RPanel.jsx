import React from 'react';
import useAppStore from './AppStore';
import useRosStore from './RosStore';
import {ParameterModule} from './ParameterModule';
import { Grommet, List, Stack,Button, TextInput, Heading, Text, Card, CardHeader, CardBody, Grid, Box,CheckBox } from 'grommet';

export const RPanel = (props) => {  
  const [connect, connection, url,imageTopic,tmpSub] = useRosStore(state=>([state.connect, state.connection, state.url,state.imageTopic,state.tmpSub])) 
  const parameters = useAppStore(state=>state.parameters)
  
  const [messages, addMessage, sendMessage, sendCoordinates,publishStates,setCanvasOpacity] = useAppStore(state=>([state.messages,state.addMessage,state.sendMessage,state.sendCoordinates,state.publishStates,state.setCanvasOpacity]))

    return (
      <Grid
      rows={['10%','70%','10%']}
      columns={['95%']}
      gap={{row: "5%", column: "none"}}
      areas={[
        {name: 'top', start:[0,0], end: [0,0]},
        {name: 'main', start:[0,1], end: [0,1]},
        {name: 'bottom', start:[0,2], end: [0,2]},
      ]}
    > 
      <Box gridArea='top' basis="medium"  alignSelf="center" align={"end"} justify="center" direction="row" gap="medium">
          <Button onClick={()=>{
            connect();
            publishStates();
          }}><Box><Text size="2vh">{connection==='connected'?"Connected":"Connect"} to {url}</Text></Box></Button>

          <CheckBox disabled={true} checked={connection==='connected'}/>
       </Box>

       <Box gridArea='bottom' basis="medium" alignSelf="center" align={"end"} justify="center" direction="row" gap="medium">
        <Button size="xsmall" alignSelf="center" label=<Box><Text size="3vh">Send</Text></Box> 
          onClick={() => {
            sendCoordinates();
            //tmpSub.displayCloud();
          }}
          />

          <Button size="xsmall" alignSelf="center" label=<Box><Text size="3vh">Execute</Text></Box> 
          onClick={() => {
            sendMessage("execute");
            setCanvasOpacity(0);
          }} />
   
        </Box> 
          <Box gridArea='main'>
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
        </Grid>
    )
}