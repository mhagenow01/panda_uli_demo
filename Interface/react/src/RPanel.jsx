import React from 'react';
import useAppStore from './AppStore';
import useRosStore from './RosStore';
import {ParameterModule} from './ParameterModule';
import { Grommet, List, Stack,Button, TextInput, Heading, Text, Card, CardHeader, CardBody, Grid, Box,CheckBox } from 'grommet';

export const RPanel = (props) => {  
  const [connect, connection, url,imageTopic] = useRosStore(state=>([state.connect, state.connection, state.url,state.imageTopic])) 
  const parameters = useAppStore(state=>state.parameters)
  
  const [messages, addMessage, sendMessage] = useAppStore(state=>([state.messages,state.addMessage,state.sendMessage]))
    return (
      <Grid
      rows={['10%', '10%','70%']}
      columns={['90%', '10%']}
      gap={{row: "5%", column: "none"}}
      areas={[
        {name: 'tl', start:[0,0], end: [0,0]},
        {name: 'tr', start:[1,0], end: [1,0]},
        {name: 'send', start:[0,1], end: [1,1]},
        {name: 'main', start:[0,2], end: [1,2]},
      ]}
    > 
        <Button gridArea='tl' onClick={connect}>{connection==='connected'?"Connected":"Connect"} to {url}</Button>

       <CheckBox gridArea="tr" disabled={true} checked={connection==='connected'}/>
       <Box gridArea='send'  basis="medium"  alignSelf="center">
        <Button size="large" alignSelf="center" label="Send"
          onClick={() => {
            sendMessage();
          }}
          />
          </Box> 
          <Box gridArea='main'>
            <Card background="light-1">
              <CardHeader pad="small" background="light-2">
                  <Text alignSelf="center" color="brand" size="large">Properties</Text>
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