import React, {useState} from 'react';
// import {List,Button,Checkbox,Card,Space,Input} from 'antd';
// import {EnterOutlined} from '@ant-design/icons';
import Stream from './Stream';
import { Grommet, List, Button, TextInput, Card, Grid, Box,CheckBox } from 'grommet';
import styled from "styled-components";
import useAppStore from './AppStore';
import useRosStore from './RosStore';

function App() {

  const [connect, connection, url,imageTopic] = useRosStore(state=>([state.connect, state.connection, state.url,state.imageTopic])) 
  const [messages, addMessage, sendMessage] = useAppStore(state=>([state.messages,state.addMessage,state.sendMessage]))

  const [text, setText] = useState('Some Message')
  return (
    <Grid
      rows={['xsmall', 'xsmall', 'xsmall','xlarge']}
      columns={['xxlarge','small', 'small']}
      gap="small"
      areas={[
        {name: 'top', start:[0,0], end: [2,0]},
        {name: 'video', start:[0,1], end: [0,3]},
        { name: 'topleft', start: [1, 1], end: [1, 1] },
        { name: 'topright', start: [2, 1], end: [2, 1] },
        { name: 'botleft', start: [1, 2], end: [1, 2] },
        { name: 'botright', start: [2, 2], end: [2, 2] },
      ]}
    >
      <Box gridArea="top" background="brand" />
      <Stream gridArea='video' imageTopic={imageTopic} />
      <Button gridArea="topleft" onClick={connect}>{connection==='connected'?"Connected":"Connect"} to {url}</Button>
      <CheckBox gridArea="topright" disabled={true} checked={connection==='connected'}/>
      <TextInput  gridArea="botleft"
      placeholder="type here"
      value={text}
      onChange={event => setText(event.target.value)}
      />

      <Button primary gridArea="botright" label="Send"
      onClick={() => {
        sendMessage(text);
      }}
      />
    </Grid>

    // <Button onClick={connect}>{connection==='connected'?"Connected":"Connect"} to {url}</Button>
      
    //   <Card 
    //     title='Messages' 
    //     bodyStyle={{padding:0}}
    //     // extra={
    //     //   <Space>
    //     //     <Input.Search 
    //     //       defaultValue={text} 
    //     //       onSearch={()=>addMessage(text)} 
    //     //       onChange={(e)=>setText(e.target.value)} 
    //     //       enterButton={<EnterOutlined/>}/>
    //     //     <Button onClick={connect}>{connection==='connected'?"Connected":"Connect"} to {url}</Button>
    //     //     <Checkbox disabled={true} checked={connection==='connected'}/>
    //     //   </Space>
    //     //   }
    //     >
    //   <List
    //     dataSource={messages}
    //     renderItem={(item,idx)=>(
    //       <List.Item title={item} style={{padding:10}} extra={connection==='connected' && <Button onClick={()=>{sendMessage(idx)}}>Send</Button>}>
    //         {<List.Item.Meta title={item}/>}
    //       </List.Item>
    //     )}
    //   >
    //   </List>
    // </Card>
    );
}

export default App;
