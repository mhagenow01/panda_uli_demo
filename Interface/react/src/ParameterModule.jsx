import React, {useCallback} from 'react';
import useAppStore from './AppStore';
import { Grommet, List, Stack,Button, Text, Card, Grid, Box,CheckBox, Select, RangeInput } from 'grommet';

export const ParameterModule = (props) => { 
  const {idx} = props;
  const parameterInfo = useAppStore(useCallback(state=>state.parameters[idx],[idx]));

  // const parameterInfoChanged = useAppStore(useCallback(state=>{
  //   state.buttonDetails[idx]
  // },[idx]));
  const [setParameter] = useAppStore(state=>([state.setParameter]))
  if(!parameterInfo)
    return null
  const label =<Box justify={"center"} alignContent={"center"} width={'30%'}><Text size="2.2vh">{parameterInfo.label}</Text></Box>
  return (
    <Box height={"xxsmall"} direction="row" gap="xxsmall">
      {label}
      {parameterInfo.type === 'button' && (
        <Button label={parameterInfo.label} 
        margin={"small"}/>
      )}
      {parameterInfo.type === 'slider' && (
        <Box pad="small" width={"70%"} direction="row" gap="xsmall" >
            <Box alignSelf='center' justify={"center"} width={"medium"} height={"1vh"}>
              <RangeInput 
              color="#9b0000"
              value= {parameterInfo.value}
              min = {parameterInfo.min}
              max = {parameterInfo.max}
              onChange={event => setParameter(idx,event.target.value)}
              margin={"medium"}/>
            </Box>
            <Box justify={"center"} alignContent={"center"} width={"small"}>
              <Text justify={"center"} alignSelf="left" size="2.2vh" >{parameterInfo.value+" "+parameterInfo.unit} </Text>
            </Box>
        </Box>
      )}
      {parameterInfo.type === 'select' && (
        <Box pad="xsmall" width={"70%"} gap="small" justify={"center"} >
          <Select
          options={parameterInfo.options}
          value= {parameterInfo.value}
          onChange={({ value,option }) => {
            setParameter(idx,option)}}
          margin={"xxsmall"}
          size="2vh"
        />
        </Box>
      )}
      
    </Box>
  )
}