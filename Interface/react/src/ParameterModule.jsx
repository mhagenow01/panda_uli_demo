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
  const label =<Box justify={"center"} alignContent={"center"} width={'30%'}><Text >{parameterInfo.label}</Text></Box>
  return (
    <Box height={"xxsmall"} direction="row" gap="medium">
      {label}
      {parameterInfo.type === 'button' && (
        <Button label={parameterInfo.label} 
        margin={"small"}/>
      )}
      {parameterInfo.type === 'slider' && (
        <Box pad="small" width={"medium"} direction="row" justify={"center"} gap="small">
            <Box justify={"center"} width={"medium"}>
              <RangeInput 
              color="#9b0000"
              value= {parameterInfo.value}
              min = {parameterInfo.min}
              max = {parameterInfo.max}
              onChange={event => setParameter(idx,event.target.value)}
              margin={"small"}/>
            </Box>
            <Box justify={"center"} width={"xxsmall"}>
              <Text alignSelf="center" >{parameterInfo.value} </Text>
            </Box>
        </Box>
      )}
      {parameterInfo.type === 'select' && (
        <Box width={"medium"} justify={"center"} >
          <Select
          options={parameterInfo.options}
          value= {parameterInfo.value}
          onChange={({ value,option }) => {
            setParameter(idx,option)}}
          margin={"small"}
        />
        </Box>
      )}
      
    </Box>
  )
}