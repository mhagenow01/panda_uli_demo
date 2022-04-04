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
  const label =<Text alignSelf="center">{parameterInfo.label}</Text>
  return (
    <Box direction="row" gap="medium">
      {label}
      {parameterInfo.type === 'button' && (
        <Button label={parameterInfo.label} 
      margin={"small"}/>
      )}
      {parameterInfo.type === 'slider' && (
      <Box direction="row" gap="medium">
          <RangeInput 
          value= {parameterInfo.value}
          min = {parameterInfo.min}
          max = {parameterInfo.max}
          onChange={event => setParameter(idx,event.target.value)}
          margin={"small"}/>
          <Text >{parameterInfo.value}</Text>
      </Box>
      )}
      {parameterInfo.type === 'select' && (
        <Select
        options={parameterInfo.options}
        value= {parameterInfo.value}
        onChange={({ value,option }) => {
          setParameter(idx,option)}}
        margin={"small"}
      />
      )}
      
    </Box>
  )
}