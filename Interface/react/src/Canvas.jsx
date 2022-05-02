import React from 'react';
import useAppStore from './AppStore';
import { Stage, Layer, Circle, Shape, Text } from 'react-konva';
import { Stack } from 'grommet';

export const Canvas = (props) => {
    var [corners,path,canvasOpacity] = useAppStore(state=>[state.corners,state.path,state.canvasOpacity]);
    const [maxWidth,maxHeight] = useAppStore(state=>[state.imageWidth, state.imageHeight])
    const setCorner = useAppStore(state=>state.setCorner)
    const handleDragEnd = (e) => {
        setCorner(e.target.attrs["cornerId"],Math.min(Math.max(e.target.attrs["x"],0),maxWidth),Math.min(Math.max(e.target.attrs["y"],0),maxHeight))
        corners.map((corner) => {
          return {
            ...corner,
            isDragging: false,
          };
        }
      );
    };
    const handleMove = (e) => {
        e.target.attrs["x"]=Math.min(Math.max(e.target.attrs["x"],0),maxWidth)
        e.target.attrs["y"]=Math.min(Math.max(e.target.attrs["y"],0),maxHeight)
        setCorner(e.target.attrs["cornerId"],e.target.attrs["x"],e.target.attrs["y"])
        corners.map((corner) => {
          return {
            ...corner
          };
        }
      );
    };
    return ( 
    <Stage width={maxWidth} height={maxHeight}>
        <Layer>
          <Shape
            sceneFunc={(context, shape) => {
              context.beginPath();
              context.moveTo(path[0].x, path[0].y);
              path.map(point=>{
                context.lineTo(point.x, point.y);  
              })
              // (!) Konva specific method, it is very important
              context.fillStrokeShape(shape);
            }}
            stroke="green"
            opacity={.5*canvasOpacity}
            strokeWidth={4}
          />

          <Shape
            sceneFunc={(context, shape) => {
              context.beginPath();
              context.moveTo(corners[0].x, corners[0].y);
              context.lineTo(corners[1].x, corners[1].y);
              context.lineTo(corners[2].x, corners[2].y);
              context.lineTo(corners[3].x, corners[3].y);
              context.closePath();
              // (!) Konva specific method, it is very important
              context.fillStrokeShape(shape);
            }}
            fill="#00D2FF"
            stroke="red"
            opacity={.1*canvasOpacity}
            strokeWidth={4}
          />
          {corners.map((corner) => (
            <Circle
                key={corner.id}
                id={corner.id}
                cornerId={corner.id}
                x={corner.x}
                y={corner.y}
                Radius={20}
                fill="#0000ff"
                opacity={0.8*canvasOpacity}
                draggable
                shadowColor="black"
                shadowBlur={10}
                shadowOpacity={0.6}
                shadowOffsetX={corner.isDragging ? 10 : 5}
                shadowOffsetY={corner.isDragging ? 10 : 5}
                scaleX={corner.isDragging ? 1.2 : 1}
                scaleY={corner.isDragging ? 1.2 : 1}
                // onDragStart={handleDragStart}
                onDragMove={handleMove}
                onDragEnd={handleDragEnd} />
          ))
          }
        </Layer>
      </Stage>
    )
}