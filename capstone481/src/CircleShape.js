import React from 'react';
import { Circle } from 'react-konva';
const CircleShape = ({id, start, end, color, strokeWidth }) => {
    if (!start || !end) {
        return null;
    }

    const radius = Math.sqrt(
        (end.x - start.x) ** 2 + (end.y - start.y) ** 2
    );

    return (
        <Circle
            id = {id}
            x={start.x}
            y={start.y}
            radius={radius}
            stroke={color}
            strokeWidth={strokeWidth}
        />
    );
};

export default CircleShape;
