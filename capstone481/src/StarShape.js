import React from 'react';
import { Star } from 'react-konva';

const StarShape = ({ start, end, color, strokeWidth }) => {
    if (!start || !end) return null;
    const radius = Math.sqrt(Math.pow(end.x - start.x, 2) + Math.pow(end.y - start.y, 2));
    return (
        <Star
            x={start.x}
            y={start.y}
            numPoints={5}
            innerRadius={radius / 2}
            outerRadius={radius}
            stroke={color}
            strokeWidth={strokeWidth}
        />
    );
};

export default StarShape;
