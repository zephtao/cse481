import React from 'react';
import { Line } from 'react-konva';

const LineShape = ({ start, end, color, strokeWidth }) => {
    if (!start || !end) return null;
    return (
        <Line
            points={[start.x, start.y, end.x, end.y]}
            stroke={color}
            strokeWidth={strokeWidth}
            lineCap="round"
        />
    );
};

export default LineShape;
