import React from 'react';
import { Rect } from 'react-konva';

const Square = ({ start, end, color, strokeWidth }) => {
    if (!start || !end) return null;
    const size = Math.max(Math.abs(end.x - start.x), Math.abs(end.y - start.y));
    return (
        <Rect
            x={Math.min(start.x, end.x)}
            y={Math.min(start.y, end.y)}
            width={size}
            height={size}
            stroke={color}
            strokeWidth={strokeWidth}
        />
    );
};

export default Square;
