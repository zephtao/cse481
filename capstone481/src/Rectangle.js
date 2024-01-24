import React from 'react';
import { Rect } from 'react-konva';

const Rectangle = ({ start, end, color, strokeWidth }) => {
    if (!start || !end) {
        return null;
    }

    return (
        <Rect
            x={Math.min(start.x, end.x)}
            y={Math.min(start.y, end.y)}
            width={Math.abs(end.x - start.x)}
            height={Math.abs(end.y - start.y)}
            stroke={color}
            strokeWidth={strokeWidth}
        />
    );
};

export default Rectangle;
