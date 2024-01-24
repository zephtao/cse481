import React from 'react';
import { Ellipse } from 'react-konva';

const EllipseShape = ({ start, end, color, strokeWidth }) => {
    if (!start || !end) return null;
    return (
        <Ellipse
            x={(start.x + end.x) / 2}
            y={(start.y + end.y) / 2}
            radiusX={Math.abs(end.x - start.x) / 2}
            radiusY={Math.abs(end.y - start.y) / 2}
            stroke={color}
            strokeWidth={strokeWidth}
        />
    );
};

export default EllipseShape;
