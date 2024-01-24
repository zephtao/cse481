import React from 'react';
import { RegularPolygon } from 'react-konva';

const Polygon = ({ start, end, color, strokeWidth }) => {
    if (!start || !end) return null;
    const radius = Math.sqrt(Math.pow(end.x - start.x, 2) + Math.pow(end.y - start.y, 2));
    return (
        <RegularPolygon
            x={start.x}
            y={start.y}
            sides={6} // Example: Hexagon
            radius={radius}
            stroke={color}
            strokeWidth={strokeWidth}
        />
    );
};

export default Polygon;
