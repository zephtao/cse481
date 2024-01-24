import React from 'react';
import { Line } from 'react-konva';

const Triangle = ({ start, end, color, strokeWidth }) => {
    if (!start || !end) {
        return null;
    }

    // Calculate the three points of the triangle
    const points = [
        start.x, start.y,                             // Point 1: Start point
        end.x, start.y,                               // Point 2: Base right
        (start.x + end.x) / 2, end.y                  // Point 3: Top center
    ];

    return (
        <Line
            points={points}
            stroke={color}  // Stroke color of the triangle
            strokeWidth={strokeWidth}
            closed={true}   // Close the line to form a triangle
        />
    );
};

export default Triangle;
