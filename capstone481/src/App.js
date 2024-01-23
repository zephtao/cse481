import React, { useState } from 'react';
import { Stage, Layer, Line } from 'react-konva';


function App() {
  const [drawing, setDrawing] = useState(false);
  const [lines, setLines] = useState([]);
  const [color, setColor] = useState('#000');
  const [lineWidth, setLineWidth] = useState(2);

  const handleMouseDown = () => {
    setDrawing(true);
    setLines([...lines, { color, points: [] }]);
  };

  const handleMouseUp = () => {
    setDrawing(false);
  };

  const handleMouseMove = (e) => {
    if (!drawing) {
      return;
    }
    const stage = e.target.getStage();
    const point = stage.getPointerPosition();
    let lastLine = lines[lines.length - 1];
    lastLine.points = lastLine.points.concat([point.x, point.y]);
    setLines([...lines.slice(0, lines.length - 1), lastLine]);
  };

  return (
      <div>
        <div>
          <label>Color:</label>
          <input
              type="color"
              value={color}
              onChange={(e) => setColor(e.target.value)}
          />
          <label>Line Width:</label>
          <input
              type="number"
              value={lineWidth}
              onChange={(e) => setLineWidth(e.target.value)}
          />
        </div>
        <Stage
            width={window.innerWidth}
            height={window.innerHeight - 50}
            onMouseDown={handleMouseDown}
            onMouseUp={handleMouseUp}
            onMouseMove={handleMouseMove}
        >
          <Layer>
            {lines.map((line, i) => (
                <Line
                    key={i}
                    points={line.points}
                    stroke={line.color}
                    strokeWidth={lineWidth}
                    tension={0.5}
                    lineCap="round"
                />
            ))}
          </Layer>
        </Stage>
      </div>
  );
}

export default App;
