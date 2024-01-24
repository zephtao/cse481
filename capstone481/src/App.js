import React, { useState } from 'react';
import { Stage, Layer, Line } from 'react-konva';
import Rectangle from './Rectangle'; // Import the Rectangle component
import CircleShape from './CircleShape';
import Triangle from "./Triangle"; // Import the Circle component

function App() {
  const [drawing, setDrawing] = useState(false);
  const [lines, setLines] = useState([]);
  const [color, setColor] = useState('#000');
  const [lineWidth, setLineWidth] = useState(2);
  const [currentShape, setCurrentShape] = useState(null);
  const [shapes, setShapes] = useState([]);

  const handleMouseDown = (e) => {
    const stage = e.target.getStage();
    const point = stage.getPointerPosition();

    if (currentShape) {
      // Common properties for both shapes
      const shapeProperties = {
        start: point,
        end: point,
        color: color,
        strokeWidth: lineWidth
      };

      // Set the type property based on the currentShape type
      if (currentShape.type === 'circle') {
        setCurrentShape({ type: 'circle', ...shapeProperties });
      } else if (currentShape.type === 'rectangle') {
        setCurrentShape({ type: 'rectangle', ...shapeProperties });
      } else if (currentShape.type === 'triangle') {
      setCurrentShape({ type: 'triangle', ...shapeProperties });
    }
    setDrawing(true);
    } else {
      // Start drawing a line
      setLines([...lines, { color, points: [point.x, point.y] }]);
      setDrawing(true);
    }
  };

  const handleMouseUp = () => {
    if (drawing) {
      setDrawing(false);
      if (currentShape) {
        // Finalize the shape (circle)
        setShapes([...shapes, currentShape]);
        setCurrentShape(null);
      }
    }
  };

  const handleMouseMove = (e) => {
    if (!drawing) return;

    const stage = e.target.getStage();
    const point = stage.getPointerPosition();

    if (currentShape) {
      if (currentShape.type === 'circle') {
        // Update the end point of the circle to adjust the radius
        setCurrentShape(prevShape => ({
          ...prevShape,
          end: point
        }));
      } else if (currentShape.type === 'rectangle') {
        setCurrentShape(prevShape => ({
          ...prevShape,
          end: point
        }));
      } else if (currentShape.type === 'triangle') {
        setCurrentShape(prevShape => ({
          ...prevShape,
          end: point
        }));
      }
    } else {
      // Update the last line's points for freehand drawing
      let lastLine = lines[lines.length - 1];
      lastLine.points = lastLine.points.concat([point.x, point.y]);
      setLines([...lines.slice(0, lines.length - 1), lastLine]);
    }
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
          <button onClick={() => setCurrentShape({ type: 'circle' })}>Circle</button>
          <button onClick={() => setCurrentShape({ type: 'rectangle' })}>Rectangle</button>
          <button onClick={() => setCurrentShape({ type: 'triangle' })}>Triangle</button>
        </div>
        <Stage
            width={window.innerWidth}
            height={window.innerHeight - 50}
            onMouseDown={handleMouseDown}
            onMouseUp={handleMouseUp}
            onMouseMove={handleMouseMove}
        >
          <Layer>
            {/* Render existing shapes */}
            {shapes.map((shape, i) => {
              if (shape.type === 'circle') {
                return <CircleShape key={i} {...shape} />;
              } else if (shape.type === 'rectangle') {
                return <Rectangle key={i} {...shape} />;
              } else if (shape.type === 'triangle') {
                return <Triangle key={i} {...shape} />;
              }
            })}

            {/* Render the current shape being drawn */}
            {currentShape && currentShape.type === 'circle' && (
                <CircleShape {...currentShape} />
            )}

            {/* Render the current shape being drawn */}
            {currentShape && currentShape.type === 'rectangle' && (
                <Rectangle {...currentShape} />
            )}

            {/* Render the current shape being drawn */}
            {currentShape && currentShape.type === 'triangle' && (
                <Triangle {...currentShape} />
            )}

            {/* Render lines as before */}
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