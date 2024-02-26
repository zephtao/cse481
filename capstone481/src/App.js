import React, { useState, useEffect, Fragment } from 'react';
import { Stage, Layer, Line } from 'react-konva';
import Rectangle from './Rectangle';
import CircleShape from './CircleShape';
import Square from './Square';
import StarShape from './StarShape';
import Polygon from './Polygon';
import EllipseShape from './EllipseShape';
import LineShape from './LineShape';
import Triangle from "./Triangle";
import ROSLIB from "roslib";


function App() {
  document.body.style = 'background: white;';  // control background color of the webpage

  // for drawing:
  const [drawing, setDrawing] = useState(false);
  const [lines, setLines] = useState([]);
  const [color, setColor] = useState('#000');
  const [lineWidth, setLineWidth] = useState(2);
  const [currentShape, setCurrentShape] = useState(null);
  const [shapes, setShapes] = useState([]);
  const [isEraserActive, setIsEraserActive] = useState(false);
  const [nextId, setNextId] = useState(0);

  // for robot/ROS:
  const [isConnected, setIsConnected] = useState(false);
  const [liftPosition, setLiftPosition] = useState(0);
  const [wristExtension, setWristExtension] = useState(0);

  const JOINT_LIMITS = {
    "joint_lift": [0.175, 1.05],
    "wrist_extension": [0.05, 0.518]
  }

  const ros = new ROSLIB.Ros({
    url : "ws://slinky.hcrlab.cs.washington.edu:9090"
  });

  let trajectoryClient = null;

  // once the ROS connection is made, create the trajectory client
  ros.on('connection', () => {
    setIsConnected(true);
    subscribeToJointState();

    trajectoryClient = new ROSLIB.ActionHandle({
      ros: ros,
      name: '/stretch_controller/follow_joint_trajectory',
      actionType: 'control_msgs/action/FollowJointTrajectory'
    });
  });

  // subscribes to the topic that publishes the robot joint position
  const subscribeToJointState = () => {
    const jointStateTopic = new ROSLIB.Topic({
      ros: ros,
      name: '/stretch/joint_states',
      messageType: 'sensor_msgs/JointState'
    });

    jointStateTopic.subscribe((msg) => {
      const liftIndex = msg.name.indexOf('joint_lift');
      const wristIndex = msg.name.indexOf('wrist_extension');

      if (liftIndex !== -1) {
        setLiftPosition(msg.position[liftIndex]);
      }

      if (wristIndex !== -1) {
        setWristExtension(msg.position[wristIndex]);
      }
    });
  };

  const moveLiftUp = () => {
    console.log("move lift up function called")
    let upGoal = new ROSLIB.ActionGoal({
      trajectory: {
        header: {
          stamp: {
            secs: 0,
            nsecs: 0
          }
        },
        joint_names: ['joint_lift'],
        points: [
          {
            positions: [liftPosition + 0.2],
            time_from_start: {
              secs: 1,
              nsecs: 0
            }
          }
        ]
      }
    });

    trajectoryClient.createClient(upGoal);
    console.log("move lift up new goal created")
    console.log(upGoal)
  }

  const moveLiftDown = () => {
    let downGoal = new ROSLIB.ActionGoal({
      trajectory: {
        header: {
          stamp: {
            secs: 0,
            nsecs: 0
          }
        },
        joint_names: ['joint_lift'],
        points: [
          {
            positions: [liftPosition - 0.2],
            time_from_start: {
              secs: 1,
              nsecs: 0
            }
          }
        ]
      }
    });

    trajectoryClient.createClient(downGoal);
  }

  const moveWristOut = () => {
    let outGoal = new ROSLIB.ActionGoal({
      trajectory: {
        header: {
          stamp: {
            secs: 0,
            nsecs: 0
          }
        },
        joint_names: ['wrist_extension'],
        points: [
          {
            positions: [wristExtension + 0.1],
            time_from_start: {
              secs: 1,
              nsecs: 0
            }
          }
        ]
      }
    });

    trajectoryClient.createClient(outGoal);
  }

  const moveWristIn = () => {
    console.log("move wrist in function called")
    let inGoal = new ROSLIB.ActionGoal({
      trajectory: {
        header: {
          stamp: {
            secs: 0,
            nsecs: 0
          }
        },
        joint_names: ['wrist_extension'],
        points: [
          {
            positions: [wristExtension - 0.1],
            time_from_start: {
              secs: 1,
              nsecs: 0
            }
          }
        ]
      }
    });

    trajectoryClient.createClient(inGoal);
  }

  const moveLift = () => {
    console.log("move lift to 0.3 function called")
    let newGoal = new ROSLIB.ActionGoal({
      trajectory: {
        header: {
          stamp: {
            secs: 0,
            nsecs: 0
          }
        },
        joint_names: ['joint_lift'],
        points: [
          {
            positions: [0.3],
            time_from_start: {
              secs: 1,
              nsecs: 0
            }
          }
        ]
      }
    });

    trajectoryClient.createClient(newGoal);
    console.log("move lift up new goal created")
    console.log(newGoal)
  };


  const handleMouseDown = (e) => {
    const stage = e.target.getStage();
    const point = stage.getPointerPosition();

    if (isEraserActive) {
      if (e.target !== stage) {
        const shapeId = e.target.id();
        setShapes(shapes.filter(shape => shape.id !== shapeId));
      }
      return;
    }

    if (currentShape) {
      // Common properties for all shapes
      const shapeProperties = {
        start: point,
        end: point,
        color: color,
        strokeWidth: lineWidth
      };
      setCurrentShape({ ...currentShape, ...shapeProperties });
      setDrawing(true);
    } else {
      // Start drawing a freehand line
      setLines([...lines, { color, points: [point.x, point.y] }]);
      setDrawing(true);
    }
  };

  const handleMouseUp = () => {
    if (drawing) {
      setDrawing(false);
      if (currentShape) {
        setShapes([...shapes, { ...currentShape, id: nextId }]);
        setNextId(nextId + 1);
      }
    }
  };

  const handleMouseMove = (e) => {
    if (!drawing) return;

    const stage = e.target.getStage();
    const point = stage.getPointerPosition();

    // Clamp the point to be within the stage dimensions
    const clampedPoint = {
      x: Math.min(Math.max(point.x, 0), stage.width()),
      y: Math.min(Math.max(point.y, 0), stage.height()),
    };

    if (currentShape) {
      setCurrentShape(prevShape => ({
        ...prevShape,
        end: clampedPoint
      }));
    } else {
      // Update the last line's points for freehand drawing
      let lastLine = lines[lines.length - 1];
      lastLine.points = lastLine.points.concat([clampedPoint.x, clampedPoint.y]);
      setLines([...lines.slice(0, lines.length - 1), lastLine]);
    }
  };
  const clearAllDrawings = () => {
    setShapes([]);
    setLines([]);
    setCurrentShape(null);
  };
  const toggleEraser = () => {
    setIsEraserActive(!isEraserActive);
    // Optionally, reset currentShape when eraser is activated
    if (isEraserActive) {
      setCurrentShape(null);
    }
  };

  // if (!isConnected) {
  //   return (<div>Loading...</div>)
  // };

  // cmd_vel topic to move base
  // follow joint trajectory action server

  return (
      <div>
        <p>Is connected:</p>
        {isConnected}
        <div>

          <h1>Welcome To Our Coloring Robot Interface!</h1>

          <p>Click on the shape you want, then drag anywhere on the page. Or, select "Free Draw" and use your cursor to draw anything!</p>

          <label>Color:</label>
          <input
              type="color"
              value={color}
              onChange={(e) => setColor(e.target.value)}
          />
          <label>Line Width:</label>
          <input
              className="normal-input"
              type="number"
              value={lineWidth}
              onChange={(e) => setLineWidth(e.target.value)}
          />
          <button className="normal-button" onClick={() => setCurrentShape({ type: 'circle' })}>Circle</button>
          <button className="normal-button" onClick={() => setCurrentShape({ type: 'rectangle' })}>Rectangle</button>
          <button className="normal-button" onClick={() => setCurrentShape({ type: 'triangle' })}>Triangle</button>
          <button className="normal-button" onClick={() => setCurrentShape({ type: 'square' })}>Square</button>
          <button className="normal-button" onClick={() => setCurrentShape({ type: 'star' })}>Star</button>
          <button className="normal-button" onClick={() => setCurrentShape({ type: 'polygon' })}>Polygon</button>
          <button className="normal-button" onClick={() => setCurrentShape({ type: 'ellipse' })}>Ellipse</button>
          <button className="normal-button" onClick={() => setCurrentShape({ type: 'line' })}>Line</button>
          <button className="normal-button" onClick={() => setCurrentShape(null)}>Free Draw</button> {/* Button to draw lines freely */}
          <button className="normal-button" onClick={toggleEraser}>{isEraserActive ? 'Disable Eraser' : 'Enable Eraser'}</button>
          <button className="normal-button" onClick={clearAllDrawings}>Clear</button>

        </div>
        <div>
        <button className="normal-button" onClick={() => moveLift()}>Move Lift To 0.3</button>
        <button className="normal-button" onClick={() => moveLiftUp()}>Move Lift Up</button>
        <button className="normal-button" onClick={() => moveLiftDown()}>Move Lift Down</button>
        <button className="normal-button" onClick={() => moveWristOut()}>Move Wrist Out</button>
        <button className="normal-button" onClick={() => moveWristIn()}>Move Wrist In</button>
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
                return <CircleShape key={i} id={shape.id} {...shape} />;
              } else if (shape.type === 'rectangle') {
                return <Rectangle key={i} id={shape.id} {...shape} />;
              } else if (shape.type === 'triangle') {
                return <Triangle key={i} id={shape.id} {...shape} />;
              } else if (shape.type === 'square') {
                return <Square key={i} id={shape.id} {...shape} />;
              } else if (shape.type === 'star') {
                return <StarShape key={i} id={shape.id} {...shape} />;
              } else if (shape.type === 'polygon') {
                return <Polygon key={i} id={shape.id} {...shape} />;
              } else if (shape.type === 'ellipse') {
                return <EllipseShape key={i} id={shape.id} {...shape} />;
              } else if (shape.type === 'line') {
                return <LineShape key={i} id={shape.id} {...shape} />;
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
            {currentShape && currentShape.type === 'square' && (
                <Square {...currentShape} />
            )}
            {currentShape && currentShape.type === 'star' && (
                <StarShape {...currentShape} />
            )}
            {currentShape && currentShape.type === 'polygon' && (
                <Polygon {...currentShape} />
            )}
            {currentShape && currentShape.type === 'ellipse' && (
                <EllipseShape {...currentShape} />
            )}
            {currentShape && currentShape.type === 'line' && (
                <LineShape {...currentShape} />
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
            {lines.map((line, i) => (
                <Line
                    key={i}
                    points={line.points}
                    stroke={isEraserActive ? 'white' : line.color}
                    strokeWidth={lineWidth}
                    globalCompositeOperation={
                      isEraserActive ? 'destination-out' : 'source-over'
                    }
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