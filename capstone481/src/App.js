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

import { 
    RosConnection, 
    ImageViewer, 
    Subscriber, 
    TopicListProvider, 
    useMsg, 
    useTopicList, 
    Publisher, 
    Param, 
    useParam, 
    ParamListProvider, 
    useParamList, 
    ServiceListProvider, 
    useServiceList, 
    ServiceCaller, 
    ServiceServer
} from "rosreact";

function App() {
  document.body.style = 'background: white;';  // control background color of the webpage

  const [drawing, setDrawing] = useState(false);
  const [lines, setLines] = useState([]);
  const [color, setColor] = useState('#000');
  const [lineWidth, setLineWidth] = useState(2);
  const [currentShape, setCurrentShape] = useState(null);
  const [shapes, setShapes] = useState([]);
  const [isEraserActive, setIsEraserActive] = useState(false);
  const [nextId, setNextId] = useState(0);

  // ROSREACT START
  const [trigger, setTrigger] = useState(false);
  const [delParam, setDelParam] = useState(false);
  const [message, setMessage] = useState({data: 0});

  useEffect(() => {
      setTimeout(() => {
          setTrigger(!trigger);
      }, 3000);
  }, [trigger])

  useEffect(() => {
      setTimeout(() => {
          setMessage({data: 4});
      }, 3000);
  }, [])

  useEffect(() => {
      setTimeout(() => {
          setDelParam(true);
      }, 10000);
  }, [])
  // ROSREACT END

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

  return (
      <div>
        {/* All ROS components are wrapped into a RosConnection */}
        <RosConnection url={"ws://slinky.hcrlab.cs.washington.edu:9090"} autoConnect>
            <Subscriber
                topic="/number"
                messageType="std_msgs/Float32"
            >
                <MsgView/>
            </Subscriber>
            
            <Param 
                name="/react/param"
                setValue={1}
                get={trigger}
                delete={delParam}
                deleteCallback={(resp) => {console.log(resp)}}
                setCallback={(resp) => {console.log(resp)}}
            >
                <ParamView/>
            </Param>
            
            <Publisher 
                autoRepeat 
                topic="/react/pub/repeat"
                throttleRate={10.0} 
                message={{data: 2}} 
                messageType="std_msgs/Float32"
            />
            
            <Publisher 
                topic="/react/pub/norepeat"
                throttleRate={10.0} 
                message={message} 
                messageType="std_msgs/Float32"
                latch={true}
            />

            <ServiceServer 
                name="/react/service" 
                serviceType="std_srvs/SetBool" 
                callback={serviceServerCallback}
            />

            <ServiceCaller 
                name="/setbool" 
                serviceType="std_srvs/SetBool" 
                request={{data: true}} 
                trigger={trigger}
                callback={(resp) => {console.log(resp)}} 
                failedCallback={(error) => {console.log(error)}}
            />
            
            <TopicListProvider
                trigger={trigger} 
                failedCallback={(e) => {console.log(e)}}
            >
                <TopicListView/>
            </TopicListProvider>
            
            <ServiceListProvider
                trigger={trigger}
                failedCallback={(e) => {console.log(e)}}
            >
                <ServiceListView/>
            </ServiceListProvider>
            
            <ParamListProvider
                trigger={trigger} 
                failedCallback={(e) => {console.log(e)}}
            >
                <ParamListView/>
            </ParamListProvider>
        
        </RosConnection>
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

// ROSREACT START
const serviceServerCallback = (request, response) => {
  if (request.data === true) {
      response.success = true;
      response.message = "Passed true value";
  } else {
      response.success = false;
      response.message = "Passed false value";
  }
}

const ParamView = () => {
  const param = useParam();
  return <p>{`${param}`}</p>
}


const MsgView = () => {
  const msg = useMsg();
  return <p> {`${msg.distance}`} </p>
}


const TopicListView = () => {
  const topicList = useTopicList();
  return ( 
      <Fragment>
      <p>{`${topicList.topics}`}</p>
      <p>{`${topicList.types}`}</p>
      </Fragment>
  );
}


const ServiceListView = () => {
  const list = useServiceList();
  return (
      <p>{`${list}`}</p>
  );
}


const ParamListView = () => {
  const list = useParamList();
  return ( 
      <p>{`${list}`}</p>
  );
}
// ROSREACT END

export default App;