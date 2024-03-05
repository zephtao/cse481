import React, { useState, useEffect } from 'react';
import './App.css';
import ROSLIB from "roslib";

////////////////////////////// CANVAS //////////////////////////////
function Canvas({ selectedShape, shapeList, setShapeList, realShapeList, setRealShapeList }) {
  const handleAreaClick = (event) => {
    const { offsetX, offsetY } = event.nativeEvent;

    const webCanvasWidth = 500;
    const webCanvasHeight = 600;
    const realCanvasWidth = 0.635;  // in meters
    const realCanvasHeight = 0.762; // in meters

    if (offsetX >= 60 && offsetX <= 440 && offsetY >= 60 && offsetY <= 540) {
      console.log('(x,y) offset:', offsetX, offsetY, 'shape:', selectedShape);

      if (selectedShape) {
          // get center of the shape element
          const shapeWidth = 100;
          const shapeHeight = 100;
          const centerX = offsetX - (shapeWidth / 2);
          const centerY = offsetY - (shapeHeight / 2);

          // offsetX/500 = ?/0.635
          // offsetY/600 = ?/0.762
          const realCenterX = (offsetX * realCanvasWidth) / webCanvasWidth;
          const realCenterY = (offsetY * realCanvasHeight) / webCanvasHeight;

          console.log('real-life centerX:', realCenterX, 'meters');
          console.log('real-life centerY:', realCenterY, 'meters');

          setShapeList([...shapeList, { type: selectedShape, x: centerX, y: centerY }]);
          setRealShapeList([...realShapeList, {type: selectedShape, x: realCenterX, y: realCenterY}]);
      }
    } else {
      alert("ERROR: You placed the shape too close to the edge of the paper!")
    }
  };

  return (
      <div className="shape-area" onClick={handleAreaClick}>
          {shapeList.map((shape, index) => (
              <div
                  key={index}
                  className={`shape ${shape.type}`}
                  style={{ left: shape.x, top: shape.y }}
              ></div>
          ))}
      </div>
  );
}
////////////////////////////// CANVAS //////////////////////////////

function App() {
 // chronological list of shape types and (x,y) centers that the user added
 // each element has fields (type, x, y)
 const [shapeList, setShapeList] = useState([]);
 const [realShapeList, setRealShapeList] = useState([]);
 const [selectedShape, setSelectedShape] = useState(null);

 document.body.style = 'background: white;';  // control background color of the webpage

 const handleShapeSelect = (shape) => {
   console.log("shape value in handleShapeSelect:", shape)
   setSelectedShape(shape);
 };

 const handleDoneClick = () => {
   console.log("User clicked on DONE button, shape list looks like:", shapeList);
   console.log("User clicked on DONE button, real shape list looks like:", realShapeList);
 };

 const handleClearClick = () => {
   setShapeList([]);
   setRealShapeList([]);
   console.log("User clicked on CLEAR button, shape list looks like:", shapeList);
 };

  // for robot/ROS:
  const [isConnected, setIsConnected] = useState(false);
  const [liftPosition, setLiftPosition] = useState(0);
  const [wristExtension, setWristExtension] = useState(0);

  // trajectory client
  const [trajectoryClient, setTrajectoryClient] = useState(null);


  const JOINT_LIMITS = {
    "joint_lift": [0.175, 1.05],
    "wrist_extension": [0.05, 0.518]
  }

  // let trajectoryClient = null;
  let jointStateTopic = null;


  useEffect(() => {
    const ros = new ROSLIB.Ros({
      url : "ws://slinky.hcrlab.cs.washington.edu:9090"
    });

    

    // once the ROS connection is made, create the trajectory client
    ros.on('connection', () => {
      console.log("is connected");
      setIsConnected(true);
      

      setTrajectoryClient(new ROSLIB.ActionHandle({
        ros: ros,
        name: '/stretch_controller/follow_joint_trajectory',
        actionType: 'control_msgs/action/FollowJointTrajectory'
      }));
      console.log(trajectoryClient != null);

      jointStateTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/stretch/joint_states',
        messageType: 'sensor_msgs/JointState'
      });

      subscribeToJointState();
      
    });
  }, [])

  // subscribes to the topic that publishes the robot joint position
  const subscribeToJointState = () => {
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
            positions: [liftPosition + 0.1],
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
            positions: [liftPosition - 0.1],
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


  // if (!isConnected) {
  //   return (<div>Loading...</div>)
  // };

 return (
   <div className="App">
            <p>Is connected:</p>
        {isConnected ? "Connected" : "Not connected"}
     <h1>Welcome To Our Robot Coloring Interface!</h1>
     <p>Select a shape option, then click anywhere on the canvas to place it!</p>
     <div>
        <button className="normal-button" onClick={() => moveLift()}>Move Lift To 0.3</button>
        <button className="normal-button" onClick={() => moveLiftUp()}>Move Lift Up</button>
        <button className="normal-button" onClick={() => moveLiftDown()}>Move Lift Down</button>
        <button className="normal-button" onClick={() => moveWristOut()}>Move Wrist Out</button>
        <button className="normal-button" onClick={() => moveWristIn()}>Move Wrist In</button>
        </div>
     <div className="shape-area-container">
     <Canvas selectedShape={selectedShape} shapeList={shapeList} setShapeList={setShapeList} realShapeList={realShapeList} setRealShapeList={setRealShapeList} />
     </div>
     <div className="shape-buttons">
       <p>Shape Options:</p>
       <button className="normal-button" onClick={() => handleShapeSelect('circle')}>Circle</button>
       <button className="normal-button" onClick={() => handleShapeSelect('triangle')}>Triangle</button>
       <button className="normal-button" onClick={() => handleShapeSelect('square')}>Square</button>
     </div>
     <div>
       <p>Actions:</p>
       <button className="normal-button" onClick={handleDoneClick}>Done</button>
       <button className="normal-button" onClick={handleClearClick}>Clear</button>
     </div>
   </div>
 );
}

export default App;