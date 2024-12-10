// App.js
import React, { useState, useEffect } from "react";
import { Canvas } from "@react-three/fiber";
import RotatingModel from "./RotatingModel";
import PositionPlot from "./PositionPlot";

export default function App() {
  const [eulerAngles, setEulerAngles] = useState({ x: 0, y: 0, z: 0 });
  const [gpsData, setGpsData] = useState({ latitude: 0, longitude: 0 });
  const [height, setHeight] = useState(0);
  const [state, setState] = useState(0);

  useEffect(() => {
    const ws = new WebSocket("ws://localhost:8080");
    ws.onmessage = (event) => {
      const data = JSON.parse(event.data);
      setGpsData({
        latitude: data.lat || 0,
        longitude: data.lon || 0,
      });
      setHeight( data.alt || 0);
      setEulerAngles({
        x: data.pitch || 0,
        y: -data.roll || 0,
        z: data.yaw || 0,
      });
      setState({
        state: data.state || 0,
      });
    };

    return () => {
      ws.close();
    };
  }, []);

  return (
    <div>
      <div>
        <h1>GPS Data</h1>
        <p>Latitude: {gpsData.latitude}</p>
        <p>Longitude: {gpsData.longitude}</p>
        <p>Height: {height}</p>
        <p>Rotation:</p>
        <p>x: {eulerAngles.x.toFixed(2)}</p>
        <p>y: {eulerAngles.y.toFixed(2)}</p>
        <p>z: {eulerAngles.z.toFixed(2)}</p>
      </div>
      <div style={{ flex: 1, display: "flex", height:"100vh" }}>
        <div style={{ flex: 1 }}>
          <Canvas camera={{ fov: 45, position: [0, 0, 5] }}>
            <ambientLight intensity={0.5} />
            <pointLight position={[10, 10, 10]} />
            <RotatingModel eulerAngles={eulerAngles} />
          </Canvas>
        </div>
        <div style={{ flex: 1 }}>
          <Canvas camera={{ fov: 60, position: [0, 0, 5] }}>
            <ambientLight intensity={0.5} />
            <pointLight position={[10, 10, 10]} />
            <PositionPlot
              latitude={gpsData.latitude}
              longitude={gpsData.longitude}
              altitude={height}
            />
          </Canvas>
        </div>
      </div>
    </div>
  );
}