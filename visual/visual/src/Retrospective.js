// Retrospective.js
import React, { useState, useEffect } from "react";
import { Canvas } from "@react-three/fiber";
import RotatingModel from "./RotatingModel";
import PositionPlot from "./PositionPlot";
import "./App.css";
import HeightTimePlot from "./HeightTimePlot";


var PLOTBREAKTIME = 5000;


export default function Retrospective() {
  const [eulerAngles, setEulerAngles] = useState({ x: 0, y: 0, z: 0 });
  const [gpsData, setGpsData] = useState({ latitude: 0, longitude: 0 });
  const [height, setHeight] = useState(0);
  const [fileContent, setFileContent] = useState("");

  const [imu1Data, setImu1Data] = useState([]);
  const [imu2Data, setImu2Data] = useState([]);

  // Read the file and store its content
  useEffect(() => {
    const readData = async () => {
      try {
        const response = await fetch("/cleaned_data.txt");
        if (!response.ok) throw new Error("Failed to fetch the file");
        const text = await response.text();
        setFileContent(text);
      } catch (error) {
        console.error(error);
      }
    };
    readData();
  }, []);

  // Parse the file content into data arrays
  useEffect(() => {
    if (!fileContent) return;

    const imu1DataTemp = [];
    const imu2DataTemp = [];

    const lines = fileContent.split("\n");

    for (let line of lines) {
      if (!line.startsWith("|")) {
        const mainPart = line.split(";")[0];
        const keyValuePairs = mainPart.split(",");

        const result = keyValuePairs.reduce((acc, kv) => {
          const [key, value] = kv.split(":");
          acc[key.toLowerCase()] = parseFloat(value);
          return acc;
        }, {});

        imu1DataTemp.push(result);
      } else {
        line = line.slice(3); // Remove the first three characters
        const mainPart = line.split(";")[0];
        const keyValuePairs = mainPart.split(",");

        const result = keyValuePairs.reduce((acc, kv) => {
          const [key, value] = kv.split(":");
          acc[key.toLowerCase()] = parseFloat(value);
          return acc;
        }, {});

        console.log(result);
        imu2DataTemp.push(result);
      }
    }

    setImu1Data(imu1DataTemp);
    setImu2Data(imu2DataTemp);
  }, [fileContent]);

  // Iterate through the data every 500ms
  useEffect(() => {
    if (imu1Data.length === 0) return;

    let index = 0;
    const intervalId = setInterval(() => {
      if (index < imu2Data.length) {
        const data = imu2Data[index];
        setHeight(data.alt || 0);
        setEulerAngles({
          x: data.pitch || 0,
          y: -data.roll || 0,
          z: data.yaw || 0,
        });
        index++;
      } else {
        clearInterval(intervalId);
      }
    }, 200);

    return () => clearInterval(intervalId);
  }, [imu1Data]);

  return (
    <div
      style={{
        height: "100vh",
        overflow: "hidden",
        display: "flex",
        flexDirection: "column",
      }}
    >
      <h1 className="data-header">Sensors Data</h1>
      <div className="data-container">
        <p className="data">Height: {height}</p>
        <p className="data">x: {eulerAngles.x.toFixed(2)}</p>
        <p className="data">y: {eulerAngles.y.toFixed(2)}</p>
        <p className="data">z: {eulerAngles.z.toFixed(2)}</p>
      </div>
      <div style={{ flex: 1, display: "flex", height: "100vh" }}>
        <div style={{ flex: 1, border: "2px solid black" }}>
          <Canvas camera={{ fov: 45, position: [0, 0, 5] }}>
            <ambientLight intensity={0.5} />
            <pointLight position={[10, 10, 10]} />
            <RotatingModel eulerAngles={eulerAngles} />
          </Canvas>
        </div>
        <div style={{ flex: 1, border: "2px solid black" }}>
          <HeightTimePlot height={height} breaktime={PLOTBREAKTIME} />
        </div>
      </div>
      {/* <div>
        <Canvas camera={{ fov: 60, position: [0, 0, 5] }}>
          <ambientLight intensity={0.5} />
          <pointLight position={[10, 10, 10]} />
          <PositionPlot
            latitude={gpsData.latitude}
            longitude={gpsData.longitude}
            altitude={height}
          />
        </Canvas>
      </div> */}
    </div>
  );
}
