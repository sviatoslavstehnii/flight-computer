// PositionPlot.js
import React, { useRef } from "react";
import { useFrame } from "@react-three/fiber";
import { OrbitControls } from "@react-three/drei";

export default function PositionPlot({ latitude, longitude, altitude }) {
  const pointRef = useRef();
  const trailRef = useRef([]);
  const maxTrailLength = 100;

  const convertToCartesian = (lat, lon, alt) => {
    const radius = 6371 + alt / 1000; // Earth radius in km + altitude in km
    const phi = (90 - lat) * (Math.PI / 180);
    const theta = (lon + 180) * (Math.PI / 180);

    const x = radius * Math.sin(phi) * Math.cos(theta);
    const z = radius * Math.sin(phi) * Math.sin(theta);
    const y = radius * Math.cos(phi);

    return [x / 1000, y / 1000, z / 1000]; // Scale down for visualization
  };

  useFrame(() => {
    const [x, y, z] = convertToCartesian(latitude, longitude, altitude);
    if (pointRef.current) {
      pointRef.current.position.set(x, y, z);

      // Update trail
      if (trailRef.current.length >= maxTrailLength) {
        trailRef.current.shift();
      }
      trailRef.current.push([x, y, z]);
    }
  });

  return (
    <>
      <axesHelper args={[5]} />

      <mesh ref={pointRef}>
        <sphereGeometry args={[0.1, 16, 16]} />
        <meshStandardMaterial color="red" />
      </mesh>

      {trailRef.current.length > 1 && (
        <line>
          <bufferGeometry>
            <bufferAttribute
              attach="attributes-position"
              count={trailRef.current.length}
              array={new Float32Array(trailRef.current.flat())}
              itemSize={3}
            />
          </bufferGeometry>
          <lineBasicMaterial color="blue" />
        </line>
      )}

      <OrbitControls />
    </>
  );
}