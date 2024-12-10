import React, { useRef } from "react";
import { useFrame } from "@react-three/fiber";
import { Stage } from "@react-three/drei";

export default function RotatingRectangle({ eulerAngles }) {
  const rectangleRef = useRef();

  useFrame(() => {
    if (rectangleRef.current) {
      rectangleRef.current.rotation.x = eulerAngles.x;
      rectangleRef.current.rotation.y = eulerAngles.y;
      rectangleRef.current.rotation.z = eulerAngles.z;
    }
  });

  return (
    <Stage>
      <mesh ref={rectangleRef}>
        <boxGeometry args={[1, .5, 2]} /> 
        <meshStandardMaterial color="green" />
      </mesh>
    </Stage>
  );
}
