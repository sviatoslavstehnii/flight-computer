import React, { useRef } from "react";
import { useFrame, useLoader } from "@react-three/fiber";
import { OBJLoader, MTLLoader } from "three-stdlib";
import { Stage } from "@react-three/drei";

export default function RotatingModel({ eulerAngles }) {
  const material = useLoader(MTLLoader, "/model.mtl"); // Load the MTL file
  const obj = useLoader(OBJLoader, "/model.obj", (loader) => {
    material.preload();
    loader.setMaterials(material); // Apply the materials to the OBJ loader
  });

  const modelRef = useRef(); // Create a ref for the model

  useFrame(() => {
    if (modelRef.current) {
      modelRef.current.rotation.x = eulerAngles.x;
      modelRef.current.rotation.y = eulerAngles.y;
      modelRef.current.rotation.z = eulerAngles.z;
    }
  });

  return (
    <Stage>
      <primitive object={obj} ref={modelRef} /> {/* Attach the ref */}
    </Stage>
  );
}
