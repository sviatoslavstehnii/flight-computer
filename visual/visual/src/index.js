import React from "react";
import ReactDOM from "react-dom/client";
import "./index.css";
import App from "./App";
import Retrospective from "./Retrospective";

const root = ReactDOM.createRoot(document.getElementById("root"));
root.render(
  <React.StrictMode>
    <Retrospective />
  </React.StrictMode>
);
