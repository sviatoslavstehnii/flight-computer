import React, { useEffect, useState } from "react";
import { Line } from "react-chartjs-2";
import { Chart as ChartJS, LineElement, CategoryScale, LinearScale, PointElement } from "chart.js";

ChartJS.register(LineElement, CategoryScale, LinearScale, PointElement);
var millisecs = 0;
const HeightTimePlot = ({ height, breaktime=-1 }) => {
  const [dataPoints, setDataPoints] = useState([]);
  const [timeStamps, setTimeStamps] = useState([]);

  useEffect(() => {
    const interval = setInterval(() => {
      setDataPoints((prevDataPoints) => [...prevDataPoints, height].slice(-50)); // Keep the last 50 points
      setTimeStamps((prevTimeStamps) => [...prevTimeStamps, millisecs].slice(-50)); // Keep the last 50 points
      millisecs += 100;
      if (breaktime > 0 && millisecs >= breaktime) {
        clearInterval(interval);
      }
    }, 100); // Update every second

    return () => clearInterval(interval); // Cleanup
  }, [height]);

  const data = {
    labels: timeStamps,
    datasets: [
      {
        label: "Height over Time",
        data: dataPoints,
        fill: false,
        borderColor: "rgba(75,192,192,1)",
        tension: 0.1,
      },
    ],
  };

  const options = {
    responsive: true,
    maintainAspectRatio: true,
    scales: {
      x: {
        title: { display: true, text: "Time (ms)" },
      },
      y: {
        title: { display: true, text: "Height (m)" },
      },
    },
  };

  return (
    <div style={{ width: "100%", height: "100%" }}>
      <Line data={data} options={options} />
    </div>
  );
};

export default HeightTimePlot;
