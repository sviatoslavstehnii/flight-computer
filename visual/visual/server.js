const WebSocket = require('ws');
const { SerialPort } = require('serialport');
const { ReadlineParser } = require('@serialport/parser-readline');

// Setup WebSocket server
const wss = new WebSocket.Server({ port: 8080 });
console.log('WebSocket server is running on ws://localhost:8080');

// Setup Serial connection
const port = new SerialPort({
  path: 'COM5', // Replace with your Teensy's port
  baudRate: 115200,
});
const parser = port.pipe(new ReadlineParser({ delimiter: '\n' }));

console.log('Serial port is open');

// Parse the serial data into roll, pitch, yaw in radians
function parseSerialData(line) {
  const data = JSON.parse(line);
  const att = {
    roll: (data.roll * Math.PI) / 180,
    pitch: (data.pitch * Math.PI) / 180,
    yaw: (data.yaw * Math.PI) / 180,
    latitude: data.lat,
    longitude: data.lon,
    alt: data.alt
  };
  return att;
}

// Listen to Serial data
parser.on('data', (line) => {
  try {
    console.log('Raw data:', line);
    // const att = parseSerialData(line); // Convert to radians
    const att = getRandomInRange(-Math.PI, Math.PI, 2); // Convert to radians
    console.log('Parsed data (radians):', att);

    // Add random GPS and alt data
    const data = {
      roll: att.roll,
      pitch: att.pitch,
      yaw: att.yaw,
      lat: att.lat,
      lon: att.lon,
      alt: att.alt,
    };

    // Broadcast data to all WebSocket clients
    wss.clients.forEach((client) => {
      if (client.readyState === WebSocket.OPEN) {
        client.send(JSON.stringify(data));
      }
    });
  } catch (error) {
    console.error('Error parsing data:', error);
  }
});

// WebSocket connection handling
wss.on('connection', (ws) => {
  console.log('Client connected');
  ws.on('close', () => console.log('Client disconnected'));
});

// Function to generate random GPS coordinates and alt
function getRandomInRange(min, max, decimals) {
  const num = Math.random() * (max - min) + min;
  return parseFloat(num.toFixed(decimals));
}
