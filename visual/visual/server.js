const WebSocket = require('ws');
const { SerialPort } = require('serialport');
const { ReadlineParser } = require('@serialport/parser-readline');


const COM_PORT = 'COM5';
const WS_PORT = 8080;


const wss = new WebSocket.Server({ port: WS_PORT });
console.log('WebSocket server is running on ws://localhost:' + WS_PORT);

const port = new SerialPort({
  path: COM_PORT,
  baudRate: 115200,
});
const parser = port.pipe(new ReadlineParser({ delimiter: '\n' }));

console.log('Serial port is open');

let fst_message_received = false;
let fst_message = null;


function parseSerialData(line) {
  const data = JSON.parse(line);
  return {
    roll: (data.roll * Math.PI) / 180,
    pitch: (data.pitch * Math.PI) / 180,
    yaw: (data.yaw * Math.PI) / 180,
    latitude: data.lat,
    longitude: data.lon,
    alt: data.alt
  };
}

parser.on('data', (line) => {

  try {
    if (!fst_message_received) {
      fst_message = parseSerialData(line);
      fst_message_received = true;
    }
    // console.log('Raw data:', line);
    const att = parseSerialData(line);

    // const att = getRandomInRange(-Math.PI, Math.PI, 2); // Convert to radians

    const data = {
      roll: att.roll - fst_message.roll,
      pitch: att.pitch - fst_message.pitch,
      yaw: att.yaw - fst_message.yaw,
      lat: att.lat,
      lon: att.lon,
      alt: att.alt 
    };

    console.log('Parsed data:', data);

    wss.clients.forEach((client) => {
      if (client.readyState === WebSocket.OPEN) {
        client.send(JSON.stringify(data));
      }
    });
  } catch (error) {
    console.error('Error parsing data:', error);
  }
});

wss.on('connection', (ws) => {
  console.log('Client connected');
  ws.on('close', () => console.log('Client disconnected'));
});

function getRandomInRange(min, max, decimals) {
  const num = Math.random() * (max - min) + min;
  return parseFloat(num.toFixed(decimals));
}
