import WebSocket, { WebSocketServer } from 'ws';
import { randomUUID } from 'crypto';
import mqtt from 'mqtt';

const clients = new Map(); // has to be a Map instead of {} due to non-string keys

const wss = new WebSocketServer({ port: 8080 }); // initiate a new server that listens on port 8080

// Connect to the MQTT broker
const mqttClient = mqtt.connect('mqtt://localhost:1883'); // Adjust the URL/port as necessary

mqttClient.on('connect', function () {
  console.log('Connected to MQTT Broker');

  mqttClient.subscribe('psl/hr', function (err) {
    if (!err) {
      console.log('Subscribed to the topic "psl/hr"');
    }
  });
});
mqttClient.on('message', function (topic, message) {
  console.log(`Received MQTT message: ${message.toString()} on topic: ${topic}`);

  try {
    const data = JSON.parse(message.toString());
    const hrValue = data.hr;

    if (hrValue !== undefined) {
      // Broadcast only the "hr" value to all connected WebSocket clients
      serverBroadcast(`HR: ${hrValue}`);
    }
  } catch (error) {
    console.error('Error parsing MQTT message:', error);
  }
});
// set up event handlers and do other things upon a client connecting to the server
wss.on('connection', (ws) => {
  // create an id to track the client
  const id = randomUUID();
  clients.set(ws, id);
  console.log(`new connection assigned id: ${id}`);

  // send a message to all connected clients upon receiving a message from one of the connected clients
  ws.on('message', (data) => {
    console.log(`received: ${data}`);
    serverBroadcast(`Client ${clients.get(ws)} ${data}`);
  });

  // stop tracking the client upon that client closing the connection
  ws.on('close', () => {
    console.log(`connection (id = ${clients.get(ws)}) closed`);
    clients.delete(ws);
  });

  // send the id back to the newly connected client
  ws.send(`You have been assigned id ${id}`);
});

// send a message to all the connected clients about how many of them there are every 15 seconds
setInterval(() => {
  console.log(`Number of connected clients: ${clients.size}`);
  serverBroadcast(`Number of connected clients: ${clients.size}`);
}, 15000);

// function for sending a message to every connected client
function serverBroadcast(message) {
  wss.clients.forEach((client) => {
    if (client.readyState === WebSocket.OPEN) {
      client.send(message);
    }
  });
}

console.log('The server is running and waiting for connections');