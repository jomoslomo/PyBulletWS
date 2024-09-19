const WebSocket = require('ws');
const crypto = require('crypto');
const readline = require('readline');

const clients = new Map();
const wss = new WebSocket.Server({ port: 8080 });

// Set up readline interface for keyboard input
const rl = readline.createInterface({
  input: process.stdin,
  output: process.stdout
});

wss.on('connection', (ws) => {
  const id = crypto.randomUUID();
  clients.set(ws, id);
  console.log(`New connection assigned id: ${id}`);

  ws.on('message', (data) => {
    console.log(`Received from client ${id}: ${data}`);
  });

  ws.on('close', () => {
    console.log(`Connection (id = ${clients.get(ws)}) closed`);
    clients.delete(ws);
  });

  ws.send(`You have been assigned id ${id}`);
});

function serverBroadcast(message) {
  wss.clients.forEach((client) => {
    if (client.readyState === WebSocket.OPEN) {
      client.send(message);
    }
  });
}

// Handle keyboard input
rl.on('line', (input) => {
  if (input.toLowerCase() === 'q') {
    console.log('Closing server...');
    wss.close();
    rl.close();
    process.exit(0);
  } else {
    console.log(`Sending command: ${input}`);
    serverBroadcast(input);
  }
});

console.log('WebSocket server is running on port 8080');
console.log('Enter commands (up, down, left, right, w, s) to control the robot.');
console.log('Enter "q" to quit the server.');