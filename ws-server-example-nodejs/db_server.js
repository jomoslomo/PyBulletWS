import WebSocket, { WebSocketServer } from 'ws';
import { randomUUID } from 'crypto';
import sqlite3 from 'sqlite3';

const clients = new Map();
const wss = new WebSocketServer({ port: 8080 });

// Initialize SQLite database
const db = new sqlite3.Database('./messages.db', (err) => {
  if (err) {
    console.error('Error opening database', err);
  } else {
    console.log('Connected to the SQLite database.');
    db.run(`CREATE TABLE IF NOT EXISTS messages (
      id INTEGER PRIMARY KEY AUTOINCREMENT,
      timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
      message TEXT
    )`);
  }
});

wss.on('connection', (ws) => {
  const id = randomUUID();
  clients.set(ws, id);
  console.log(`New connection assigned id: ${id}`);

  ws.on('message', (data) => {
    console.log(`Received: ${data}`);
    const message = `Client ${id}: ${data}`;
    
    // Store the message in the database
    db.run(`INSERT INTO messages (message) VALUES (?)`, [message], function(err) {
      if (err) {
        return console.error('Error inserting into database', err);
      }
      console.log(`Message stored in database with ID: ${this.lastID}`);
    });

    // Broadcast the message to all clients
    serverBroadcast(message);
  });

  ws.on('close', () => {
    console.log(`Connection (id = ${id}) closed`);
    clients.delete(ws);
  });

  // Send welcome message
  ws.send(`You have been assigned id ${id}`);

  // Send the last 5 messages from the database
  db.all(`SELECT message FROM messages ORDER BY timestamp DESC LIMIT 5`, [], (err, rows) => {
    if (err) {
      console.error('Error querying database', err);
    } else {
      const messages = rows.map(row => row.message).reverse();
      ws.send(`Last 5 messages:\n${messages.join('\n')}`);
    }
  });
});

function serverBroadcast(message) {
  wss.clients.forEach((client) => {
    if (client.readyState === WebSocket.OPEN) {
      client.send(message);
    }
  });
}

console.log('The server is running and waiting for connections on port 8080');

// Properly close the database connection when the server is shutting down
process.on('SIGINT', () => {
  db.close((err) => {
    if (err) {
      console.error(err.message);
    }
    console.log('Closed the database connection.');
    process.exit(0);
  });
});