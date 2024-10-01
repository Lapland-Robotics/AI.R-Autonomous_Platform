require('dotenv').config();
const { EventHubConsumerClient, EventHubProducerClient } = require("@azure/event-hubs");
const WebSocket = require("ws");

// Azure Event Hubin asetukset / Azure Event Hub Settings
const connectionString = process.env.AZURE_CONNECTION_STRING;
const eventHubName = process.env.EVENT_HUB_NAME;
const consumerGroup = process.env.CONSUMER_GROUP || "$Default"; // oletus consumer group

// Luo WebSocket-palvelin / Creating a WebSocket Server
const wss = new WebSocket.Server({ port: 3000 });

wss.on('connection', ws => {
  console.log('Client connected');

  const consumerClient = new EventHubConsumerClient(consumerGroup, connectionString, eventHubName);
  const producerClient = new EventHubProducerClient(connectionString, eventHubName);

  // Event Hubin viestien vastaanotto / Receiving messages from Event Hub
  const subscription = consumerClient.subscribe({
    processEvents: async (events, context) => {
      console.log(`Received ${events.length} event(s)`);
      for (const event of events) {
        const message = {
          body: event.body,
          partitionId: context.partitionId,
          consumerGroup: context.consumerGroup,
          direction: 'Message Received' 
        };
        ws.send(JSON.stringify(message));
        console.log('Message sent to WebSocket client:', message);
      }
    },
    processError: async (err, context) => {
      console.error(`Error: ${err}`);
    }
  });

  // Käsittele viestit asiakkaalta / Process messages from the client
  ws.on('message', async (message) => {
    try {
      const parsedMessage = JSON.parse(message);
      console.log('Received message from client:', parsedMessage);
      // Lähetä viesti Azure Event Hubiin / Send a message to Azure Event Hub
      const batch = await producerClient.createBatch();
      batch.tryAdd({ body: parsedMessage.body });
      await producerClient.sendBatch(batch);
      console.log('Message sent to Azure Event Hub:', parsedMessage.body);

      // Lähetä viesti takaisin WebSocket-asiakkaalle vahvistuksena / Send a message back to WebSocket-client as a confirmation
      ws.send(JSON.stringify({ ...parsedMessage, direction: 'Message Sent' }));
    } catch (err) {
      console.error('Error sending message to Azure Event Hub:', err);
    }
  });

  ws.on('close', () => {
    console.log('Client disconnected');
    subscription.close();
  });
});

console.log('WebSocket server is running on ws://localhost:3000');

