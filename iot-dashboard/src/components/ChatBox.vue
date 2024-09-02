<template>
  <div class="chatbox">
    <div v-if="connectionStatus" class="status">
      <p>{{ connectionStatus }}</p>
    </div>
    <div v-for="(message, index) in messages" :key="index" class="message">
      <p v-if="message.direction === 'Message Received'"><strong>Partition:</strong> {{ message.partitionId }}</p>
      <p v-if="message.direction === 'Message Received'"><strong>Group:</strong> {{ message.consumerGroup }}</p>
      <p><strong>Message:</strong> {{ message.body }}</p>
      <p>
        <strong>Status: </strong>
        <span :class="message.direction === 'Message Sent' ? 'sent' : 'received'">{{ message.direction }}</span>
      </p>
    </div>
  </div>
</template>

<script>
export default {
  data() {
    return {
      messages: [],
      connectionStatus: 'Connecting...' // Oletustila
    };
  },
  mounted() {
    this.connectWebSocket();
  },
  methods: {
    connectWebSocket() {
      const socket = new WebSocket('ws://localhost:3000');
      socket.onopen = () => {
        this.connectionStatus = 'Connected to WebSocket';
        console.log("WebSocket connection established");
      };
      socket.onmessage = (event) => {
        const message = JSON.parse(event.data);
        console.log('Received message from Event Hub:', message);
        this.messages.push({ ...message, direction: 'Message Received' });
      };
      socket.onerror = (error) => {
        this.connectionStatus = 'WebSocket error';
        console.error('WebSocket error:', error);
      };
      socket.onclose = () => {
        this.connectionStatus = 'WebSocket connection closed';
        console.log("WebSocket connection closed");
      };
    },
    sendMessageToChatbox(message) {
      this.messages.push(message);
    }
  }
};
</script>

<style>
.chatbox {
  border: 1px solid #ccc;
  padding: 10px;
  height: 400px;
  width: 800px;
  overflow-y: scroll;
  background-color: #f9f9f9;
  box-shadow: 0 2px 5px rgba(0, 0, 0, 0.1);  
}
.status {
  background-color: #e0e0e0;
  padding: 5px;
  margin-bottom: 10px;
  text-align: center;
}
.message {
  margin-bottom: 5px; 
  padding: 5px; 
  border-bottom: 1px solid #eee;
  text-align: left; 
}
.message p {
  margin: 2px 0; 
}
.sent {
  color: #BD462A;
}
.received {
  color: #BD462A;
}
</style>

