<template>
  <div id="input-container">
    <input type="text" id="messageInput" v-model="message" placeholder="Type your message...">
    <button @click="sendMessage">Send</button>
  </div>
</template>

<script>
export default {
  data() {
    return {
      socket: null,
      message: ''
    };
  },
  mounted() {
    this.connectWebSocket();
  },
  methods: {
    connectWebSocket() {
      this.socket = new WebSocket('ws://localhost:3000');
      this.socket.onopen = () => {
        console.log('WebSocket connection established in TextFieldButton');
      };
      this.socket.onclose = () => {
        console.log('WebSocket connection closed in TextFieldButton');
      };
    },
    sendMessage() {
      if (this.socket && this.message) {
        const messageObject = { body: this.message, direction: 'Message Sent' };
        this.socket.send(JSON.stringify(messageObject));

        // Lähetä viesti myös ChatBox-komponentille
        this.$emit('message-sent', messageObject);

        this.message = ''; // Tyhjennä tekstikenttä lähettämisen jälkeen
        console.log('Message sent');
      }
    }
  }
};
</script>
  
  <style scoped>
  #input-container {
    display: flex;
    align-items: center;
    margin-top: 10px;
    width: 100%;
    max-width: 800px;
  }

  #messageInput {
  flex: 1; /* Tekstikenttä vie kaiken tilan nappiin verrattuna */
  padding: 10px;
  border: 1px solid #ccc;
  }

  button {
  margin-left: 10px; /* Tilaa tekstikentän ja napin väliin */
  padding: 10px 20px;
  border: none;
  color: white;
  cursor: pointer;

  display: inline-block;
  padding: 10px 20px;
  background-color: #BD462A;
  text-decoration: none;
  border-radius: 4px;
  }   

  button:hover {
  background-color: #611e0f;
  }
</style>
  