<template>
  <div class="container">
    <h1>{{ robotName }} Robot Data Center</h1>
    
    <div class="content">
      <p>Welcome to {{ robotName }} Robot Data Center! Here you can monitor {{ robotName }} robot's data in real time and give it commands.</p>
    </div>
    
    <div id="chat-container">
      <ChatBox ref="chatbox"></ChatBox>
      <TextFieldButton @message-sent="handleMessageSent"></TextFieldButton>
    </div>
  </div>
</template>

<script>
import ChatBox from '@/components/ChatBox.vue';
import TextFieldButton from '@/components/TextFieldButton.vue';

export default {
  name: 'Yahboom-Rosmaster-X3',
  components: {
    ChatBox,
    TextFieldButton
  },
  props: {
    name: {
      type: String,
      required: true
    }
  },
  computed: {
    robotName() {
      return this.name.replace('-', ' ');
    }
  },
  methods: {
    handleMessageSent(message) {
      this.$refs.chatbox.sendMessageToChatbox(message);
    }
  }
}
</script>

<style scoped>
.container {
  display: flex;
  flex-direction: column;
  align-items: center;
  padding: 20px;
}

#chat-container {
  display: flex;
  flex-direction: column;
  align-items: center;
  margin-top: 20px;
}

#input-container {
  display: flex;
  align-items: center;
  margin-top: 10px; /* Vähän tilaa chatboxin alapuolelle */
  width: 100%; /* Leveys ottaa huomioon marginaalin */
  max-width: 800px; /* Maksimileveys syöttökenttä ja nappi */
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
