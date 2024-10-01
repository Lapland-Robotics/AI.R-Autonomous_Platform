<template>
    <div style="height:400px; width:800px">
      <l-map ref="map" v-model:zoom="zoom" :center="[47.41322, -1.219482]">
        <l-tile-layer
          url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
          layer-type="base"
          name="OpenStreetMap"
        ></l-tile-layer>
      </l-map>
    </div>
</template>

<script>
import "leaflet/dist/leaflet.css";
import { LMap, LTileLayer } from "@vue-leaflet/vue-leaflet";

export default {
components: {
    LMap,
    LTileLayer,
},
data() {
    return {
    coordinates: [],
    zoom: 15,
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
        console.log(event);
        const message = JSON.parse(event.data);
        console.log('Received message:', message);
        this.coordinates.push(message.coordinates);
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


<style></style>