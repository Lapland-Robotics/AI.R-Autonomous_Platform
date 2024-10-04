

<template>
  <div class="fullscreen-wrapper">
    <div class="video-container">
      <img src="http://snower.local:5000/video_feed">
      <button class="fullscreen-button" @click="toggle">Toggle Fullscreen</button>
    </div> 
  </div>
</template>

<script>
import { api as fullscreen } from 'vue-fullscreen'
export default {
  methods: {
    async toggle () {
      await fullscreen.toggle(this.$el.querySelector('.video-container'), {
        teleport: this.teleport,
      })
      this.fullscreen = fullscreen.isFullscreen
    },
  },
  data() {
    return {
      fullscreen: false,
      teleport: true,
    }
  }
}
</script>

<style scoped>
.video-container {
  
  height: 400px;
  width: 800px;
  position: relative;
  overflow-y: scroll;
  background-color: #f9f9f9;
  border: 1px solid #ccc;
  box-shadow: 0 2px 5px rgba(0, 0, 0, 0.1);  
  overflow: hidden;
  border-bottom-left-radius: 10px;
  border-bottom-right-radius: 10px;
}

.fullscreen-button {
  position: absolute;
  bottom: 0;
  right: 0;
  background-color: #ece9e98a;
  color: white;
  font-size: 16px;
  padding: 12px 12px;
  border: none;
  cursor: pointer;
  border-radius: 5px;
  transition: background-color 0.3s;
}

img {
  width: 100%;
  height: 100%;
  object-fit:fill;
}

.fullscreen-button:hover {
  background-color: black;
}
</style>