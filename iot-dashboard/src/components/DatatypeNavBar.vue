<template>
  <nav class="navbar">
    <div class="navbar-logo">
    </div>
    <ul class="navbar-menu">
      <li 
        class="navbar-item messages" 
        :class="{ active: activeItem === 'Messages' }" 
        @click="setActiveItem('Messages')"
      >
        Messages
      </li>
      <li 
        class="navbar-item camera-feed" 
        :class="{ active: activeItem === 'Camera feed' }" 
        @click="setActiveItem('Camera feed')"
      >
        Camera feed
      </li>
      <li 
        class="navbar-item gps" 
        :class="{ active: activeItem === 'GPS' }" 
        @click="setActiveItem('GPS')"
      >
        GPS
      </li>
    </ul>
  </nav>
</template>

<script>
export default {
  name: 'DatatypeNavBar',
  data() {
    return {
      activeItem: 'Messages' // Default active item
    };
  },
  methods: {
    setActiveItem(item) {
      this.activeItem = item;
      this.$emit('active-tab-changed', item);

    }
  },
}
</script>

<style scoped>
.navbar {
  display: flex;
  justify-content: center;
  align-items: center;
  background-color: #BD462A;
  color: #fff;
  width: 100%;
  border-top-left-radius: 10px;
  border-top-right-radius: 10px;
}

.navbar-logo img {
  height: 60px;
}

.navbar-menu {
  display: flex;
  list-style: none;
  margin: 0;
  padding: 0;
}


.navbar-menu .navbar-item {
  margin:0;
  padding: 20px;
  border-right-style: solid;
  border-left-style: solid;
  border-color: transparent;  
}

.navbar-item {
  transition: background-color 0.3s;
}

.navbar-item:hover {
  cursor: pointer;
  background-color: #df5b3e;
}

.navbar-item.active {
  border-color: #ffffffcb;
  background-color: #df5b3e;
}

.navbar-item a:hover {
  color: #ffffff23;
  text-decoration: none;
}

</style>