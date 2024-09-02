<template>
    <div class="container">
      <div class="white-box">
        <h1>Welcome to the Centralized ROS-server!</h1>
        <p>You can now select your desired robot from the navigation menu and begin working immediately.</p>
        <router-link to="/robot-list" class="button">To The Robot List</router-link>
      </div>
    </div>
  </template>
  
  <script>
  export default {
    name: 'FrontPage'
  }
  </script>
  
  <style scoped>
  .container {
    display: flex;
    justify-content: center;
    align-items: center;
    height: 100vh;
  }
  
  .white-box {
    background-color: white;
    padding: 40px; /* Suurentaa paddingia */
    border-radius: 8px;
    text-align: center;
    box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);
    max-width: 1200px; /* Maksimi leveys */
  }
  
  .button {
    display: inline-block;
    padding: 10px 20px;
    margin-top: 20px;
    background-color: #BD462A;
    color: white;
    text-decoration: none;
    border-radius: 4px;
  }
  
  .button:hover {
    background-color: #96311b;
  }
  </style>
  