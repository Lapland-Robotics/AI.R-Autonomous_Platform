<template>
    <div class="content">
      <h1>Robot List</h1>
      <table>
        <thead>
          <tr>
            <th>Robot</th>
            <th>ROS-version</th>
            <th>On/Off</th>
            <th>View Data</th>
          </tr>
        </thead>
        <tbody>
          <tr v-for="robot in robots" :key="robot.name">
            <td>{{ robot.name }}</td>
            <td>{{ robot.rosVersion }}</td>
            <td><input type="checkbox" :checked="robot.status" disabled></td>
            <td>
              <router-link :to="`/robot/${robot.name.replace(/\s+/g, '-')}`" class="view-data-button">View Data</router-link>
            </td>
          </tr>
        </tbody>
      </table>
  
      <p>Here you can see the list of the current robots available in the Centralized ROS-server. You can check the robots name, ROS-version and current activity status from the table.</p>
      <p>By pressing "View Data" you will be able to view selected robots data in real-time, if the robot is connected to the server and the program is running on the robot's end.</p>
    </div>
  </template>
  
  <script>
  export default {
    name: 'RobotList',
    data() {
      return {
        robots: [
          { name: 'Yahboom Rosmaster X3', rosVersion: 'ROS 2', status: false }, // NOT CONNECTED
          { name: 'Mini-ATV', rosVersion: 'ROS 1', status: false }, // NOT CONNECTED
          { name: 'Big-ATV', rosVersion: 'ROS 1', status: false }, // NOT CONNECTED
          { name: 'Snower', rosVersion: 'ROS 1', status: true },
          { name: 'Yahboom Rosmaster X3 Plus', rosVersion: 'ROS 2', status: false } // NOT CONNECTED
          // Add other robots if needed
        ]
      }
    }
  }
  </script>
  
<style>
        body {
            background-color: white;
            color: #333;
            font-family: Arial, sans-serif;
            margin: 0;
            padding: 0;
        }

        header {
            background-color: #BD462A;
            color: white;
            padding: 20px;
            display: flex;
            justify-content: space-between;
            align-items: center;
        }

        .logo img {
            max-width: 150px; 
            height: 100; /* Säilyttää kuvansuhteen oikein */
            vertical-align: middle;
        }

        nav ul {
            list-style-type: none;
            margin: 0;
            padding: 0;
            display: flex;
        }

        nav ul li {
            margin-right: 20px;
            position: relative;
        }

        nav ul li:after {
            content: "";
            position: absolute;
            width: 100%;
            height: 2px;
            bottom: -5px;
            left: 0;
            opacity: 0;
            transition: opacity 0.3s ease;
        }

        nav ul li:hover:after {
            opacity: 1;
        }

        nav ul li a {
            color: white;
            text-decoration: none;
            font-size: 16px;
        }

        .content {
            padding: 20px;
        }

        table {
            width: 100%;
            border-collapse: collapse;
            margin-bottom: 20px;
        }

        th, td {
            padding: 10px;
            text-align: left;
            border-bottom: 1px solid #ddd;
        }

        th {
            background-color: #f2f2f2;
        }

        input[type="checkbox"] {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
            transform: scale(1.5);
        }

        .view-data-button {
            padding: 8px 16px;
            background-color: #BD462A;
            border: none;
            color: white;
            text-align: center;
            text-decoration: none;
            display: inline-block;
            font-size: 16px;
            margin: 4px 2px;
            cursor: pointer;
            border-radius: 5px;
            
        }

</style>