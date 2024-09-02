<template>
  <nav class="navbar">
    <div class="navbar-logo">
      <router-link to="/">
        <img src="@/assets/logo.png" alt="Logo" />
      </router-link>
    </div>
    <ul class="navbar-menu">
      <li class="navbar-item">
        <router-link to="/">Home</router-link>
      </li>
      <li class="navbar-item">
        <router-link to="/robot-list">Robot List</router-link>
      </li>
      <li class="navbar-item">
        <router-link to="/about">About</router-link>
      </li>
    </ul>
  </nav>
</template>

<script>
export default {
  name: 'NavBarComponent'
}
</script>

<style scoped>
.navbar {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 10px 20px;
  background-color: #BD462A;
  color: #fff;
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

.navbar-item {
  margin-left: 20px;
}

.navbar-item a {
  color: #fff;
  text-decoration: none;
}
</style>
