import { createRouter, createWebHistory } from 'vue-router'
import RobotList from '../views/RobotList.vue'
import RobotDetail from '../views/RobotDetail.vue'
import FrontPage from '@/views/FrontPage.vue'
import AboutView from '@/views/AboutView.vue'

const routes = [
  {
    path: '/',
    name: 'Home',
    component: FrontPage
  },
  {
    path: '/robot-list',
    name: 'RobotList',
    component: RobotList
  },
  {
    path: '/robot/:name',
    name: 'RobotDetail',
    component: RobotDetail,
    props: true
  },
  {
    path: '/about',
    name: 'About',
    component: AboutView
  }
]

const router = createRouter({
  history: createWebHistory(process.env.BASE_URL),
  routes
})

export default router
