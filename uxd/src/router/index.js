import { createWebHistory, createRouter } from 'vue-router'
import Home from '../views/Home.vue'
import About from '../views/About.vue'
import Intro from '../views/Intro.vue'
import Game from '../views/Game.vue'

const routes = [
    {
        path: '/',
        name: 'Home',
        component: Home
    },
    {
        path: '/about',
        name: 'About',
        component: About
    },
    {
        path: '/intro',
        name: 'Intro',
        component: Intro
    },
    {
        path: '/game',
        name: 'Game',
        component: Game
    }
]
const router = createRouter({
    history: createWebHistory(),
    routes
})

export default router