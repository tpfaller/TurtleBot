<script setup>
import MainNav from './components/navbars/MainNav.vue'
import HomeNav from './components/navbars/HomeNav.vue'
import GameNav from './components/navbars/GameNav.vue'
</script>

<template>
  <component :is="getConditionallyRenderedNavbar"></component>
  <router-view />
</template>

<script>
export default {
  components: {
    MainNav
  },
  computed: {
    getConditionallyRenderedNavbar() {
      if (this.$route.name == "Home") {
        return HomeNav
      } else if (this.$route.name == "About") {
        return MainNav
      } else {
        return GameNav
      }
    }
  }
}
</script>