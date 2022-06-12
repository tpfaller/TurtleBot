<script setup>
import MainNav from './components/navbars/MainNav.vue'
import HomeNav from './components/navbars/HomeNav.vue'
import GameNav from './components/navbars/GameNav.vue'
</script>

<template>
  <component :is="getConditionallyRenderedNavbar" :score="score" :playerName="playerName"
    :playerImageNo="playerImageNo"></component>
  <router-view @updateScore="updateScore" @updatePlayerinfo="updatePlayerinfo" />
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
      } else if (this.$route.name == "Game") {
        return GameNav
      } else {
        return MainNav
      }
    }
  },
  data() {
    return {
      score: 0,
      playerName: "",
      playerImageNo: 1
    };
  },
  created() {
    document.documentElement.setAttribute('lang', this.$i18n.locale)
  },
  watch: {
    '$i18n.locale': function (newVal, oldVal) {
      document.documentElement.setAttribute('lang', this.$i18n.locale)
    }
  },
  methods: {
    updateScore(score) {
      this.score = score;
    },
    updatePlayerinfo({ playerName, playerImageNo }) {
      this.playerName = playerName;
      this.playerImageNo = playerImageNo;
    }
  }
}
</script>