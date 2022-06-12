import { createApp } from 'vue'
import App from './App.vue'
import router from './router'
import './assets/css/main.css'
import { createI18n } from "vue-i18n"
import de from './locales/de.json'
import en from './locales/en.json'
import LottieAnimation from "lottie-web-vue"

const i18n = createI18n({
  locale: 'de',
  messages: {
    de,
    en,
  }
})

createApp(App).use(router).use(i18n).use(LottieAnimation).mount('#app')