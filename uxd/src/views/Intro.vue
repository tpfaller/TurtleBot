<script setup>
import IntroSlider from "../components/IntroSlider.vue"
</script>

<template>
    <section class="content intro">
        <IntroSlider/>
        <router-link to="/game" custom v-slot="{ navigate }">
            <button @click="navigate" @keypress.enter="navigate">{{ $t('button.start') }}</button>
        </router-link>
    </section>
</template>