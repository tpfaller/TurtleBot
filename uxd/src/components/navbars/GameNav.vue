<script setup>
import LanguageSwitcher from '../LanguageSwitcher.vue';
import HomeIcon from '../icons/IconHome.vue';
</script>

<template>
    <div id="nav">
        <router-link to="/">
            <HomeIcon/>
        </router-link>
        <div class="text-box">{{ $t('turtlebot.title') }}</div>
        <LanguageSwitcher />
    </div>
</template>