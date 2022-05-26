<script setup>
import LanguageSwitcher from '../LanguageSwitcher.vue';
</script>

<template>
    <div id="nav">
        <div class="logo">
            <img class="logo-img" src="../../assets/images/Logo.svg">
            <div class="logo-title">{{ $t('coinhunter.title') }}</div>
        </div>
        <router-link to="/about">
            {{ $t('about.title') }}
        </router-link>
        <LanguageSwitcher />
    </div>
</template>

<style scoped>
#nav > .locale-changer{
    margin-left: 0;
}

#nav > a {
    margin-left: auto;
}
</style>