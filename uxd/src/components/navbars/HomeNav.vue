<script setup>
import LanguageSwitcher from '../LanguageSwitcher.vue';
</script>

<template>
    <div id="nav">
        <div class="logo">
            <img class="logo-img" src="../../assets/images/Logo.svg">
            <div class="logo-title">{{ $t('coinhunter.title') }}</div>
        </div>
        <LanguageSwitcher />
        <router-link to="/about">
            {{ $t('about.title') }}
        </router-link>
    </div>
</template>