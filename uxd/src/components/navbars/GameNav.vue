<script setup>
import LanguageSwitcher from '../LanguageSwitcher.vue';
import HomeIcon from '../icons/IconHome.vue';
</script>

<template>
    <div id="nav">
        <router-link to="/">
            <div class="logo">
                <img class="logo-img" src="../../assets/images/Logo.svg">
                <div class="logo-title">{{ $t('coinhunter.title') }}</div>
            </div>
        </router-link>
        <LanguageSwitcher />
    </div>
</template>