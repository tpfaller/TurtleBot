<script setup>
import LanguageSwitcher from '../controls/LanguageSwitcher.vue'
import Logo from '../../assets/images/Logo.vue'
import SettingsModalSwitcher from '../controls/SettingsModalSwitcher.vue';
</script>

<template>
    <div id="nav">
        <router-link to="/">
            <div class="logo">
                <Logo/>
                <div class="logo-title">{{ $t('coinhunter.title') }}</div>
            </div>
        </router-link>
        <div class="controls">
            <LanguageSwitcher />
            <SettingsModalSwitcher/>
        </div>
    </div>
</template>