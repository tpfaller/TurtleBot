<script setup>
import LanguageSwitcher from '../controls/LanguageSwitcher.vue'
import SettingsModalSwitcher from '../controls/SettingsModalSwitcher.vue'
import Logo from '../../assets/images/Logo.vue'
</script>

<template>
    <div id="nav">
        <router-link to="/">
            <div class="logo">
                <Logo/>
                <div class="logo-title">{{ $t('coinhunter.title') }}</div>
            </div>
        </router-link>
        <router-link to="/about">
            {{ $t('about.title') }}
        </router-link>
        <div class="controls">
            <LanguageSwitcher />
            <SettingsModalSwitcher />
        </div>
    </div>
</template>