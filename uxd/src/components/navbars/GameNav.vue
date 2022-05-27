<script setup>
import LanguageSwitcher from '../LanguageSwitcher.vue';
import VolumeSwitcher from '../VolumeSwitcher.vue';
</script>

<template>
    <div id="nav">
        <router-link to="/">
            <div class="logo">
                <img class="logo-img" src="../../assets/images/Logo.svg">
                <div class="logo-title">{{ $t('coinhunter.title') }}</div>
            </div>
        </router-link>
        <div class="player-info">
            <div class="player">
                <div><img :src="playerImage"></div>
                {{ playerName }}
            </div>
            <div class="points">
                {{ score }}
            </div>
        </div>
        <VolumeSwitcher @volumeChanged="changeVolume"/>
        <LanguageSwitcher />
    </div>
</template>

<script>
export default {
    props: {
        score: Number,
        playerName: String,
        playerImageNo: Number,
        muted: Boolean
    },
    data() {
        return {
            playerImage: "src/assets/images/Player/Player_" + this.playerImageNo + ".svg"
        }
    },
    methods:{
        changeVolume(volume){
            this.$emit('volumeChanged', volume);
        }
    }
}
</script>