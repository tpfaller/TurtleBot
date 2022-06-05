<script setup>
import LeaderBoard from '../assets/data/leaderboard.json'
</script>

<template>
    <section class="content leaderboard">
        <div class="winner">
            <div v-for="(player, index) in LeaderBoard.slice(0, 3)" :class="`rank-${index + 1}`">
                <img src="src/assets/images/Player/Player_1.svg">
                <div>{{ player.name }}</div>
                <div class="coins">{{ player.coins }}</div>
            </div>
        </div>
        <div class="list">
            <div v-for="(player, i) in LeaderBoard.slice(3)">
                <div>{{ i + 4 }}</div>
                <img src="src/assets/images/Player/Player_1.svg">
                <div>{{ player.name }}</div>
                <div class="coins">{{ player.coins }}</div>
            </div>
        </div>
        <div class="button-container">
            <router-link to="/" custom v-slot="{ navigate }">
                <button @click="navigate" @keypress.enter="navigate">{{ $t('home.home') }}</button>
            </router-link>
            <router-link to="/game" custom v-slot="{ navigate }">
                <button @click="navigate" @keypress.enter="navigate">{{ $t('button.restart') }}</button>
            </router-link>
        </div>
    </section>
</template>

<script>
var audio_leaderboard = new Audio('/src/assets/audio/leaderboard.mp3');

export default {
    mounted() {
        audio_leaderboard.currentTime = 0;
        audio_leaderboard.play();
    }
}
</script>