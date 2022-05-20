<script setup>
import gameturtlebot from "../assets/gameturtlebot";
</script>

<template>
    <section class="content game">
        <div id="game-interaction">
            <GameCanvas :cellSize="cellSize" :boardSize="boardSize" :speed="speed" :isPlaying="isPlaying" :stop="stop"
                :scores="scores" :obstacles="obstacles" />
        </div>
        <div id="game-instructions">
            <h1 class="title">{{ $t(gameturtlebot[currentStep].title) }}</h1>
            <p class="description">{{ $t(gameturtlebot[currentStep].description) }}</p>
            <img :src="gameturtlebot[currentStep].img">
            <button :class="{ disabled: buttonDisabled, loading: buttonLoading }" :disabled=buttonDisabled @click="nextStep">{{
                    $t(gameturtlebot[currentStep].action)
            }}</button>
        </div>
    </section>
</template>

<script>
import GameCanvas from '../components/GameCanvas/GameCanvas.vue';

export default {
    name: "Game",
    components: {
        GameCanvas
    },
    data() {
        return {
            cellSize: 1,
            boardSize: [1920, 1080],
            speed: 10,
            scores: 0,
            isPlaying: true,
            obstacles: [[0, 0], [100, 150], [1820, 980]],
            currentStep: 0,
            buttonDisabled: false,
            buttonLoading: false
        };
    },
    methods: {
        nextStep() {
            if (this.currentStep + 1 == 1) {
                this.currentStep++;
                this.detectObstacles();
            }
        },
        detectObstacles() {
            //TODO: Obstacle Detection anstossen
            this.buttonLoading = true;
            this.buttonDisabled = true;
            setTimeout(() => this.drawObstacles(), 2000);
        },
        drawObstacles() {
            this.currentStep++;
            this.drawCoins();
        },
        drawCoins() {
            setTimeout(() => this.setOrder(), 2000);
        },
        setOrder(){
            this.buttonLoading = false;
            this.currentStep++;
        }
    }
};
</script>