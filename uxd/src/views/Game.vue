<script setup>
import coinhunter_steps from '../assets/coinhunter_steps.js'
import LeaderBoard from '../assets/data/leaderboard.json'
import confetti from '../assets/lottiefiles/confetti.json'
</script>

<template>
    <section class="content game">
        <div id="onscreen-controls">
            <button id="left" @touchstart="move = 37" @touchend="move = null">ᐊ</button>
            <button id="up" @touchstart="move = 38" @touchend="move = null">ᐃ</button>
            <button id="right" @touchstart="move = 39" @touchend="move = null">ᐅ</button>
            <button id="down" @touchstart="move = 40" @touchend="move = null">ᐁ</button>
        </div>
        <div id="game-interaction">
            <GameCanvas @clickedFigure="updateOrder" @collectedCoin="updateScore"
                @updateReachedHeroes="updateReachedHeroes" :cellSize="cellSize" :speed="speed" :isPlaying="isPlaying"
                :score="score" :positionData="positionData" :coinSize="coinSize" :coinCount="coinCount" :order="order"
                :currentStep="currentStep" :move="move" />
        </div>
        <div id="game-instructions">
            <div class="content">
                <h1 class="title">{{ $t(coinhunter_steps[currentStep].title) }}</h1>
                <p class="description">{{ $t(coinhunter_steps[currentStep].description) }}</p>
                <component :is="coinhunter_steps[currentStep].component" :order="order" :reachedHeroes="reachedHeroes"
                    :score="score" :duration="duration" @updateOrder="updateOrder"></component>
            </div>
            <button v-if="showButton" :class="{ disabled: buttonDisabled, loading: buttonLoading }"
                :disabled=buttonDisabled @click="nextStep">{{
                        $t(coinhunter_steps[currentStep].action)
                }}</button>
        </div>
    </section>
    <div id="confetti" :class="visibility">
        <lottie-animation ref="anim" :loop="false" :animationData="confetti" :autoPlay="false" />
    </div>
</template>

<script>
import GameCanvas from '../components/GameCanvas/GameCanvas.vue'
import PosData from '../components/GameCanvas/exampleinput.json'
import OrderSet from '../components/Game/OrderSet.vue'
import OrderView from '../components/Game/OrderView.vue'
import Results from '../components/Game/Results.vue'

import { uniqueNamesGenerator, adjectives, colors, animals } from 'unique-names-generator';

var audio_coin = new Audio('/src/assets/audio/coin.mp3');
var audio_hero = new Audio('/src/assets/audio/hero.mp3');
var audio_finish = new Audio('/src/assets/audio/finished.wav');

export default {
    name: "Game",
    components: {
        GameCanvas
    },
    data() {
        return {
            cellSize: 1,
            speed: 10,
            isPlaying: true,
            playerName: String,
            playerImageNo: 3,
            score: 0,
            currentStep: 0,
            showButton: true,
            buttonDisabled: false,
            buttonLoading: false,
            positionData: JSON.parse(JSON.stringify(PosData)),
            coinSize: 40,
            coinCount: 20,
            current: "order",
            order: [],
            reachedHeroes: [false, false, false],
            startTime: Date,
            endTime: Date,
            duration: String,
            get muted() {
                return localStorage.getItem('muted') || "false";
            },
            set muted(value) {
                localStorage.setItem('muted', value);
            },
            leaderBoardData: JSON.parse(JSON.stringify(LeaderBoard)),
            visibility: "invisible",
            move: 0,
            sound: null
        };
    },
    mounted() {
        this.generatePlayerInfo();
        this.showButton = coinhunter_steps[this.currentStep].button == 1;
        this.saveLeaderboard();
        this.prepareAudio();
    },
    methods: {
        generatePlayerInfo() {
            this.playerName = uniqueNamesGenerator({
                dictionaries: [adjectives, colors, animals],
                separator: '',
                style: 'capital'
            });
            this.playerImageNo = Math.floor(Math.random() * 4) + 1;
            this.$emit('updatePlayerinfo', { playerName: this.playerName, playerImageNo: this.playerImageNo });
        },
        updateOrder(newOrder) {
            this.order = newOrder;
            console.log("parent, order: " + this.order);
            if (this.order.length == 3) {
                this.buttonDisabled = false;
            }
        },
        updateReachedHeroes(newReachedHeroes) {
            console.log("updated" + newReachedHeroes);
            this.reachedHeroes = newReachedHeroes;
            if (this.reachedHeroes.every(Boolean)) {
                this.endTime = new Date();
                //audio_finish.currentTime = 0;
                //audio_finish.play();
                this.sound.currentTime = 0;
                this.sound.src="/src/assets/audio/finished.wav";
                this.visibility = "";
                this.$refs.anim.play();
                this.calculateDuration();
                this.saveLeaderboard();
                this.nextStep();
            } else if (this.muted == 'false') {
                //audio_hero.currentTime = 0;
                //audio_hero.play();
                this.sound.currentTime = 0;
                this.sound.src="/src/assets/audio/hero.mp3";
            }
        },
        saveLeaderboard() {
            let newId = this.leaderBoardData[this.leaderBoardData.length - 1].id + 1;
            for (var i = 0; i < this.leaderBoardData.length; i++) {
                if (this.leaderBoardData[i].coins < 11) {
                    this.leaderBoardData.splice(i, 0, { id: newId, name: this.playerName, player: 1, time: this.startTime, coins: this.score, duration: this.duration })
                    console.log("insert index: " + i + ", newid: " + newId);
                    console.log("insert index: " + JSON.stringify(this.leaderBoardData));
                    break;
                }
            }
        },
        calculateDuration() {
            var diffTime = (this.endTime - this.startTime);
            let days = diffTime / (24 * 60 * 60 * 1000);
            let hours = (days % 1) * 24;
            let minutes = (hours % 1) * 60;
            let secs = (minutes % 1) * 60;
            this.duration = Math.floor(minutes).toString().padStart(2, '0') + ":" + Math.floor(secs).toString().padStart(2, '0');
        },
        updateScore(newScore) {
            this.score = newScore;
            this.$emit('updateScore', this.score);
            if (this.muted == 'false') {
                //audio_coin.currentTime = 0;
                //audio_coin.play();
                this.sound.currentTime = 0;
                this.sound.src = "/src/assets/audio/coin.mp3";
            }
        },
        nextStep() {
            console.log("current step: " + this.currentStep);
            this.setPositionData();
            if (this.currentStep + 1 == 1) {
                this.sound.src = "data:audio/mpeg;base64,SUQzBAAAAAABEVRYWFgAAAAtAAADY29tbWVudABCaWdTb3VuZEJhbmsuY29tIC8gTGFTb25vdGhlcXVlLm9yZwBURU5DAAAAHQAAA1N3aXRjaCBQbHVzIMKpIE5DSCBTb2Z0d2FyZQBUSVQyAAAABgAAAzIyMzUAVFNTRQAAAA8AAANMYXZmNTcuODMuMTAwAAAAAAAAAAAAAAD/80DEAAAAA0gAAAAATEFNRTMuMTAwVVVVVVVVVVVVVUxBTUUzLjEwMFVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVf/zQsRbAAADSAAAAABVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVf/zQMSkAAADSAAAAABVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV";
                this.currentStep++;
                this.detectObstacles();
            } else if (this.currentStep == 3) {
                this.currentStep++;
                this.startTime = new Date();
                console.log("Starting. Time: " + this.startTime);
            } else if (this.currentStep == 4) {
                this.currentStep++;
            } else if (this.currentStep == 5) {
                this.currentStep++;
            } else if (this.currentStep == 6) {
                this.$router.push({ name: 'Leaderboard' })
            }
            this.showButton = coinhunter_steps[this.currentStep].button == 1;
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
        setOrder() {
            this.buttonLoading = false;
            this.currentStep++;
        },
        setPositionData() {
            this.positionData = JSON.parse(JSON.stringify(PosData));
        },
        prepareAudio() {
            this.sound = new Audio();
            this.sound.autoplay = true;
        }
    },
    components: {
        setOrder: OrderSet,
        viewOrder: OrderView,
        results: Results
    }
};
</script>
