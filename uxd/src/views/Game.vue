<script setup>
import coinhunter_steps from "../assets/coinhunter_steps";
</script>

<template>
    <section class="content game">
        <div id="game-interaction">
            <GameCanvas ref="gameCanvas" @clickedFigure="updateOrder" @collectedCoin="updateScore"
                @updateReachedHeroes="updateReachedHeroes" :cellSize="cellSize" :speed="speed" :isPlaying="isPlaying"
                :score="score" :positionData="positionData" :coinSize="coinSize" :coinCount="coinCount" :order="order"
                :step="currentStep" />
        </div>
        <div id="game-instructions">
            <div class="content">
                <h1 class="title">{{ $t(coinhunter_steps[currentStep].title) }}</h1>
                <p class="description">{{ $t(coinhunter_steps[currentStep].description) }}</p>
                <component :is="coinhunter_steps[currentStep].component" :order="order" :reachedHeroes="reachedHeroes"
                    :score="score" :duration="duration"></component>
            </div>
            <button v-if="showButton" :class="{ disabled: buttonDisabled, loading: buttonLoading }"
                :disabled=buttonDisabled @click="nextStep">{{
                        $t(coinhunter_steps[currentStep].action)
                }}</button>
        </div>
    </section>
</template>

<script>
import GameCanvas from '../components/GameCanvas/GameCanvas.vue';
import PosData from '../components/GameCanvas/exampleinput.json';
import OrderSet from '../components/Game/OrderSet.vue'
import OrderView from '../components/Game/OrderView.vue'
import Results from '../components/Game/Results.vue'

var audio = new Audio('/src/assets/audio/coin.mp3');

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
            playerName: "Player 2",
            playerImageNo: 3,
            score: 0,
            currentStep: 0,
            showButton: true,
            buttonDisabled: false,
            buttonLoading: false,
            positionData: JSON.parse(JSON.stringify(PosData)),
            coinSize: 50,
            coinCount: 20,
            ros: this.initRos(),
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
            }
        };
    },
    mounted() {
        this.playerImageNo = Math.floor(Math.random() * 4) + 1;
        this.$emit('updatePlayerinfo', { playerName: this.playerName, playerImageNo: this.playerImageNo });
        this.showButton = coinhunter_steps[this.currentStep].button == 1;
    },
    methods: {
        initRos() {
            var ros = new ROSLIB.Ros({
                url : 'ws://localhost:9090'
            });

            ros.on('connection', function() {
                console.log('Connected to websocket server.');
            });

            ros.on('error', function(error) {
                console.log('Error connecting to websocket server: ', error);
            });

            ros.on('close', function() {
                console.log('Connection to websocket server closed.');
            });

            var objectListener = new ROSLIB.Topic({
                ros : ros,
                name : '/game_objects',
                messageType : 'topdown_camera/ObjectPoseArray'
            });

            objectListener.subscribe(this.handle_obj_message);

            return ros
        },
        handle_obj_message(message) {
            var objects = {}
            const width = this.positionData["Spielfeld"][0]
            const height = this.positionData["Spielfeld"][1]
            message.objects.forEach(obj => {
                var position = [
                    Math.floor((obj.x - obj.width / 2) * width), Math.floor((obj.y - obj.height / 2) * height),
                    Math.floor(obj.width * width), Math.floor(obj.height * height)
                ]
                objects[obj.obj_id] = position
            });
            this.$refs.gameCanvas.handleObjectPositions(objects)
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
                this.calculateDuration();
                this.nextStep();
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
                audio.currentTime = 0;
                audio.play();
            }
        },
        nextStep() {
            console.log("current step: " + this.currentStep);
            this.setPositionData();
            if (this.currentStep + 1 == 1) {
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
        }
    },
    components: {
        setOrder: OrderSet,
        viewOrder: OrderView,
        results: Results
    }
};
</script>