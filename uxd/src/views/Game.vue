<script setup>
import gameturtlebot from "../assets/gameturtlebot";
</script>

<template>
    <section class="content game">
        <div id="game-interaction">
            <GameCanvas :cellSize="cellSize" :speed="speed" :isPlaying="isPlaying"
                :scores="scores" :positionData="positionData" :coinSize="coinSize" :coinCount="coinCount" />
        </div>
        <div id="game-instructions">
            <h1 class="title">{{ $t(gameturtlebot[currentStep].title) }}</h1>
            <p class="description">{{ $t(gameturtlebot[currentStep].description) }}</p>
            <img :src="gameturtlebot[currentStep].img">
            <button :class="{ disabled: buttonDisabled, loading: buttonLoading }" :disabled=buttonDisabled
                @click="nextStep">{{
                        $t(gameturtlebot[currentStep].action)
                }}</button>
        </div>
    </section>
</template>

<script>
import GameCanvas from '../components/GameCanvas/GameCanvas.vue';
import PosData from '../components/GameCanvas/exampleinput.json'

export default {
    name: "Game",
    components: {
        GameCanvas
    },
    data() {
        this.initRos()
        return {
            cellSize: 1,
            speed: 10,
            isPlaying: true,
            scores: 0,
            currentStep: 0,
            buttonDisabled: false,
            buttonLoading: false,
            positionData: JSON.parse(JSON.stringify(PosData)),
            coinSize: 50,
            coinCount: 20
        };
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

            var listener = new ROSLIB.Topic({
                ros : ros,
                name : '/test',
                messageType : 'topdown_camera/ObjectPose'
            });

            listener.subscribe(function(message) {
                console.log('Received message on ' + listener.name + ': ' + JSON.stringify(message));
                listener.unsubscribe();
            });
        },
        nextStep() {
            this.setPositionData();
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
        setOrder() {
            this.buttonLoading = false;
            this.currentStep++;
        },
        setPositionData() {
            this.positionData = JSON.parse(JSON.stringify(PosData));
        }
    }
};
</script>