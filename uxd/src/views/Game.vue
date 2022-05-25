<script setup>
import gameturtlebot from "../assets/gameturtlebot";
</script>

<template>
    <section class="content game">
        <div id="game-interaction">
            <GameCanvas ref="gameCanvas" :cellSize="cellSize" :speed="speed" :isPlaying="isPlaying" :scores="scores"
                :positionData="positionData" :coinSize="coinSize" :coinCount="coinCount" />
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
            coinCount: 20,
            ros: this.initRos()
        };
    },
    methods: {
        initRos() {
            var ros = new ROSLIB.Ros({
                url: 'ws://localhost:9090'
            });

            ros.on('connection', function () {
                console.log('Connected to websocket server.');
            });

            ros.on('error', function (error) {
                console.log('Error connecting to websocket server: ', error);
            });

            ros.on('close', function () {
                console.log('Connection to websocket server closed.');
            });

            var objectListener = new ROSLIB.Topic({
                ros: ros,
                name: '/game_objects',
                messageType: 'topdown_camera/ObjectPoseArray'
            });

            objectListener.subscribe(this.handle_obj_message);

            var goalQueue = ['Captain_America', 'Hulk', 'Iron_Man'];
            this.goals(ros, goalQueue);

            return ros
        },
        handle_obj_message(message) {
            var objects = {}
            const width = this.positionData["Spielfeld"][0]
            const height = this.positionData["Spielfeld"][1]
            message.objects.forEach(obj => {
                var position = [
                    Math.floor(obj.x * width), Math.floor(obj.y * height),
                    Math.floor(obj.width * width), Math.floor(obj.height * height)
                ]
                objects[obj.obj_id] = position
            });
            this.$refs.gameCanvas.handleObjectPositions(objects)
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
        },
        goals(ros, goalQueue) {

            var goalSubscriber = () => {
                var goalSuccessListener = new ROSLIB.Topic({
                    ros: ros,
                    name: '/goal_success',
                    messageType: 'std_msgs/Bool'
                });

                goalSuccessListener.subscribe(() => { goalQueue.shift() });
            }
            goalSubscriber();

            var goalPublisher = () => {

                var goalListener = new ROSLIB.Topic({
                    ros: ros,
                    name: '/current_goal',
                    messageType: 'std_msgs/String'
                });

                var goal = function (currentGoal) {
                    var msg = new ROSLIB.Message({
                        data: currentGoal
                    });
                    goalListener.publish(msg);
                }

                setInterval(() => {
                    goal(goalQueue[0]);
                }, 1000)
            }
            goalPublisher();
        }
    }
};
</script>