export default {
    name: "SnakeCanvas",
    props: {
        cellSize: Number,
        speed: Number,
        isPlaying: Boolean,
        scores: Number,
        positionData: Array
    },
    computed: {
        boardSizePxX(axis) {
            return this.cellSize * this.positionData["Spielfeld"][0];
        },
        boardSizePxY() {
            return this.cellSize * this.positionData["Spielfeld"][1];
        }
    },
    data() {
        return {
            bot: this.positionData["Objekte"]["TurtleBot"],
            bot_image: Image,
            heroes: [this.positionData["Objekte"]["Captain_America"], this.positionData["Objekte"]["Iron_Man"], this.positionData["Objekte"]["Hulk"]],
            heroes_names: ["Captain_America", "Iron_Man", "Hulk"],
            heroes_images: Image
        }
    },
    mounted() {
        this.boardContext = this.$refs.board.getContext("2d");
        window.addEventListener("keydown", this.onKeyPress);
        this.clear();
        this.reset();
        this.setupHeroes();
        this.setupBot();
        this.move();

        console.log(this.positionData);
    },
    created() {
        this.reset();
    },
    beforeDestroy() {
        window.removeEventListener("keydown", this.onKeyPress);
    },
    watch: {
        isPlaying(value) {
            this.clear();
            if (value) {
                this.reset();
                this.move();
            }
        },
        positionData() {
            console.log(this.positionData["Spielfeld"]);
        }
    },
    methods: {
        reset() {
            //TODO
        },
        drawObstacles() {
            var obstacles = [this.positionData["Objekte"]["Hindernis_1"], this.positionData["Objekte"]["Hindernis_2"], this.positionData["Objekte"]["Hindernis_3"]];

            obstacles.forEach(obstacle => {
                this.boardContext.beginPath();
                this.boardContext.rect(
                    this.cellSize + obstacle[0],
                    this.cellSize + obstacle[1],
                    this.cellSize + obstacle[2],
                    this.cellSize + obstacle[3]
                );
                this.boardContext.fillStyle = "red";
                this.boardContext.fill();
                this.boardContext.closePath();
            });
        },
        setupHeroes() {
            this.heroes.forEach(function (hero, i) {

                var image = document.createElement('img');
                image.onload = () => this.drawHeroes();

                image.src = "../src/assets/images/" + this.heroes_names[i] + ".svg";

                this.heroes_images[i] = image;
            }, this);
        },
        drawHeroes() {
            this.heroes.forEach(hero => {
                this.boardContext.beginPath();
                this.boardContext.rect(
                    this.cellSize + hero[0],
                    this.cellSize + hero[1],
                    this.cellSize + hero[2],
                    this.cellSize + hero[3]
                );
                this.boardContext.fillStyle = "yellow";
                this.boardContext.fill();
                this.boardContext.closePath();
            }, this);

            this.heroes.forEach(function (hero, i) {
                this.boardContext.drawImage(this.heroes_images[i], hero[0], hero[1], hero[2], hero[3]);
            }, this);
        },
        setupBot() {
            var image = document.createElement('img');
            image.onload = () => this.drawBot();

            image.src = "../src/assets/images/Bot.svg";

            this.bot_image = image;
        },
        drawBot() {
            this.boardContext.beginPath();
            this.boardContext.rect(
                this.cellSize + this.bot[0],
                this.cellSize + this.bot[1],
                this.cellSize + this.bot[2],
                this.cellSize + this.bot[3]
            );
            this.boardContext.fillStyle = "yellow";
            this.boardContext.fill();
            this.boardContext.closePath();

            this.boardContext.drawImage(this.bot_image, this.bot[0], this.bot[1], this.bot[2], this.bot[3]);
        },
        move() {
            if (!this.isPlaying) {
                return;
            }

            this.clear();
            this.drawObstacles();
            this.drawHeroes();
            this.drawBot();
        },
        clear() {
            this.boardContext.clearRect(0, 0, this.boardSizePxX, this.boardSizePxY);
        },
        onKeyPress(event) {

            switch(event.keyCode){
                case 37: this.bot[0] -= 10; break;
                case 38: this.bot[1] -= 10; break;
                case 39: this.bot[0] += 10; break;
                case 40: this.bot[1] += 10; break;
            }

            this.move();
        }
    }
};
