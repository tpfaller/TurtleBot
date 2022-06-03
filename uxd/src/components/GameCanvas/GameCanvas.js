export default {
    name: "SnakeCanvas",
    props: {
        cellSize: Number,
        speed: Number,
        isPlaying: Boolean,
        scores: Number,
        positionData: Array,
        coinSize: Number,
        coinCount: Number,
        order: Array,
        step: Number
    },
    computed: {
        boardSizePxX() {
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
            heroes_reached: [false, false, false],
            heroes_names: ["Captain_America", "Iron_Man", "Hulk"],
            heroes_images: Image,
            obstacles: [this.positionData["Objekte"]["Hindernis_1"], this.positionData["Objekte"]["Hindernis_2"], this.positionData["Objekte"]["Hindernis_3"]],
            obstacles_image: Image,
            coins: [],
            coins_image: Image,
            coins_collected: 0
        }
    },
    mounted() {
        this.boardContext = this.$refs.board.getContext("2d");
        window.addEventListener("keydown", this.onKeyPress);

        this.clear();
        this.reset();
        //this.setupObstacles();
        this.setupHeroes();
        this.setupBot();
        //this.generateCoins();
        //this.setupCoins();
        this.move();

        //TODO: erst beim richtigen Schritt zulassen
        let that = this;
        this.$refs.board.addEventListener('click', function (event) {

            var rect = that.$refs.board.getBoundingClientRect();
            var x = (event.clientX - rect.left) / (rect.right - rect.left) * that.$refs.board.width;
            var y = (event.clientY - rect.top) / (rect.bottom - rect.top) * that.$refs.board.height;

            that.heroes.forEach(function (hero, i) {
                if (x >= hero[0] && x <= hero[0] + hero[2] && y >= hero[1] && y <= hero[1] + hero[3]) {
                    console.log("clicked on: " + that.heroes_names[i])
                    if (!that.order.includes(that.heroes_names[i])) {
                        that.order.push(that.heroes_names[i]);
                    } else {
                        //TODO: Reihenfolge ändern
                    }
                    that.$emit('clickedFigure', that.order);
                }
            }, this);

        }, false);

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
        },
        step(currentStep) {
            if (currentStep == 2) {
                this.setupObstacles();
                this.drawObstacles();
            } else if (currentStep == 3) {
                this.generateCoins();
                this.setupCoins();
            }
        }
    },
    methods: {
        reset() {
            //TODO
        },
        setupObstacles() {
            this.obstacles.forEach(function (obstacle, i) {

                var image = document.createElement('img');
                image.onload = () => this.drawObstacles();

                image.src = "../src/assets/images/Obstacle.svg";

                this.obstacles_image = image;
            }, this);
        },
        drawObstacles() {
            this.obstacles.forEach(obstacle => {
                this.boardContext.beginPath();
                this.boardContext.rect(
                    this.cellSize + obstacle[0],
                    this.cellSize + obstacle[1],
                    this.cellSize + obstacle[2],
                    this.cellSize + obstacle[3]
                );
                this.boardContext.fillStyle = "transparent";
                this.boardContext.fill();
                this.boardContext.closePath();
            });

            this.obstacles.forEach(function (obstacle, i) {
                /*
                if (obstacle[2] < obstacle[3]) {
                    this.boardContext.translate(this.positionData["Spielfeld"][0]/2,this.positionData["Spielfeld"][1]/2);
                    this.boardContext.rotate(Math.PI/2);
                    this.boardContext.drawImage(this.obstacles_image, obstacle[0], obstacle[1], obstacle[2], obstacle[3]);
                    this.boardContext.rotate(-Math.PI/2);
                    this.boardContext.translate(-this.positionData["Spielfeld"][0]/2,-this.positionData["Spielfeld"][1]/2);
                    return;
                }*/
                this.boardContext.drawImage(this.obstacles_image, obstacle[0], obstacle[1], obstacle[2], obstacle[3]);
            }, this);
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
                this.boardContext.fillStyle = "transparent";
                this.boardContext.fill();
                this.boardContext.closePath();
            }, this);

            this.heroes.forEach(function (hero, i) {
                this.boardContext.drawImage(this.heroes_images[i], hero[0], hero[1], hero[2], hero[3]);
            }, this);
        },
        checkHeroCollision() {
            this.heroes.forEach(function (hero, i) {
                if (this.heroes_reached[this.order.indexOf(this.heroes_names[i])]) return;
                if (this.bot[0] + this.bot[2] >= hero[0] && this.bot[0] <= hero[0] + hero[2] && this.bot[1] + this.bot[3] >= hero[1] && this.bot[1] <= hero[1] + hero[3]) {
                    this.heroes_reached[this.order.indexOf(this.heroes_names[i])] = true;
                    console.log("heroes reached " + this.heroes_reached);
                    this.$emit('updateReachedHeroes', this.heroes_reached);
                }
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
            this.boardContext.fillStyle = "transparent";
            this.boardContext.fill();
            this.boardContext.closePath();

            this.boardContext.drawImage(this.bot_image, this.bot[0], this.bot[1], this.bot[2], this.bot[3]);
            this.checkCoinCollision();
        },
        checkCoinCollision() {
            this.coins.forEach(coin => {
                if (coin[2] == true) return;
                if (this.bot[0] + this.bot[2] >= coin[0] && this.bot[0] <= coin[0] + this.coinSize 
                    && this.bot[1] + this.bot[3] >= coin[1] && this.bot[1] <= coin[1] + this.coinSize) {
                    coin[2] = true;
                    this.coins_collected++;
                    this.$emit('collectedCoin', this.coins_collected);
                }
            }, this);
        },
        isNotEmpty(x, y, width, height) {
            let notEmpty = false;
            notEmpty = this.coins.some(coin => {
                if (x + width >= coin[0] && x <= coin[0] + this.coinSize && y + height >= coin[1] && y <= coin[1] + this.coinSize) {
                    console.log("Coin in way!");
                    return true;
                }
            }, this);
            if (notEmpty) return true;
            notEmpty = this.obstacles.some(obstacle => {
                if (x + width >= obstacle[0] && x <= obstacle[0] + obstacle[2] && y + height >= obstacle[1] && y <= obstacle[1] + obstacle[3]) {
                    console.log("Obstacle in way!");
                    return true;
                }
            });
            if (notEmpty) return true;
            notEmpty = this.heroes.some(hero => {
                if (x + width >= hero[0] && x <= hero[0] + hero[2] && y + height >= hero[1] && y <= hero[1] + hero[3]) {
                    console.log("Hero in way!");
                    return true;
                }
            });
            if (notEmpty) return true;
            if (x + width >= this.bot[0] && x <= this.bot[0] + this.bot[2] && y + height >= this.bot[1] && y <= this.bot[1] + this.bot[3]) {
                console.log("Bot in way!");
                return true;
            }
        },
        generateCoins() {
            var generatedCoins = 0;
            while (generatedCoins < this.coinCount) {
                let randomX = Math.floor(Math.random() * (this.positionData["Spielfeld"][0] - this.coinSize));
                let randomY = Math.floor(Math.random() * (this.positionData["Spielfeld"][1] - this.coinSize));
                if (this.isNotEmpty(randomX, randomY, this.coinSize, this.coinSize)) {
                    console.log("Skipped!");
                    continue;
                }
                let generatedCoin = [randomX, randomY, false];
                this.coins.push(generatedCoin);
                generatedCoins++;
            }
            console.log("Generated coins: " + this.coins);
        },
        setupCoins() {
            var image = document.createElement('img');
            image.onload = () => this.drawCoins();

            image.src = "../src/assets/images/Coin.svg";

            this.coins_image = image;
        },
        drawCoins() {
            this.coins.forEach(coin => {
                if (coin[2] == true) return;
                this.boardContext.beginPath();
                this.boardContext.rect(
                    this.cellSize + coin[0],
                    this.cellSize + coin[1],
                    this.cellSize + this.coinSize,
                    this.cellSize + this.coinSize
                );
                this.boardContext.fillStyle = "transparent";
                this.boardContext.fill();
                this.boardContext.closePath();
            }, this);

            this.coins.forEach(coin => {
                if (coin[2] == true) return;
                this.boardContext.drawImage(this.coins_image, coin[0], coin[1], this.coinSize, this.coinSize);
            }, this);
        },
        move() {
            if (!this.isPlaying) {
                return;
            }

            this.clear();
            if (this.step > 1) {
                this.drawObstacles();
            }
            if (this.step > 2) {
                this.drawCoins();
            }

            this.drawHeroes();
            this.drawBot();

            this.checkHeroCollision();

        },
        clear() {
            this.boardContext.clearRect(0, 0, this.boardSizePxX, this.boardSizePxY);
        },
        onKeyPress(event) {

            switch (event.keyCode) {
                case 37: this.bot[0] -= 10; break;
                case 38: this.bot[1] -= 10; break;
                case 39: this.bot[0] += 10; break;
                case 40: this.bot[1] += 10; break;
            }

            this.move();
        }
    }
};