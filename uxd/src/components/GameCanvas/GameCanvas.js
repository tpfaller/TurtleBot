import heroes_json from '../../assets/heroes.json'

export default {
    name: "GameCanvas",
    props: {
        cellSize: Number,
        speed: Number,
        isPlaying: Boolean,
        scores: Number,
        positionData: Array,
        coinSize: Number,
        coinCount: Number,
        order: Array,
        currentStep: Number,
        move: Number
    },
    computed: {
        boardSizePxX() {
            return this.cellSize * this.positionData["Spielfeld"][0];
        },
        boardSizePxY() {
            return this.cellSize * this.positionData["Spielfeld"][1];
        },
        heroes_data() {
            return JSON.parse(JSON.stringify(heroes_json));
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
            coins_collected: 0,
            updatingCanvas: null,
            showCollectedCoins: false,
            moving: false
        }
    },
    mounted() {
        //this.heroes = [this.positionData[this.heroes_data["Captain_America"].name.topdowncamera], this.positionData[this.heroes_data["Iron_Man"].name.topdowncamera], this.positionData[this.heroes_data["Hulk"].name.topdowncamera]];

        this.boardContext = this.$refs.board.getContext("2d");
        window.addEventListener("keydown", this.onKeyPress);

        this.updateCanvas();

        this.clear();
        this.reset();
        this.setupHeroes();
        this.setupBot();

        this.$refs.board.addEventListener('touchstart', (event) => {
            e.preventDefault();
            this.clickedCanvas(event.touches[0].clientX, event.touches[0].clientY);
        }, false);

        this.$refs.board.addEventListener('click', (event) => {
            this.clickedCanvas(event.clientX, event.clientY);
        }, false);
    },
    created() {
        this.reset();
    },
    beforeDestroy() {
        window.removeEventListener("keydown", this.onKeyPress);
    },
    unmounted() {
        clearInterval(this.updatingCanvas);
    },
    watch: {
        isPlaying(value) {
            this.clear();
            if (value) {
                this.reset();
                this.update();
            }
        },
        positionData() {
            console.log(this.positionData["Spielfeld"]);
        },
        currentStep(currentStep) {
            if (currentStep == 2) {
                this.setupObstacles();
                this.drawObstacles();
            } else if (currentStep == 3) {
                this.generateCoins();
                this.setupCoins();
            } else if (currentStep == 6) {
                var that = this;
                var count = 30,
                    timer = setInterval(function () {
                        count--;
                        if (count % 2 == 1) {
                            that.showCollectedCoins = true;
                        }
                        else {
                            that.showCollectedCoins = false;
                        }
                        if (count == 0) clearInterval(timer);
                    }, 350);
                
            }
        },
        move(value){
            console.log("value" + value);
            this.moving = true;
            if(value == null){
                this.moving = false;
            }
            let that = this;
            if(this.moving){
                var timer = setInterval(function () {
                    switch (value) {
                        case 37: that.bot[0] -= 10; break;
                        case 38: that.bot[1] -= 10; break;
                        case 39: that.bot[0] += 10; break;
                        case 40: that.bot[1] += 10; break;
                    }
                    if (!that.moving) clearInterval(timer);
                }, 50);
            }
        }
    },
    methods: {
        reset() {
            //TODO
        },
        handleObjectPositions(positions) {
            for(var i = 0; i < this.heroes.length; i++) {
                var pos = positions[this.heroes_names[i]]
                if(pos != undefined) {
                    this.heroes[i] = pos
                }
            }
            var bot_pos = positions['turtlebot']
            if(bot_pos != undefined) {
                this.bot = bot_pos
            }
            this.obstacles = []
            for(const [obj_id, pos] of Object.entries(positions)) {
                if(obj_id.startsWith('obstacle')) {
                    this.obstacles.push(pos)
                }
            }
        },
        clickedCanvas(clientX, clientY){
            if (this.currentStep != 3) {
                return;
            }
            var rect = this.$refs.board.getBoundingClientRect();
            var x = (clientX - rect.left) / (rect.right - rect.left) * this.$refs.board.width;
            var y = (clientY - rect.top) / (rect.bottom - rect.top) * this.$refs.board.height;

            this.heroes.forEach(function (hero, i) {
                if (x >= hero[0] && x <= hero[0] + hero[2] && y >= hero[1] && y <= hero[1] + hero[3]) {
                    console.log("clicked on: " + this.heroes_names[i])
                    if (!this.order.includes(this.heroes_names[i])) {
                        this.order.push(this.heroes_names[i]);
                        this.$emit('clickedFigure', this.order);
                    } else {
                        let newOrder = this.order.filter(x => x !== this.heroes_names[i]);
                        newOrder.splice(0, 0, this.heroes_names[i]);
                        this.$emit('clickedFigure', newOrder);
                    }
                }
            }, this);
        },
        updateCanvas() {
            this.updatingCanvas = setInterval(this.update, 50);
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
                if(obstacle == undefined) {
                    return
                }
                this.boardContext.beginPath();
                this.boardContext.rect(
                    this.cellSize + obstacle[0],
                    this.cellSize + obstacle[1],
                    this.cellSize + obstacle[2],
                    this.cellSize + obstacle[3]
                );
                this.boardContext.fillStyle = "#d6e2a4";
                this.boardContext.fill();
                this.boardContext.closePath();
            });

            /*
            this.obstacles.forEach(function (obstacle, i) {
                
                if (obstacle[2] < obstacle[3]) {
                    this.boardContext.setTransform(1, 0, 0, 1, obstacle[0], obstacle[1]);
                    this.boardContext.rotate(Math.PI/2);
                    this.boardContext.drawImage(this.obstacles_image, -obstacle[2]/2+10, -obstacle[3]/2, obstacle[3], obstacle[2]);
                    this.boardContext.setTransform(1,0,0,1,0,0);
                    return;
                }
                this.boardContext.drawImage(this.obstacles_image, obstacle[0], obstacle[1], obstacle[2], obstacle[3]);
            }, this);*/
        },
        setupHeroes() {
            this.heroes.forEach(function (hero, i) {

                var image = document.createElement('img');
                image.onload = () => this.drawHeroes();

                image.src = this.heroes_data[this.heroes_names[i]].img.src;

                this.heroes_images[i] = image;
            }, this);
        },
        drawHeroes() {
            this.heroes.forEach(function (hero, i) {
                /*
                let distance = 8 * this.cellSize; //this.distance([this.bot[4][0], this.bot[4][1]], [this.bot[0][0], this.bot[0][1]]);
                let x = this.cellSize * (hero[4][0] - distance / 2);
                let y = this.cellSize * (hero[4][1] - distance / 2);
                let width = this.cellSize * distance;
                let height = this.cellSize * distance;

                this.boardContext.beginPath();
                this.boardContext.rect(x, y, width, height);
                this.boardContext.fillStyle = "red";
                this.boardContext.fill();
                this.boardContext.closePath();

                let aspectRatioHero = this.heroes_data[this.heroes_names[i]].img.width / this.heroes_data[this.heroes_names[i]].img.height
                let aspectRatioBox = hero[2] / hero[3];
                //let width, height, x, y;
                if (aspectRatioBox > aspectRatioHero) {
                    let aspectRatioX = hero[3] / this.heroes_data[this.heroes_names[i]].img.height;
                    width = this.heroes_data[this.heroes_names[i]].img.width * aspectRatioX;
                    height = hero[3];
                    x = hero[0] + hero[2] / 2 - width / 2;
                    y = hero[1];
                } else {
                    let aspectRatioY = hero[2] / this.heroes_data[this.heroes_names[i]].img.width;
                    width = hero[2];
                    height = this.heroes_data[this.heroes_names[i]].img.height * aspectRatioY;
                    x = hero[0];
                    y = hero[1] + hero[3] / 2 - height / 2;
                }
                this.boardContext.drawImage(this.heroes_images[i], x, y, width, height);
                */

                let aspectRatioHero = this.heroes_data[this.heroes_names[i]].img.width / this.heroes_data[this.heroes_names[i]].img.height
                let aspectRatioBox = hero[2] / hero[3];
                let width, height, x, y;
                if (aspectRatioBox > aspectRatioHero) {
                    let aspectRatioX = hero[3] / this.heroes_data[this.heroes_names[i]].img.height;
                    width = this.heroes_data[this.heroes_names[i]].img.width * aspectRatioX;
                    height = hero[3];
                    x = hero[0] + hero[2] / 2 - width / 2;
                    y = hero[1];
                } else {
                    let aspectRatioY = hero[2] / this.heroes_data[this.heroes_names[i]].img.width;
                    width = hero[2];
                    height = this.heroes_data[this.heroes_names[i]].img.height * aspectRatioY;
                    x = hero[0];
                    y = hero[1] + hero[3] / 2 - height / 2;
                }
                this.boardContext.drawImage(this.heroes_images[i], x, y, width, height);

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
            image.src = "../src/assets/images/bot_front.svg";
            this.bot_image = image;
        },
        drawBot() {
            /*
            let distance = 8 * this.cellSize; //this.distance([this.bot[4][0], this.bot[4][1]], [this.bot[0][0], this.bot[0][1]]);
            let x = this.cellSize * (this.bot[4][0] - distance / 2);
            let y = this.cellSize * (this.bot[4][1] - distance / 2);
            let width = this.cellSize * distance;
            let height = this.cellSize * distance;

            this.boardContext.beginPath();
            this.boardContext.rect(x, y, width, height);
            this.boardContext.fillStyle = "red";
            this.boardContext.fill();
            this.boardContext.closePath();

            this.boardContext.drawImage(this.bot_image, x, y, width, height);*/
            this.boardContext.drawImage(this.bot_image, this.bot[0], this.bot[1], this.bot[2], this.bot[3]);
            this.checkCoinCollision();
        },
        distance(center, point) {
            return Math.sqrt(Math.pow(center[0] - point[0], 2) + Math.pow(center[1] - point[1], 2))
        },
        checkCoinCollision() {
            this.coins.forEach(coin => {
                if (coin[2] == true) return;
                /*
                if (this.bot[4][0] + 20 >= coin[0] && this.bot[0] <= coin[0] + this.coinSize
                    && this.bot[1] + this.bot[3] >= coin[1] && this.bot[1] <= coin[1] + this.coinSize) {
                    coin[2] = true;
                    this.coins_collected++;
                    this.$emit('collectedCoin', this.coins_collected);
                }
                */
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
                let randomX = Math.floor(Math.random() * (this.positionData["Spielfeld"][0] - this.coinSize) * this.cellSize);
                let randomY = Math.floor(Math.random() * (this.positionData["Spielfeld"][1] - this.coinSize) * this.cellSize);
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
                if (coin[2] == true && !this.showCollectedCoins) return;
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
                if (!coin[2] || this.showCollectedCoins)
                this.boardContext.drawImage(this.coins_image, coin[0], coin[1], this.coinSize, this.coinSize);
            }, this);
        },
        update() {

            //console.log("update called");

            if (!this.isPlaying) {
                return;
            }

            this.clear();
            if (this.currentStep > 1) {
                this.drawObstacles();
            }
            if (this.currentStep > 2) {
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

            /*
            switch (event.keyCode) {
                case 37: this.bot[4][0] -= 2; break;
                case 38: this.bot[4][1] -= 2; break;
                case 39: this.bot[4][0] += 2; break;
                case 40: this.bot[4][1] += 2; break;
            }*/

            switch (event.keyCode) {
                case 37: this.bot[0] -= 10; break;
                case 38: this.bot[1] -= 10; break;
                case 39: this.bot[0] += 10; break;
                case 40: this.bot[1] += 10; break;
            }

            //this.update();
        }
    }
};
