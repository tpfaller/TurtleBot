import constants from "./constants";
import { onMounted } from 'vue'

export default {
    name: "SnakeCanvas",
    props: {
        cellSize: Number,
        boardSize: Array,
        speed: Number,
        isPlaying: Boolean,
        stop: Function,
        scores: Number,
        obstacles: Array
    },
    computed: {
        boardSizePxX(axis) {
            return this.cellSize * this.boardSize[0];
        },
        boardSizePxY() {
            return this.cellSize * this.boardSize[1];
        }
    },
    mounted() {
        this.boardContext = this.$refs.board.getContext("2d");
        window.addEventListener("keydown", this.onKeyPress);
        this.clear();
        this.resetSnake();
        this.move();
        this.setupObstacles();
        this.setupHeroes();
        console.log(this.obstacles);
    },
    created() {
        this.resetSnake();
    },
    beforeDestroy() {
        window.removeEventListener("keydown", this.onKeyPress);
    },
    watch: {
        isPlaying(value) {
            this.clear();
            if (value) {
                this.resetSnake();
                this.move();
            }
        }
    },
    methods: {
        resetSnake() {
            this.snake = [
                {
                    x: this.getMiddleCellX(),
                    y: this.getMiddleCellY()
                }
            ];
            const randomDirectionIndex = Math.floor(Math.random() * 4);
            this.direction = constants[randomDirectionIndex];
            this.targetCell = null;
        },
        setupObstacles() {
            this.obstacles.forEach(obstacle => {
                this.boardContext.beginPath();
                this.boardContext.rect(
                    this.cellSize + obstacle[0],
                    this.cellSize + obstacle[1],
                    this.cellSize + 100,
                    this.cellSize + 100
                );
                this.boardContext.fillStyle = "red";
                this.boardContext.fill();
                this.boardContext.closePath();
            });
        },
        setupHeroes() {
            let p = new Path2D('M10 10 h 180 v 80 h -80 Z');
            this.boardContext.translate(100, 100);
            this.boardContext.fill(p);
            var image = new Image();
            image.onload = function () {
                this.boardContext.drawImage(this, 0, 0);
            }
            image.src = "../src/assets/powerranger.svg";
        },
        getMiddleCellX() {
            return Math.round(this.boardSize[0] / 2);
        },
        getMiddleCellY() {
            return Math.round(this.boardSize[1] / 2);
        },
        move() {
            if (!this.isPlaying) {
                return;
            }

            this.clear();
            this.setupObstacles();
            //this.setTargetCell();

            const newHeadCell = {
                x: this.snake[0].x + this.direction.move.x,
                y: this.snake[0].y + this.direction.move.y
            };

            if (
                this.isCellOutOfBoard(newHeadCell) ||
                this.amountCellsInSnake(this.snake[0]) > 1
            ) {
                this.stop();
                alert(`Game over! You've scored ${this.scores} points.`);
            }

            /*
            if (this.isTargetNewHead()) {
                this.snake.unshift(this.targetCell);
                //this.targetCell = null;
            } else {
                this.snake.unshift(newHeadCell);
                this.snake.pop();
            }*/

            this.snake.unshift(newHeadCell);
            this.snake.pop();

            this.boardContext.beginPath();
            this.snake.forEach(this.drawCell);
            this.boardContext.closePath();
        },
        clear() {
            this.boardContext.clearRect(0, 0, this.boardSizePxX, this.boardSizePxY);
        },
        drawCell({ x, y }) {
            this.boardContext.rect(
                x * this.cellSize + 100,
                y * this.cellSize + 100,
                this.cellSize + 100,
                this.cellSize + 100
            );
            this.boardContext.fillStyle = "black";
            this.boardContext.fill();
        },
        isCellOutOfBoard({ x, y }) {
            return x < 0 || y < 0 || x >= this.boardSize[0] || y >= this.boardSize[1];
        },
        onKeyPress(event) {
            const newDirection = constants.find(c => c.keyCode === event.keyCode);
            if (!newDirection) {
                return;
            }
            this.direction = newDirection;
            this.move();
        },
        /*
        setTargetCell() {
            if (!this.targetCell) {
                let targetCell = this.getRandomCell();
                while (this.amountCellsInSnake(targetCell) > 0) {
                    targetCell = this.getRandomCell;
                }
                this.targetCell = targetCell;
            }

            this.boardContext.beginPath();
            this.boardContext.rect(
                this.targetCell.x * this.cellSize,
                this.targetCell.y * this.cellSize,
                this.cellSize,
                this.cellSize
            );
            this.boardContext.fillStyle = "red";
            this.boardContext.fill();
            this.boardContext.closePath();
        },
        getRandomCell() {
            return {
                x: Math.floor(Math.random() * this.boardSizeX),
                y: Math.floor(Math.random() * this.boardSizeY)
            };
        },*/
        amountCellsInSnake(cell) {
            return this.snake.filter(({ x, y }) => x === cell.x && y === cell.y)
                .length;
        },
        isTargetNewHead() {
            return (
                this.snake[0].x + this.direction.move.x === this.targetCell.x &&
                this.snake[0].y + this.direction.move.y === this.targetCell.y
            );
        }
    }
};
