<script setup>
import coinhunter_intro from "../assets/coinhunter_intro";
</script>

<template>
    <carousel :items-to-show="1">
        <slide v-for="item in coinhunter_intro" :key="item">
            <div class="carousel__item">
                <lottie-animation ref="anim" :loop="true" :animationData="item.src" />
                <div>
                    <h3 class="title">{{ $t(item.title) }}</h3>
                    <p>{{ $t(item.content) }}</p>
                </div>
            </div>
        </slide>

        <template #addons>
            <navigation />
        </template>
    </carousel>
</template>

<script>
import 'vue3-carousel/dist/carousel.css';
import { Carousel, Slide, Pagination, Navigation } from 'vue3-carousel';

export default {
    name: 'IntroSlider',
    components: {
        Carousel,
        Slide,
        Pagination,
        Navigation
    }
}
</script>

<style>
.carousel__item {
    min-height: 200px;
    width: 100%;
    color: var(--text-color);
    display: flex;
    flex-direction: column;
    /*gap: 2rem;*/
    padding: 2rem;
}

.carousel__item>img {
    height: 30vh;
    object-fit: contain;
}

.carousel__item>* {
    pointer-events: none;
    user-select: none;
}

.carousel__prev--in-active,
.carousel__next--in-active {
    display: none;
}

.carousel__prev,
.carousel__next {
    background-color: var(--text-color-primary);
}

.carousel__viewport {
    border-radius: 2rem;
    background-color: var(--white);
    box-shadow: 0px 0px 0px 1px var(--border-color);
}

.carousel__slide {
    align-items: flex-start;
}

.intro svg {
    max-height: 30vh;
}

.intro svg>g {
    transform: translate(-90px, -110px)scale(1.15);
}
</style>