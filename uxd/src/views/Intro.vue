<script setup>
import coinhunter_intro from "../assets/coinhunter_intro";
</script>

<template>
    <section class="content intro">
        <carousel :items-to-show="1">
            <slide v-for="item in coinhunter_intro" :key="item">
                <div class="carousel__item">
                    <img :src="item.src">
                    <h3 class="title">{{ $t(item.title) }}</h3>
                    <p>{{ $t(item.content) }}</p>
                </div>
            </slide>

            <template #addons>
                <navigation />
            </template>
        </carousel>
        <router-link to="/game" custom v-slot="{ navigate }">
            <button @click="navigate" @keypress.enter="navigate">{{ $t('intro.start') }}</button>
        </router-link>
    </section>
</template>

<script>
import 'vue3-carousel/dist/carousel.css';
import { Carousel, Slide, Pagination, Navigation } from 'vue3-carousel';

export default {
    name: 'App',
    components: {
        Carousel,
        Slide,
        Pagination,
        Navigation,
    },
    data() {
        return {
            items: [{ message: 'Foo' }, { message: 'Bar' }]
        }
    }
};
</script>

<style>
.carousel__item {
  min-height: 200px;
  width: 100%;
  background-color: var(--white);
  color:  var(--text-color);
  display: flex;
  flex-direction: column;
}

.carousel__item > img{
    height: 50vh;
    object-fit: cover;
}

.carousel__prev--in-active,
.carousel__next--in-active {
  display: none;
}

.carousel__prev, .carousel__next{
    background-color: var(--text-color-primary);
}

.carousel__viewport{
    border-radius: 2rem;
    border: 1px solid var(--border-color);
}
</style>