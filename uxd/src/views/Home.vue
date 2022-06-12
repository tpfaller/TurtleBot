<template>
    <section class="content center-v">
        <div class="col-container col-2">
            <div class="home-col">
                <h1 class="title">{{ $t('home.title') }}</h1>
                <h2>{{ $t('home.subtitle') }}</h2>
                <div>{{ $t('home.description') }}</div>
                <router-link to="/intro" custom v-slot="{ navigate }">
                    <button @click="navigate" @keypress.enter="navigate">{{ $t('button.start') }}</button>
                </router-link>
            </div>
            <div>
                <img src="/src/assets/images/Home.svg">
            </div>
        </div>
    </section>
</template>