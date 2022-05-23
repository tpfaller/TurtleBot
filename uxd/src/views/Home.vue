<template>
    <section class="content center-v">
        <div class="col-container col-2">
            <div>
                <h1 class="title">{{ $t('home.title') }}</h1>
                <h2 class="subtitle">{{ $t('home.subtitle') }}</h2>
                <div>{{ $t('home.description') }}</div>
            </div>
            <div class="game-col">
                <img src="https://www.thi.de/fileadmin/daten/UeberUns/Campus/Campus_THIBau.jpg">
                <router-link to="/intro" custom v-slot="{ navigate }">
                    <button @click="navigate" @keypress.enter="navigate">{{ $t('home.start-game') }}</button>
                </router-link>
            </div>
        </div>
    </section>
</template>