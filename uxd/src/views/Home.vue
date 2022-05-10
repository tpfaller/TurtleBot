<template>
    <section class="content">
        <div class="col-container space-between">
            <div>
                <h1>{{ $t('home.title') }}</h1>
                <h2>{{ $t('home.subtitle') }}</h2>
            </div>
            <div>
                <router-link to="/about" custom v-slot="{ navigate }">
                    <button @click="navigate" @keypress.enter="navigate">{{ $t('about.title') }}</button>
                </router-link>
            </div>
        </div>
        <div class="col-container games-container">
            <router-link class="game-container" to="/intro">
                <img src="https://www.thi.de/fileadmin/daten/UeberUns/Campus/Campus_THIBau.jpg">
                <h3>{{ $t('turtlebot.title') }}</h3>
            </router-link>
        </div>
    </section>
</template>

<style>
.game-container{
    text-decoration: unset;
    color: unset;
}

.game-container img{
    aspect-ratio: 1/1;
    object-fit: cover;
    max-width: 15rem;
}

.games-container{
    flex-grow: 1;
    align-items: center;
    justify-content: center;
}

.game-container > h3{
    text-align: center;
}
</style>