<template>
    <section class="content about">
        <h1 class="title">{{ $t('about.title') }}</h1>
        <div class="col-container col-2">
            <div>
                <img src="../../src/assets/images/About/gruppe.jpeg">
            </div>
            <div>
                <h2>{{ $t('about.title') }}</h2>
                <p>{{ $t('about.summary') }}</p>
            </div>
        </div>
        <div class="col-container col-2">
            <div>
                <img src="../../src/assets/images/About/ki1.jpeg">
                <img src="../../src/assets/images/About/ki2.jpeg">
                <h3 class="image-title">{{ $t('about.ki.names') }}</h3>
            </div>
            <div>
                <h2>{{ $t('about.ki.title') }}</h2>
                <p>{{ $t('about.ki.content') }}</p>
            </div>
        </div>
        <div class="col-container col-2">
            <div>
                <img src="../../src/assets/images/About/inf.jpeg">
                <h3 class="image-title">{{ $t('about.inf.names') }}</h3>
            </div>
            <div>
                <h2>{{ $t('about.inf.title') }}</h2>
                <p>{{ $t('about.inf.content') }}</p>
            </div>
        </div>
        <div class="col-container col-2">
            <div>
                <img src="../../src/assets/images/About/uxd.jpeg">
                <h3 class="image-title">{{ $t('about.uxd.names') }}</h3>
            </div>
            <div>
                <h2>{{ $t('about.uxd.title') }}</h2>
                <p>{{ $t('about.uxd.content') }}</p>
            </div>
        </div>
    </section>
</template>

<style>
img {
    width: 100%;
}
</style>
