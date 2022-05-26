<template>
    <div class="results">
        <div class="result">
            <span class="title">{{ $t('coinhunter.game.results.coins') }}</span>
            <span class="content">{{score}}</span>
        </div>
        <div class="result">
            <span class="title">{{ $t('coinhunter.game.results.time') }}</span>
            <span class="content">{{duration}}</span>
        </div>
    </div>
</template>

<script>
export default {
    props: {
        score: Number,
        duration: String
    }
}
</script>