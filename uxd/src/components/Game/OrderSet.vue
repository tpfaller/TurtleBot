<template>
    <draggable class="heroes-order" group="people" @start="drag = true" @end="drag = false"
        item-key="id" tag="transition-group" v-model="order" v-bind="dragOptions" :component-data="{ tag: 'div', name: 'flip-list', type: 'transition' }">
        <template #item="{ element }">
            <div>
                <img :src="list[element].img.src">
                {{ $t(list[element].name.string) }}
                <div class="img-container">
                    <svg xmlns="http://www.w3.org/2000/svg" width="16" height="16" fill="currentColor"
                        class="bi bi-grip-vertical" viewBox="0 0 16 16">
                        <path
                            d="M7 2a1 1 0 1 1-2 0 1 1 0 0 1 2 0zm3 0a1 1 0 1 1-2 0 1 1 0 0 1 2 0zM7 5a1 1 0 1 1-2 0 1 1 0 0 1 2 0zm3 0a1 1 0 1 1-2 0 1 1 0 0 1 2 0zM7 8a1 1 0 1 1-2 0 1 1 0 0 1 2 0zm3 0a1 1 0 1 1-2 0 1 1 0 0 1 2 0zm-3 3a1 1 0 1 1-2 0 1 1 0 0 1 2 0zm3 0a1 1 0 1 1-2 0 1 1 0 0 1 2 0zm-3 3a1 1 0 1 1-2 0 1 1 0 0 1 2 0zm3 0a1 1 0 1 1-2 0 1 1 0 0 1 2 0z" />
                    </svg>
                </div>
            </div>
        </template>
    </draggable>
</template>

<script>
import heroes from "../../assets/heroes.json";
import draggable from 'vuedraggable'
export default {
    props: {
        order: Array
    },
    components: {
        draggable
    },
    watch: {
        order(newOrder){
            this.$emit('updateOrder', newOrder);
        }
    },
    computed: {
        dragOptions() {
            return {
                animation: 200,
                group: "description",
                disabled: false,
                ghostClass: "ghost"
            };
        }
    },
    data() {
        return {
            enabled: true,
            list: JSON.parse(JSON.stringify(heroes)),
            drag: false
        }
    }
}
</script>